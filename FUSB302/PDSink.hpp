/**
 * @file    PDSink.hpp
 * @brief   FUSB302 SINK 模式 PD 驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  定位 (配合 907-PowerOneT6 的硬件架构)
 * ============================================================================
 *  A1 = SW3526S 快充多协议输出芯片 (USBA 口), 平时自己跑协议快充;
 *  FUSB302 的 CC 脚直连 SW3526S 的 CC —— 进入「可调电源模式」时, 用
 *  FUSB302 的 SINK 走 CC/PD 向 SW3526S-A1 申请 PPS, 把 USBA 口变成
 *  电压可调输出 (USBA 转 DC 线带 12V 路由器 / 20V 显示器等)。
 *  本驱动只做【受电 (Sink)】一侧: 检测 CC 上的 Rp → 收 Source Capabilities
 *  → 选 PDO 构造 RDO → 发 Request → 等 Accept → 等 PS_RDY → 合同生效。
 *
 * ============================================================================
 *  设计 (与本库 SW3526 / FUSB302 同款: 直绑工程头, 阻塞 I2C, 无堆分配)
 * ============================================================================
 *  - 轮询式 (tick 驱动) + INT 脚友好: begin() 会配好中断掩码
 *    (Mask1 全屏蔽; Maska 解 HARDRST/TXSENT/HARDSENT/RETRYFAIL; Maskb 解 GCRCSENT)
 *    并把 CONTROL0.INT_MASK 清 0 打开 INT 脚 (板: PA5, 外部 10K 上拉, 下降沿);
 *    tick 每次都把 INTERRUPT/INTERRUPTA/INTERRUPTB 读掉 (读即清), 所以 INT 脚
 *    不会卡在低电平。上层可用 EXINT → 立刻调 tick(), 也可只靠周期 tick。
 *  - 全部 PD 报文用硬件自动重发 (CONTROL3 AUTO_RETRY ×3) + 自动 GoodCRC
 *    (SWITCHES1.AUTO_CRC); TX 失败靠「等不到回包超时重发」兜底。
 *  - 无动态内存; Source Capabilities 最多 7 个 PDO 存静态数组。
 *
 * ============================================================================
 *  状态机 (tick(nowMs) 每次主循环调用)
 * ============================================================================
 *    IDLE       未启动 (或 stop() 后, CC 上 PDWN 下拉已释放)
 *    DETECT     交替 MEAS_CC1/CC2 读 STATUS0.BC_LVL 扫 Rp (5ms 一轮×2)
 *               连续两轮稳定 且 max(bc1,bc2)>0 → attach, 取大者为极性
 *    WAIT_CAP   等 Source Capabilities (t=10s; 超时先换另一根 CC 极性再试
 *               (仅当两根 CC 都有 Rp 的直连拓扑), 再发 Get_Source_Cap,
 *               仍无 → FAULT)
 *    CAPS       已收到 Source Caps; setAutoRequest(false) 时停在这里等上层
 *               requestNow() (给用户看 PDO 列表用), 否则直接进 SEND_REQ
 *    SEND_REQ   按 setTarget() 选 PDO 构造 RDO, 组 Request 压 TX FIFO
 *    WAIT_ACCEPT 等 Accept/Reject/Wait (t=2.7s, 参考 PD_T_SENDER_RESPONSE;
 *               超时重发 Request ×3 → FAULT)
 *    WAIT_PS_RDY 等 PS_RDY (t=5s; 到 → READY, 合同生效)
 *    READY      合同生效:
 *               - PPS: 每 5s 周期重发 Request (tPPSRequest, 断 PPS 会掉)
 *               - target 变了 (setTarget) → 重新 SEND_REQ 协商
 *               - CC 的 BC_LVL 连续两轮 (500ms) 归零 → 拔出 → DETECT
 *    FAULT      出错停机 (fault() 给原因), 需 start() 重新走 DETECT
 *
 *  收到 Hard Reset (INTERRUPTA.I_HARDRST) → 清合同回 WAIT_CAP (源端会重发 caps)。
 *
 * ============================================================================
 *  典型用法
 * ============================================================================
 *    LoveFinderLib::FUSB302  ct;
 *    LoveFinderLib::PDSink   pd;
 *    ct.init(LF_I2C_BUS1, 0x22, "FUSB302");
 *    pd.init(ct);
 *    if (pd.begin() && ct.probe()) {
 *        pd.setTarget(12000, 1000);   // 12V / 1A
 *        pd.start();
 *    }
 *    // 主循环:
 *    pd.tick(lf_get_tick());
 *    if (pd.isReady()) { use(pd.contractMv(), pd.contractMa()); }
 *
 * ============================================================================
 *  PD 报文 / PDO 位域速查 (USB PD 3.0)
 * ============================================================================
 *  Header (16bit LE): [2:0] type  [3] powerRole  [5:4] specRev  [6] dataRole
 *                     [11:9] msgId (0..7 环回)  [14:12] numObj  [15] extended
 *    Data:  1=SourceCaps 2=Request 4=SinkCaps      Ctrl: 1=GoodCRC 3=Accept
 *           4=Reject 6=PS_RDY 7=Get_Source_Cap 0xC=Wait 0xD=SoftReset
 *  Fixed PDO  (type=00): [9:0]=imax 10mA  [20:10]=v 50mV
 *  PPS APDO   (type=11, subtype=00): [6:0]=imax 50mA  [15:8]=vmin 100mV
 *                                    [24:17]=vmax 100mV
 *  RDO Fixed: [9:0]=max 10mA [19:10]=op 10mA [24] noUsbSuspend [31:28]=objPos(1基)
 *  RDO PPS:   [6:0]=op 50mA  [20:9]=v 20mV   [24] noUsbSuspend [31:28]=objPos
 *  TX FIFO: {0x12×3,0x13, PACKSYM|len, hdr+data, 0xFF JAM_CRC,0x14 EOP,
 *            0xFE TXOFF, 0xA1 TXON(触发发送)}
 */
#ifndef PDSINK_HPP
#define PDSINK_HPP

#include "FUSB302.hpp"

#include <cstdint>

namespace LoveFinderLib {

/*============================================================================
 * 超时常量 (ms) —— 可在 lf_config.h 里覆盖
 *============================================================================*/
#ifndef PD_SINK_T_WAIT_CAP_MS
#define PD_SINK_T_WAIT_CAP_MS      1500U  /* 等 Source Capsabilities (每级 1.5s, 快速出结果) */
#endif

#ifndef PD_SINK_T_SENDER_RESP_MS
#define PD_SINK_T_SENDER_RESP_MS   2700U  /* Request→Accept (参考 PD_T_SENDER_RESPONSE) */
#endif

#ifndef PD_SINK_T_PS_RDY_MS
#define PD_SINK_T_PS_RDY_MS        5000U  /* Accept→PS_RDY */
#endif

#ifndef PD_SINK_T_PPS_REQ_MS
#define PD_SINK_T_PPS_REQ_MS       5000U  /* PPS 周期重请求 (tPPSRequest) */
#endif

/*============================================================================
 * PDSink — SINK 侧 PD 协商状态机
 *============================================================================*/
class PDSink {
public:
    enum class State : uint8_t {
        IDLE, DETECT, WAIT_CAP, CAPS, SEND_REQ, WAIT_ACCEPT, WAIT_PS_RDY, READY, FAULT
    };

    /** 故障原因 (state()==FAULT 时有效) */
    enum class Fault : uint8_t {
        NONE,          ///< 无故障
        INIT_FAIL,     ///< begin() 配置写失败 (I2C / 芯片不在)
        NO_PDO,        ///< 目标电压在所有 PDO 里都找不到 (源端没有 PPS 且无同压固定 PDO)
        NO_RESPONSE,   ///< 等不到 Accept / PS_RDY (Request 重发 3 次仍无回包)
        NOSRCS,        ///< 10s×2 等不到 Source Capabilities
        I2C_ERROR      ///< 收发过程中 I2C 出错
    };

    PDSink() = default;

    void init(FUSB302 &fusb) noexcept { m_fusb = &fusb; }
    void init(FUSB302 *fusb) noexcept { m_fusb = fusb; }

    /**
     * @brief 上电配置序列 (SW_RES / POWER / MASK / CONTROL / TX+RX flush)。
     * @return true = 配置写全部成功。应在 ct.probe() 前后调用一次。
     */
    bool begin() noexcept;

    /**
     * @brief 进入 DETECT (CC 检测)。start 前先 setTarget() 设目标。
     * 会打开 CC1/CC2 的 PDWN 下拉 (Rd), 让 SW3526S 察觉 sink 接入。
     */
    void start() noexcept;

    /** @brief 停机回 IDLE, 释放 CC 下拉 (模拟拔出)。 */
    void stop() noexcept;

    /**
     * @brief 状态机推进 (每次主循环调用, 非阻塞)。
     * @param nowMs 单调毫秒时基 (lf_get_tick())
     */
    void tick(uint32_t nowMs) noexcept;

    /**
     * @brief 设定请求目标 (mV / mA)。任意时刻可调:
     *        - READY 时目标变了 → 置 m_reneg, 下一拍自动重协商;
     *        - 未 READY 时只记录, 收到 Source Caps 后按它选 PDO。
     *        找不到匹配 PDO → 后续 FAULT(NO_PDO)。
     */
    void setTarget(int mv, int ma) noexcept
    {
        if (mv != m_tgMv || ma != m_tgMa) m_reneg = true;
        m_tgMv = mv;
        m_tgMa = ma;
    }

    /**
     * @brief 设成「只收 caps 不发 Request」模式 (默认 true = 收到就自动申请)。
     *
     * 上层若要先把 Source Capabilities 列表给用户看、再决定何时申请,
     * 就在 start() 之前 setAutoRequest(false): 收到 caps 后停在 State::CAPS,
     * 由上层调 requestNow() 触发 Request。
     */
    void setAutoRequest(bool on) noexcept { m_autoReq = on; }

    /**
     * @brief 手动发起一次 Request (State::CAPS / READY 时有效)。
     * @return true = 已进入 SEND_REQ (还没收到 caps 时返回 false)。
     */
    bool requestNow() noexcept;

    /**
     * @brief 发一个 Hard Reset (CONTROL3.SEND_HARD_RESET, 不经 FIFO)。
     * @return true = 寄存器写成功。
     *
     * 用途: 单独验证"我们的 BMC 发射能不能被对端看见" —— 对端收到 Hard Reset
     * 一般会回一个 Hard Reset, 于是 hardResetCount() 会 +1。
     */
    bool sendHardReset() noexcept;

    /*---------- 访问器 ----------*/
    [[nodiscard]] State state() const noexcept { return m_state; }
    [[nodiscard]] Fault fault() const noexcept { return m_fault; }
    [[nodiscard]] bool   isReady() const noexcept { return m_state == State::READY; }
    [[nodiscard]] bool   isAttached() const noexcept { return m_cc != CcNone; }

    [[nodiscard]] int  contractMv() const noexcept { return m_cMv; }  ///< 合同电压 (READY 才有效)
    [[nodiscard]] int  contractMa() const noexcept { return m_cMa; }  ///< 合同电流
    [[nodiscard]] bool contractIsPps() const noexcept { return m_cPps; }
    [[nodiscard]] int  targetMv() const noexcept { return m_tgMv; }
    [[nodiscard]] int  targetMa() const noexcept { return m_tgMa; }

    [[nodiscard]] uint8_t pdoCount() const noexcept { return m_numPdo; }
    [[nodiscard]] uint32_t pdoAt(uint8_t i) const noexcept
    {
        return (i < m_numPdo) ? m_pdos[i] : 0u;
    }

    /** 状态名 ("IDLE"/"DETECT"/..., 调试与 UI 用, 永不为 nullptr) */
    [[nodiscard]] const char *stateName() const noexcept;
    /** 故障名 ("OK"/"NO_PDO"/...) */
    [[nodiscard]] const char *faultName() const noexcept;

    /*---------- 握手排障计数 / 读数 (调试页用) ---------- */
    [[nodiscard]] uint8_t  ccLevel1() const noexcept      ///< CC1 的 BC_LVL (0..3)
    {
        return static_cast<uint8_t>(m_bc1 < 0 ? 0 : m_bc1);
    }
    [[nodiscard]] uint8_t  ccLevel2() const noexcept      ///< CC2 的 BC_LVL (0..3)
    {
        return static_cast<uint8_t>(m_bc2 < 0 ? 0 : m_bc2);
    }
    [[nodiscard]] uint8_t  ccPolarity() const noexcept    ///< 0=未附着 1=CC1 2=CC2
    {
        return static_cast<uint8_t>(m_cc);
    }
    [[nodiscard]] uint32_t rxCount() const noexcept { return m_rxCount; }      ///< 收到的报文数
    [[nodiscard]] uint32_t txCount() const noexcept { return m_txCount; }      ///< 发出的报文数
    [[nodiscard]] uint32_t txSentCount() const noexcept { return m_txSentCnt; }///< TX 拿到 GoodCRC 次数
    [[nodiscard]] uint32_t retryFailCount() const noexcept { return m_rfCnt; } ///< TX 重发耗尽次数
    [[nodiscard]] uint32_t txStuckCount() const noexcept { return m_txStuckCnt; } ///< 压了包但 FIFO 不空 (卡住) 次数
    [[nodiscard]] uint32_t hardResetCount() const noexcept { return m_hrCnt; } ///< 收到 Hard Reset 次数
    [[nodiscard]] uint32_t txHardResetSentCount() const noexcept { return m_txHrSentCnt; } ///< 我们发出 Hard Reset 次数
    [[nodiscard]] uint16_t lastRxHdr() const noexcept { return m_lastRxHdr; }  ///< 最近一包报头 (0=还没有)

    /** 重置 FAULT → DETECT (换 target 后可直接重试)。IDLE 下无效果。 */
    void clearFault() noexcept
    {
        if (m_state == State::FAULT) {
            m_fault = Fault::NONE;
            m_state = State::DETECT;
            resetScan();
        }
    }

private:
    /*---------- CC 极性 ----------*/
    enum Cc : uint8_t { CcNone = 0, Cc1 = 1, Cc2 = 2 };

    /*---------- 内部动作 ----------*/
    bool   writeConfig() noexcept;                 ///< begin() 的寄存器序列
    void   enterState(State s, uint32_t nowMs) noexcept;
    void   setPolarity(Cc cc) noexcept;            ///< SWITCHES0/1 极性 (PDWN+MEAS+TXCC+AUTO_CRC)
    void   resetScan() noexcept                    ///< DETECT 扫描状态复位
    {
        m_p1 = -1;
        m_p2 = -1;
        m_measPhase = 0;
        m_scanAt = 0;
        m_capTries = 0;
        m_polFlipTried = false;
        m_srcCapSent = false;
    }
    void   scanDetect(uint32_t nowMs) noexcept;    ///< DETECT: 交替读两根 CC
    void   pumpRx(uint32_t nowMs) noexcept;        ///< 排空 RX FIFO (每 tick)
    void   onSourceCaps(uint16_t hdr, const uint8_t *data,
                        uint8_t nObj, uint32_t nowMs) noexcept;
    void   onControl(uint8_t type, uint32_t nowMs) noexcept;
    void   checkDetach(uint32_t nowMs) noexcept;   ///< BC_LVL 连续归零 → 拔出
    void   checkInts(uint32_t nowMs) noexcept;     ///< 读 INTERRUPTA/STATUS0A (HardRst 等)
    bool   sendRaw(const uint8_t *payload, std::size_t len) noexcept; ///< 压 TX FIFO 触发发送
    bool   sendRequest(uint32_t nowMs) noexcept;   ///< 选 PDO + 构造 RDO + 压 TX FIFO
    bool   sendControl(uint8_t type) noexcept;     ///< 发空 payload 控制报文
    bool   buildRdo(int mv, int ma, uint32_t *rdo,
                    bool *isPps, int *cMv, int *cMa) const noexcept;
    void   fail(Fault f) noexcept                  ///< 进 FAULT
    {
        m_fault = f;
        m_state = State::FAULT;
    }

    /*---------- 成员 ----------*/
    FUSB302 *m_fusb = nullptr;

    State m_state = State::IDLE;
    Fault m_fault = Fault::NONE;

    /* DETECT 扫描 */
    int8_t  m_p1 = -1, m_p2 = -1;      ///< 上一轮 bc1/bc2 (两轮稳定判 attach)
    uint8_t m_measPhase = 0;           ///< 0=本轮测 CC1, 1=本轮测 CC2
    uint32_t m_scanAt = 0;             ///< 下一次扫描/读取时刻

    /* attach / 极性 */
    Cc      m_cc = CcNone;             ///< 当前附着的 CC (None=未附着)
    int8_t  m_bc1 = 0, m_bc2 = 0;      ///< 最近一轮 BC_LVL (0..3)
    uint8_t m_rev = 2;                 ///< 对端 specRev: 自动 GoodCRC 的报头要用它
                                       ///< (收到 caps 后按报头更新)。默认 2 = PD3.0 ——
                                       ///< GoodCRC 必须与所确认报文 SpecRev 一致, 否则
                                       ///< 源端认为没被确认 → 反复重发 → Hard Reset 循环。
    uint8_t m_detCnt = 0;              ///< 拔出连续计数 (500ms 一轮)

    /* Source Caps */
    uint32_t m_pdos[7] = {};
    uint8_t  m_numPdo = 0;

    /* 报文 */
    uint8_t m_msgId = 0;               ///< 发送计数 0..7 (取模 8)
    bool    m_autoReq = true;          ///< 收到 caps 就自动发 Request (false = 停在 CAPS)

    /* 握手排障计数 */
    uint32_t m_rxCount = 0;            ///< 收到的报文数 (SOP 本体)
    uint32_t m_txCount = 0;            ///< 成功压入 FIFO 的报文数
    uint32_t m_txSentCnt = 0;          ///< INTERRUPTA.I_TXSENT 次数 (TX 拿到 GoodCRC)
    uint32_t m_rfCnt = 0;              ///< INTERRUPTA.I_RETRYFAIL 次数 (TX 无人应答)
    uint32_t m_txStuckCnt = 0;         ///< 压包后 TX FIFO 仍不空 (解卡) 次数
    uint32_t m_hrCnt = 0;              ///< 收到 Hard Reset 次数
    uint32_t m_txHrSentCnt = 0;        ///< 我们发出 Hard Reset 次数 (TX 自检)
    uint32_t m_txMs = 0;               ///< 最近一次压包的时基 (卡住检测用, 0=无)
    uint16_t m_lastRxHdr = 0;          ///< 最近一包报头

    /* 协商 / 合同 */
    int  m_tgMv = 5000, m_tgMa = 1000; ///< 目标 (setTarget)
    int  m_cMv = 0,     m_cMa = 0;     ///< 合同 (READY 后有效)
    bool m_cPps = false;
    int  m_pMv = 0,  m_pMa = 0;        ///< 最近发出的 RDO 合同值 (PS_RDY 时落账)
    bool m_pPps = false;
    bool m_reneg = false;              ///< setTarget 改过目标 → READY 后重协商
    uint32_t m_t0 = 0;                 ///< 当前状态进入时刻 (超时 = nowMs-m_t0, 回绕安全)
    uint32_t m_lastReqMs = 0;          ///< READY: 最近一次 PPS 重请求
    uint8_t  m_reqTries = 0;           ///< Request 重发计数
    uint8_t  m_capTries = 0;           ///< 等 caps 阶段计数 (换极性 / Get_Source_Cap 次数)
    bool     m_polFlipTried = false;   ///< 已经试过另一根 CC 极性
    bool     m_srcCapSent = false;     ///< 本轮已主动发过 Get_Source_Cap
};

} // namespace LoveFinderLib

#endif /* PDSINK_HPP */
