/**
 * @file    PDSink.cpp
 * @brief   FUSB302 SINK 模式 PD 驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * 轮询式: 每次 tick 读中断闩锁 / 拔出采样 / 排空 RX FIFO, 不接 INT 脚。
 * 报文收发走 FUSB302 的 FIFO 寄存器 (0x43): 写 = 压 TX 包并由 TXON(0xA1)
 * 触发发送; 读 = 弹 RX 包 (token + 报头 + 载荷 + CRC)。
 * 位域速查与状态机说明见 PDSink.hpp 文件头。
 */
#include "PDSink.hpp"

namespace LoveFinderLib {

namespace {

/*============================================================================
 * 中断 MASK (0 = 不屏蔽 → 会拉低 INT 脚并置闩锁)
 *   Mask1: 先全屏蔽 (VBUSOK / BC_LVL 靠轮询, 少一批假中断)
 *   Maska: 解 HARDRST(0) / TXSENT(2) / HARDSENT(3) / RETRYFAIL(4)
 *   Maskb: 解 GCRCSENT(0) —— 收到任何报文并自动 GoodCRC 都会触发
 *   CONTROL0.INT_MASK = 0 → INT 脚使能 (板: PA5, 外部 10K 上拉, 下降沿有效)
 *============================================================================*/
constexpr uint8_t MK_A_UNMASK = 0xE2;  ///< 0xFF & ~(0x01|0x04|0x08|0x10)
constexpr uint8_t MK_B_UNMASK = 0xFE;  ///< 0xFF & ~0x01

/*============================================================================
 * 寄存器位 (FUSB302 / FUSB302B 数据手册)
 *============================================================================*/
/* SWITCHES0 */
constexpr uint8_t SW0_PDWN1    = 0x01;  ///< CC1 Rd 下拉 (让源端看到 sink)
constexpr uint8_t SW0_PDWN2    = 0x02;  ///< CC2 Rd 下拉
constexpr uint8_t SW0_MEAS_CC1 = 0x04;  ///< 比较器选 CC1 (读 STATUS0.BC_LVL)
constexpr uint8_t SW0_MEAS_CC2 = 0x08;  ///< 比较器选 CC2
/* SWITCHES1 */
constexpr uint8_t SW1_TXCC1    = 0x01;  ///< BMC 收发走 CC1
constexpr uint8_t SW1_TXCC2    = 0x02;  ///< BMC 收发走 CC2
constexpr uint8_t SW1_AUTO_CRC = 0x04;  ///< 自动 GoodCRC
/* CONTROL0/1/3 */
constexpr uint8_t C0_INT_MASK  = 0x20;  ///< CONTROL0.INT_MASK: 1 = 关 INT 脚 (我们用 0)
constexpr uint8_t C0_TX_FLUSH  = 0x40;  ///< 冲 TX FIFO (自清)
constexpr uint8_t C0_HOST_CUR_3A = 0x0C;///< HOST_CUR=3A (bits3:2) + INT_MASK=0 (同 usb-pd)
constexpr uint8_t C1_RX_FLUSH  = 0x04;  ///< 冲 RX FIFO (自清)
constexpr uint8_t C3_AUTORETRY = 0x07;  ///< 自动重发 ×3
constexpr uint8_t C3_SEND_HARDRST = 0x40; ///< 发送 Hard Reset (自清, 不经 FIFO)
/* 状态位 */
constexpr uint8_t ST0_BC_LVL   = 0x03;  ///< STATUS0: CC 上 Rp 分压等级 (0 = 无 Rp)
constexpr uint8_t ST1_RX_EMPTY = 0x20;  ///< STATUS1: RX FIFO 空
constexpr uint8_t ST1_TX_EMPTY = 0x08;  ///< STATUS1: TX FIFO 空 (0 = 包还压着没发)
constexpr uint8_t IA_HARDRST   = 0x01;  ///< INTERRUPTA/STATUS0A: 收到硬复位
constexpr uint8_t IA_TXSENT    = 0x04;  ///< INTERRUPTA: 本包发送成功 (收到对端 GoodCRC)
constexpr uint8_t IA_RETRYFAIL = 0x10;  ///< INTERRUPTA: 自动重发耗尽 (对端没应答)
/* TX FIFO token (usb-pd / pdsink 同款) */
constexpr uint8_t TX_SOP1      = 0x12;
constexpr uint8_t TX_SOP2      = 0x13;
constexpr uint8_t TX_PACKSYM   = 0x80;  ///< 低 5 位 = 报头+载荷字节数
constexpr uint8_t TX_JAM_CRC   = 0xFF;
constexpr uint8_t TX_EOP       = 0x14;
constexpr uint8_t TX_TXOFF     = 0xFE;
constexpr uint8_t TX_TXON      = 0xA1;  ///< 末字节, 触发发送
/* RX token: 高 3 位 0b111 = SOP 本体 */
constexpr uint8_t RX_TOK_MASK  = 0xE0;
constexpr uint8_t RX_TOK_SOP   = 0xE0;
/* PD 报文类型 (低 3 位) */
constexpr uint8_t PD_CTRL_ACCEPT     = 0x3;
constexpr uint8_t PD_CTRL_REJECT     = 0x4;
constexpr uint8_t PD_CTRL_PS_RDY     = 0x6;
constexpr uint8_t PD_CTRL_GET_SRC_CAP= 0x7;
constexpr uint8_t PD_CTRL_WAIT       = 0xC;
constexpr uint8_t PD_CTRL_SOFT_RESET = 0xD;
constexpr uint8_t PD_DATA_SRC_CAP    = 0x1;
constexpr uint8_t PD_DATA_REQ        = 0x2;

} // namespace

/*============================================================================
 * 上电配置
 *============================================================================*/
bool PDSink::begin() noexcept
{
    if (m_fusb == nullptr) {
        fail(Fault::INIT_FAIL);
        return false;
    }
    if (!writeConfig()) {
        fail(Fault::INIT_FAIL);
        return false;
    }
    m_fault = Fault::NONE;
    return true;
}

bool PDSink::writeConfig() noexcept
{
    FUSB302 &f = *m_fusb;
    /* 顺序: 软复位 → 上电 → 开自动重发 → 配中断掩码 (解出握手相关事件) →
       INT 脚使能 (CONTROL0.INT_MASK = 0) → 冲 FIFO → 清 PD 内核 → CC 保持全断
       (start() 才挂 Rd)。
       参考 pdsink (Zephyr FUSB302 sink): Mask1 全屏蔽 + Maska/Maskb 解 RX/TX 事件,
       最后才清 CONTROL0.INT_MASK 打开 INT 脚。 */
    return f.writeReg(e_FUSB302_Reg::RESET,    0x01)                 /* SW_RES */
        && f.writeReg(e_FUSB302_Reg::POWER,    0x0F)                 /* 内部全上电 */
        && f.writeReg(e_FUSB302_Reg::CONTROL2, 0x00)                 /* 关 CC toggle */
        && f.writeReg(e_FUSB302_Reg::CONTROL3, C3_AUTORETRY)          /* GoodCRC 重发×3 */
        && f.writeReg(e_FUSB302_Reg::MASK1,    0xFF)                 /* VBUSOK/BC_LVL 走轮询 */
        && f.writeReg(e_FUSB302_Reg::MASKA,    MK_A_UNMASK)           /* HARDRST/TXSENT/HARDSENT/RETRYFAIL */
        && f.writeReg(e_FUSB302_Reg::MASKB,    MK_B_UNMASK)           /* GCRCSENT (收到报文) */
        && f.writeReg(e_FUSB302_Reg::CONTROL1, C1_RX_FLUSH)          /* 冲 RX FIFO */
        && f.writeReg(e_FUSB302_Reg::CONTROL0, C0_TX_FLUSH)          /* 冲 TX FIFO (INT 仍关) */
        && f.writeReg(e_FUSB302_Reg::CONTROL0, C0_HOST_CUR_3A)       /* INT 开 + HOST_CUR=3A (同 usb-pd) */
        && f.writeReg(e_FUSB302_Reg::RESET,    0x02)                 /* PD_RESET 清 PD 内核 */
        && f.writeReg(e_FUSB302_Reg::SWITCHES0, 0x00);               /* CC 断开 */
}

void PDSink::start() noexcept
{
    if (m_fusb == nullptr) {
        fail(Fault::INIT_FAIL);
        return;
    }
    m_fault  = Fault::NONE;
    m_cc     = CcNone;
    m_numPdo = 0;
    m_cMv = m_cMa = 0;
    m_cPps = false;
    m_pMv = m_pMa = 0;
    m_pPps = false;
    m_reneg = false;
    m_msgId = 0;
    m_rev   = 2;
    m_detCnt = 0;
    resetScan();

    /* CC1+CC2 挂 Rd 下拉 (SW3526S 据此进入 source), 比较器先选 CC1 */
    (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES0,
                           SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC1);
    const uint32_t now = lf_get_tick();
    enterState(State::DETECT, now);
    m_scanAt = now + 5U;   /* MEAS 切换后给比较器 5ms 稳定 */
}

void PDSink::stop() noexcept
{
    if (m_fusb != nullptr) {
        (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES0, 0x00);  /* 释放 Rd */
    }
    m_cc     = CcNone;
    m_numPdo = 0;
    m_cMv = m_cMa = 0;
    m_cPps = false;
    m_reneg = false;
    m_state = State::IDLE;
    m_fault = Fault::NONE;
    m_rev   = 2;                       /* 默认按 PD3.0 发自动 GoodCRC (见 m_rev 注释) */
}

/*============================================================================
 * 状态机
 *============================================================================*/
void PDSink::enterState(State s, uint32_t nowMs) noexcept
{
    m_state = s;
    m_t0    = nowMs;                 /* 超时用 nowMs - m_t0 无符号减, 回绕安全 */
    if (s == State::READY) m_lastReqMs = nowMs;
}

void PDSink::tick(uint32_t nowMs) noexcept
{
    if (m_fusb == nullptr) return;

    checkInts(nowMs);                /* 读掉中断闩锁 (DETECT 也要清, 否则 INT 脚卡住) */

    if (m_state == State::DETECT) {
        scanDetect(nowMs);
        return;
    }
    if (m_state == State::IDLE || m_state == State::FAULT) return;

    /* ---- PD 活动状态 (WAIT_CAP..READY) 的逐拍维护 ---- */
    if (m_state != State::WAIT_CAP && m_state != State::CAPS &&
        m_state != State::SEND_REQ && m_state != State::WAIT_ACCEPT &&
        m_state != State::WAIT_PS_RDY && m_state != State::READY) {
        return;
    }
    checkDetach(nowMs);              /* 拔出 → 回 DETECT */
    if (m_state == State::DETECT) return;

    /* TX 卡住检测: 压过包但 30ms 后 TX FIFO 还不空 → 包没发出去 (BMC/极性异常),
       冲掉 FIFO 解卡并计数 (STATUS1.TX_EMPTY = bit3)。 */
    if (m_txMs != 0u && (nowMs - m_txMs) >= 30u) {
        const auto s1 = m_fusb->readReg(e_FUSB302_Reg::STATUS1);
        if (!s1.has_value() || ((*s1 & ST1_TX_EMPTY) == 0u)) {
            (void)m_fusb->writeReg(e_FUSB302_Reg::CONTROL0, C0_TX_FLUSH);
            ++m_txStuckCnt;
        }
        m_txMs = 0u;
    }

    pumpRx(nowMs);                   /* 收包可改变状态 (caps/Accept/PS_RDY...) */

    switch (m_state) {
    case State::WAIT_CAP:
        if (!m_srcCapSent) {
            /* 一进 WAIT_CAP 就主动催一次 caps (不等超时):
               ① 源端可能已经在我们"去抖/扫 CC"期间把 caps 发过一次而漏掉;
               ② 立刻能从 TXSENT/RETRYFAIL 看出 CC 极性/对端 PD 是否活着。 */
            m_srcCapSent = true;
            if (sendControl(PD_CTRL_GET_SRC_CAP)) enterState(State::WAIT_CAP, nowMs);
            break;
        }
        if (nowMs - m_t0 >= PD_SINK_T_WAIT_CAP_MS) {
            if (!m_polFlipTried && m_bc1 > 0 && m_bc2 > 0) {
                /* 两根 CC 都有 Rp (直连双线拓扑) → 换极性再试 (对端 PD 可能走另一根) */
                m_polFlipTried = true;
                setPolarity(m_cc == Cc1 ? Cc2 : Cc1);
                enterState(State::WAIT_CAP, nowMs);
            } else if (m_capTries < 3) {
                ++m_capTries;                       /* 再补两枪 Get_Source_Cap */
                if (sendControl(PD_CTRL_GET_SRC_CAP)) enterState(State::WAIT_CAP, nowMs);
            } else {
                fail(Fault::NOSRCS);
            }
        }
        break;

    case State::SEND_REQ:
        sendRequest(nowMs);          /* 成功 → WAIT_ACCEPT; 失败 → FAULT */
        break;

    case State::WAIT_ACCEPT:
        if (nowMs - m_t0 >= PD_SINK_T_SENDER_RESP_MS) {
            if (m_reqTries < 3) {
                ++m_reqTries;
                enterState(State::SEND_REQ, nowMs);
            } else {
                fail(Fault::NO_RESPONSE);
            }
        }
        break;

    case State::WAIT_PS_RDY:
        if (nowMs - m_t0 >= PD_SINK_T_PS_RDY_MS) {
            fail(Fault::NO_RESPONSE);
        }
        break;

    case State::READY:
        if (m_reneg) {                /* setTarget 改过目标 → 重协商 */
            m_reneg = false;
            m_reqTries = 0;
            enterState(State::SEND_REQ, nowMs);
        } else if (m_cPps && nowMs - m_lastReqMs >= PD_SINK_T_PPS_REQ_MS) {
            /* PPS 合同 5s 不续就掉 (tPPSRequest) → 周期重发同一 RDO */
            enterState(State::SEND_REQ, nowMs);
        }
        break;

    case State::CAPS:                 /* 已收到 caps, 等上层 requestNow() */
        break;

    default:
        break;
    }
}

bool PDSink::requestNow() noexcept
{
    if (m_fusb == nullptr) return false;
    if (m_state != State::CAPS && m_state != State::READY) return false;
    if (m_numPdo == 0) return false;          /* 还没收到 caps */
    m_reqTries = 0;
    enterState(State::SEND_REQ, lf_get_tick());
    return true;
}

/*============================================================================
 * DETECT: 交替扫两根 CC 的 BC_LVL
 *============================================================================*/
void PDSink::scanDetect(uint32_t nowMs) noexcept
{
    if ((int32_t)(nowMs - m_scanAt) < 0) return;   /* 未到采样时刻 */

    if (m_measPhase == 0) {
        /* 上一步选了 MEAS_CC1 → 读 CC1, 然后切 CC2 */
        const auto st = m_fusb->readReg(e_FUSB302_Reg::STATUS0);
        if (st.has_value()) m_bc1 = (int8_t)(*st & ST0_BC_LVL);
        (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES0,
                               SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC2);
        m_measPhase = 1u;
        m_scanAt = nowMs + 5U;
        return;
    }

    /* 本轮读 CC2 → 收一轮 */
    const auto st = m_fusb->readReg(e_FUSB302_Reg::STATUS0);
    if (st.has_value()) m_bc2 = (int8_t)(*st & ST0_BC_LVL);
    (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES0,
                           SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC1);
    m_measPhase = 0u;
    m_scanAt = nowMs + 5U;

    /* attach 判据: 连续两轮读数一致 (稳定) 且至少一根 CC 有 Rp。
       单线拓扑 (正常线缆) 只有一根 >0; 直连双线两根都 >0, 平手先取 CC1,
       若 10s 等不到 caps 由 WAIT_CAP 分支自动换 CC2 重试。 */
    const int mx = (m_bc1 > m_bc2) ? (int)m_bc1 : (int)m_bc2;
    if (mx > 0 && m_bc1 == m_p1 && m_bc2 == m_p2) {
        const Cc pol = (m_bc2 > m_bc1) ? Cc2 : Cc1;
        resetScan();
        m_detCnt = 0;
        setPolarity(pol);            /* MEAS/TXCC 定极性, m_cc 落位 */
        enterState(State::WAIT_CAP, nowMs);
        return;
    }
    m_p1 = m_bc1;
    m_p2 = m_bc2;
}

void PDSink::setPolarity(Cc cc) noexcept
{
    const uint8_t sw0 = (uint8_t)(SW0_PDWN1 | SW0_PDWN2 |
                                  (cc == Cc2 ? SW0_MEAS_CC2 : SW0_MEAS_CC1));
    const uint8_t sw1 = (uint8_t)((cc == Cc2 ? SW1_TXCC2 : SW1_TXCC1) |
                                  SW1_AUTO_CRC |
                                  (uint8_t)((m_rev & 0x03u) << 5));
    (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES0, sw0);
    (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES1, sw1);
    m_cc = cc;
}

/*============================================================================
 * 拔出检测 / 中断闩锁
 *============================================================================*/
void PDSink::checkDetach(uint32_t nowMs) noexcept
{
    if (nowMs - m_scanAt < 500U) return;   /* PD 态复用 m_scanAt 做采样节拍 */
    m_scanAt = nowMs;

    const auto st = m_fusb->readReg(e_FUSB302_Reg::STATUS0);
    if (!st.has_value()) return;           /* 读失败: 下拍再试 (不误判拔出) */
    if ((*st & ST0_BC_LVL) == 0) {
        if (++m_detCnt >= 2) {           /* 连续两轮 (1s) 无 Rp = 真拔出 */
            m_detCnt = 0;
            m_cc     = CcNone;
            m_numPdo = 0;
            m_cMv = m_cMa = 0;
            m_cPps = false;
            m_reneg = false;
            resetScan();
            (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES0,
                                   SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC1);
            enterState(State::DETECT, nowMs);
            m_scanAt = nowMs + 5U;
        }
    } else {
        m_detCnt = 0;
    }
}

void PDSink::checkInts(uint32_t nowMs) noexcept
{
    /* 把使能了的中断闩锁全部读掉 (读即清), 否则 INT 脚会一直被钉在低电平,
       后续事件再也拉不出下降沿: INTERRUPT (VBUSOK 等) + INTERRUPTA + INTERRUPTB。
       INTERRUPTA / STATUS0A 的 HARDRST 位用来判"收到硬复位";
       I_TXSENT / I_RETRYFAIL 记进排障计数 (TX 有没有被对端 GoodCRC 应答)。 */
    const auto in = m_fusb->readReg(e_FUSB302_Reg::INTERRUPT);
    const auto ia = m_fusb->readReg(e_FUSB302_Reg::INTERRUPTA);
    const auto ib = m_fusb->readReg(e_FUSB302_Reg::INTERRUPTB);
    const auto sa = m_fusb->readReg(e_FUSB302_Reg::STATUS0A);
    (void)in;
    (void)ib;
    if (ia.has_value()) {
        if ((*ia & IA_TXSENT) != 0)    ++m_txSentCnt;
        if ((*ia & IA_RETRYFAIL) != 0) ++m_rfCnt;
    }
    const bool hard = (ia.has_value() && ((*ia & IA_HARDRST) != 0)) ||
                      (sa.has_value() && ((*sa & IA_HARDRST) != 0));
    if (hard) ++m_hrCnt;
    if (!hard) return;

    /* 硬复位: 合同作废 → 源端会重发 Source Caps, 我们回到等 caps。
       ⚠ 不冲 RX FIFO: 源端在硬复位后立刻重发的 caps 可能已经在 FIFO 里,
       冲掉就等于把新 caps 扔了 (实测会陷入 HR 循环、列表一直空)。 */
    m_cMv = m_cMa = 0;
    m_cPps = false;
    m_numPdo = 0;
    m_msgId = 0;
    m_capTries = 0;
    m_polFlipTried = false;
    m_srcCapSent = false;
    enterState(State::WAIT_CAP, nowMs);
}

/*============================================================================
 * RX: 排空 FIFO 并分发
 *============================================================================*/
void PDSink::pumpRx(uint32_t nowMs) noexcept
{
    for (uint8_t n = 0; n < 8; ++n) {
        const auto st = m_fusb->readReg(e_FUSB302_Reg::STATUS1);
        if (!st.has_value()) return;                      /* 读失败: 下拍再试 */
        if ((*st & ST1_RX_EMPTY) != 0) return;            /* 空了 */

        uint8_t hb[3] = {0, 0, 0};                        /* token + 报头×2 */
        if (!m_fusb->readBytes(e_FUSB302_Reg::FIFO, hb, 3)) return;

        const uint16_t hdr  = (uint16_t)((uint32_t)hb[1] | ((uint32_t)hb[2] << 8));
        const bool     ext  = (hdr & 0x8000u) != 0u;
        const uint8_t  type = (uint8_t)(hdr & 0x07u);
        const uint8_t  nObj = (uint8_t)((hdr >> 12) & 0x07u);

        if (ext) {
            /* 扩展/分块报文一期不解析, 整条 FIFO 冲掉避免残留错位 */
            (void)m_fusb->writeReg(e_FUSB302_Reg::CONTROL1, C1_RX_FLUSH);
            return;
        }

        uint8_t body[32] = {0};                           /* 7×4 载荷 + 4 CRC */
        const uint32_t rest = (uint32_t)nObj * 4u + 4u;
        if (rest > sizeof(body)) return;                  /* 不可达 (nObj≤7) */
        if (!m_fusb->readBytes(e_FUSB302_Reg::FIFO, body, rest)) return;

        if ((hb[0] & RX_TOK_MASK) != RX_TOK_SOP) continue; /* 非 SOP 本体, 丢 */
        ++m_rxCount;                                       /* 排障计数: 收到的报文数 */
        m_lastRxHdr = hdr;
        if (nObj == 0) {                                   /* 控制报文 */
            if (type == 0x1) continue;                     /* GoodCRC (自动回的) */
            onControl(type, nowMs);
        } else if (type == PD_DATA_SRC_CAP) {
            onSourceCaps(hdr, body, nObj, nowMs);
        }
        /* 其余数据报文 (SinkCaps 回执等) 一期忽略 */

        if (m_state == State::IDLE || m_state == State::FAULT ||
            m_state == State::DETECT) {
            return;                                        /* 分发后已离场 */
        }
    }
}

void PDSink::onSourceCaps(uint16_t hdr, const uint8_t *data,
                          uint8_t nObj, uint32_t nowMs) noexcept
{
    if (nObj > 7) nObj = 7;
    for (uint8_t i = 0; i < nObj; ++i) {
        m_pdos[i] = (uint32_t)data[(std::size_t)i * 4u]
                  | ((uint32_t)data[(std::size_t)i * 4u + 1u] << 8)
                  | ((uint32_t)data[(std::size_t)i * 4u + 2u] << 16)
                  | ((uint32_t)data[(std::size_t)i * 4u + 3u] << 24);
    }
    m_numPdo = nObj;
    m_rev    = (uint8_t)((hdr >> 4) & 0x03u);
    /* SWITCHES1.SPECREV 只影响自动 GoodCRC 的报头 → 跟对端 */
    (void)m_fusb->writeReg(e_FUSB302_Reg::SWITCHES1,
                           (uint8_t)((m_cc == Cc2 ? SW1_TXCC2 : SW1_TXCC1) |
                                     SW1_AUTO_CRC | (uint8_t)(m_rev << 5)));

    switch (m_state) {
    case State::WAIT_CAP:              /* 首次拿到 caps → 发 Request (或停在 CAPS 等上层命令) */
    case State::WAIT_ACCEPT:           /* 收 caps = 合同要重建 → 重发 Request */
    case State::WAIT_PS_RDY:
        m_reqTries = 0;
        if (m_autoReq) {
            enterState(State::SEND_REQ, nowMs);
        } else {
            /* 手动模式 (app 要先给用户看 PDO 列表): 停在 CAPS, 等 requestNow() */
            enterState(State::CAPS, nowMs);
        }
        break;
    default:                           /* READY: 只刷新列表 (重协商由 m_reneg/
                                         SEND_REQ/周期重请求分支触发) */
        break;
    }
}

void PDSink::onControl(uint8_t type, uint32_t nowMs) noexcept
{
    switch (type) {
    case PD_CTRL_ACCEPT:
        if (m_state == State::WAIT_ACCEPT || m_state == State::READY) {
            enterState(State::WAIT_PS_RDY, nowMs);
        }
        break;

    case PD_CTRL_PS_RDY:
        if (m_state == State::WAIT_PS_RDY) {
            m_cMv = m_pMv;            /* RDO 落账 = 合同生效 */
            m_cMa = m_pMa;
            m_cPps = m_pPps;
            m_reqTries = 0;
            enterState(State::READY, nowMs);
        }
        break;

    case PD_CTRL_REJECT:
        if (m_state == State::WAIT_ACCEPT) {
            fail(Fault::NO_RESPONSE); /* 源端明确拒绝当前 RDO */
        }
        /* READY 里收到 (周期 PPS 被拒): 老合同仍有效, 忽略 */
        break;

    case PD_CTRL_WAIT:
        if (m_state == State::WAIT_ACCEPT) {
            enterState(State::WAIT_ACCEPT, nowMs);  /* 重置 t0 拿宽限期 */
        }
        break;

    case PD_CTRL_SOFT_RESET:
        (void)sendControl(PD_CTRL_SOFT_RESET);      /* 协议层复位握手回包 */
        m_msgId = 0;
        m_numPdo = 0;
        enterState(State::WAIT_CAP, nowMs);         /* 源端随后重发 caps */
        break;

    default:                                        /* Get_Sink_Cap 等: 一期忽略 */
        break;
    }
}

/*============================================================================
 * TX
 *============================================================================*/
bool PDSink::sendRaw(const uint8_t *payload, std::size_t len) noexcept
{
    /* payload = 报头 2B [+ 数据对象 4B×n], len = 2+4n ∈ [2,30] */
    if (payload == nullptr || len < 2u || len > 30u) return false;

    /* 按参考实现 (FUSB302-master 的 transmit) 的发送序列:
       ① 先冲 TX FIFO (参考栈都这么做, 保证不会把新包追加到旧包后面);
       ② "寄存器地址 + SOP1×3 + SOP2 + PACKSYM|len + 报头 + 载荷 + JAM_CRC +
          EOP + TXOFF + TXON" 一次不分片 burst 写完 (FUSB302-master 就是这么干的,
          整包可达 40B, 所以不能用有 16B 限制的 i2cMemWrite)。 */
    if (len + 9u > 40u) return false;

    (void)m_fusb->writeReg(e_FUSB302_Reg::CONTROL0, C0_TX_FLUSH);   /* 冲 TX FIFO */

    uint8_t buf[40];
    std::size_t k = 0;
    buf[k++] = TX_SOP1;                       /* SOP1×3 + SOP2 */
    buf[k++] = TX_SOP1;
    buf[k++] = TX_SOP1;
    buf[k++] = TX_SOP2;
    buf[k++] = static_cast<uint8_t>(TX_PACKSYM | static_cast<uint8_t>(len));
    for (std::size_t i = 0; i < len; ++i) buf[k++] = payload[i];
    buf[k++] = TX_JAM_CRC;                    /* 伪 CRC (硬件自己算真 CRC) */
    buf[k++] = TX_EOP;
    buf[k++] = TX_TXOFF;
    buf[k++] = TX_TXON;                       /* 末字节触发发送 */

    if (!m_fusb->writeBurst(e_FUSB302_Reg::FIFO, buf, k)) return false;
    ++m_txCount;                              /* 排障计数: 发出的报文数 */
    m_txMs = lf_get_tick();
    return true;
}

bool PDSink::sendHardReset() noexcept
{
    if (m_fusb == nullptr) return false;
    /* CONTROL3.SEND_HARD_RESET (bit6, 自清): 不走 FIFO 的 TX —— 用它单独验证
       "我们的 BMC 发射能不能被对端看见" (对端收到后一般会回一个 Hard Reset)。 */
    const auto c3 = m_fusb->readReg(e_FUSB302_Reg::CONTROL3);
    const uint8_t v = static_cast<uint8_t>(
        (c3.has_value() ? *c3 : C3_AUTORETRY) | C3_SEND_HARDRST);
    if (!m_fusb->writeReg(e_FUSB302_Reg::CONTROL3, v)) return false;
    ++m_txHrSentCnt;
    return true;
}

bool PDSink::sendControl(uint8_t type) noexcept
{
    const uint16_t hdr = (uint16_t)((uint32_t)(type & 0x07u)
                                  | ((uint32_t)m_rev << 4)
                                  | ((uint32_t)(m_msgId & 0x07u) << 9));
    uint8_t pl[2] = { (uint8_t)(hdr & 0xFFu), (uint8_t)(hdr >> 8) };
    if (!sendRaw(pl, sizeof(pl))) return false;
    m_msgId = (uint8_t)((m_msgId + 1u) & 0x07u);
    return true;
}

bool PDSink::sendRequest(uint32_t nowMs) noexcept
{
    uint32_t rdo = 0;
    bool     pps = false;
    int      cm  = 0, ca = 0;
    if (!buildRdo(m_tgMv, m_tgMa, &rdo, &pps, &cm, &ca)) {
        fail(Fault::NO_PDO);                 /* 所有 PDO 都盖不住目标 */
        return false;
    }

    const uint16_t hdr = (uint16_t)(PD_DATA_REQ                  /* Request */
                       | ((uint32_t)m_rev << 4)
                       | ((uint32_t)(m_msgId & 0x07u) << 9)
                       | (1u << 12));                             /* numObj=1 */
    uint8_t pl[6];
    pl[0] = (uint8_t)(hdr & 0xFFu);
    pl[1] = (uint8_t)(hdr >> 8);
    pl[2] = (uint8_t)(rdo & 0xFFu);
    pl[3] = (uint8_t)((rdo >> 8) & 0xFFu);
    pl[4] = (uint8_t)((rdo >> 16) & 0xFFu);
    pl[5] = (uint8_t)((rdo >> 24) & 0xFFu);

    if (!sendRaw(pl, sizeof(pl))) {
        fail(Fault::I2C_ERROR);
        return false;
    }
    m_msgId = (uint8_t)((m_msgId + 1u) & 0x07u);
    m_pMv = cm;                              /* 待 PS_RDY 落账的合同值 */
    m_pMa = ca;
    m_pPps = pps;
    enterState(State::WAIT_ACCEPT, nowMs);
    return true;
}

/*============================================================================
 * RDO 构造 (选 PDO + 位域拼装)
 *============================================================================*/
bool PDSink::buildRdo(int mv, int ma, uint32_t *rdo,
                      bool *isPps, int *cMv, int *cMa) const noexcept
{
    if (rdo == nullptr || isPps == nullptr || cMv == nullptr || cMa == nullptr) {
        return false;
    }
    if (m_numPdo == 0) return false;

    /* 1) 优先 PPS APDO (type=11, subtype=00): 取覆盖 mv 且范围最小的 */
    int ppsIdx = -1;
    int bestSpan = 0x7FFFFFFF;
    /* 2) 退路: 固定 PDO 精确同压 (type=00, v == mv) */
    int fixIdx = -1;

    for (uint8_t i = 0; i < m_numPdo; ++i) {
        const uint32_t p = m_pdos[i];
        const uint32_t t = p >> 30;
        if (t == 3u && ((p >> 28) & 0x03u) == 0u) {         /* SPR PPS APDO */
            const int vmin = (int)((p >> 8) & 0xFFu) * 100; /* 100mV */
            const int vmax = (int)((p >> 17) & 0xFFu) * 100;
            if (mv >= vmin && mv <= vmax && (vmax - vmin) < bestSpan) {
                bestSpan = vmax - vmin;
                ppsIdx = (int)i;
            }
        } else if (t == 0u) {                               /* 固定 PDO */
            const int v = (int)((p >> 10) & 0x3FFu) * 50;   /* 50mV */
            if (v == mv && fixIdx < 0) fixIdx = (int)i;
        }
    }

    if (ppsIdx >= 0) {
        const uint32_t p = m_pdos[(uint8_t)ppsIdx];
        const int imaxU = (int)(p & 0x7Fu);                 /* APDO 电流 50mA 格 */
        int oper = (ma > 0) ? ma : 100;
        if (oper > imaxU * 50) oper = imaxU * 50;           /* 钳到 APDO 上限 */
        int operU = (oper + 25) / 50;                       /* 50mA 格, 四舍五入 */
        if (operU < 1) operU = 1;
        if (operU > imaxU) operU = imaxU;
        int v20 = (mv + 10) / 20;                           /* 20mV 格, 四舍五入 */
        const int vminU = (int)((p >> 8) & 0xFFu) * 5;      /* vmin/20mV */
        const int vmaxU = (int)((p >> 17) & 0xFFu) * 5;
        if (v20 < vminU) v20 = vminU;
        if (v20 > vmaxU) v20 = vmaxU;

        *rdo = ((uint32_t)(ppsIdx + 1) << 28)               /* obj_pos (1 基) */
             | (1u << 24)                                   /* no_usb_suspend */
             | ((uint32_t)v20 << 9)                         /* 输出电压 20mV */
             | (uint32_t)operU;                             /* 工作电流 50mA */
        *isPps = true;
        *cMv = v20 * 20;
        *cMa = operU * 50;
        return true;
    }

    if (fixIdx >= 0) {
        const uint32_t p = m_pdos[(uint8_t)fixIdx];
        const int imaxU = (int)(p & 0x3FFu);                /* 10mA 格 */
        const int v     = (int)((p >> 10) & 0x3FFu) * 50;   /* 50mV */
        int oper = (ma > 0) ? ma : 100;
        if (oper > imaxU * 10) oper = imaxU * 10;
        int operU = (oper + 5) / 10;                        /* 10mA 格, 四舍五入 */
        if (operU < 1) operU = 1;
        if (operU > imaxU) operU = imaxU;

        *rdo = ((uint32_t)(fixIdx + 1) << 28)
             | (1u << 24)
             | ((uint32_t)imaxU << 10)                      /* max_current */
             | (uint32_t)operU;                             /* operating_current */
        *isPps = false;
        *cMv = v;
        *cMa = operU * 10;
        return true;
    }

    return false;                                           /* 无 PPS 且无同压固定 PDO */
}

/*============================================================================
 * 访问器
 *============================================================================*/
const char *PDSink::stateName() const noexcept
{
    switch (m_state) {
    case State::IDLE:        return "IDLE";
    case State::DETECT:      return "DETECT";
    case State::WAIT_CAP:    return "WAIT_CAP";
    case State::CAPS:        return "CAPS";
    case State::SEND_REQ:    return "SEND_REQ";
    case State::WAIT_ACCEPT: return "WAIT_ACCEPT";
    case State::WAIT_PS_RDY: return "WAIT_PS_RDY";
    case State::READY:       return "READY";
    case State::FAULT:       return "FAULT";
    }
    return "?";
}

const char *PDSink::faultName() const noexcept
{
    switch (m_fault) {
    case Fault::NONE:        return "OK";
    case Fault::INIT_FAIL:   return "INIT_FAIL";
    case Fault::NO_PDO:      return "NO_PDO";
    case Fault::NO_RESPONSE: return "NO_RESPONSE";
    case Fault::NOSRCS:      return "NOSRCS";
    case Fault::I2C_ERROR:   return "I2C_ERROR";
    }
    return "?";
}

} // namespace LoveFinderLib
