/**
 * @file    SW3538.cpp
 * @brief   SW3538 驱动实现 — 寄存器读写时序 / ADC / 状态与测量 (C++17)
 *
 * 时序细节见 SW3538.hpp 文件头。所有 I2C 失败都更新 m_last,
 * TIMEOUT / BUS_ERROR 触发 i2cMasterRecover() (外设级软复位)。
 */
#include "SW3538.hpp"

#include "i2c_master.hpp"

#include <cstdint>

namespace LoveFinderLib {

/*============================================================================
 * 内部: 失败善后 / 解锁序列
 *============================================================================*/
/** 外设状态机卡住 (超时 / 总线错误) 时复位一次 I2C 外设。 */
bool SW3538::maybeRecover() noexcept
{
    if (m_i2c == nullptr) return false;
    if (m_last == I2cStatus::TIMEOUT || m_last == I2cStatus::BUS_ERROR) {
        i2cMasterRecover(m_i2c);
        return true;
    }
    return false;
}

/** Reg0x10 写使能三写序列 (0x20 → 0x40 → 0x80)。 */
bool SW3538::armWriteEnable()
{
    static constexpr uint8_t kSeq[3] = { 0x20u, 0x40u, 0x80u };
    for (const uint8_t v : kSeq) {
        m_last = i2cMemWrite(m_i2c, m_addr7, static_cast<uint8_t>(e_SW3538_Reg::I2C_EN),
                             &v, 1, m_timeoutMs);
        if (m_last != I2cStatus::OK) {
            maybeRecover();
            return false;
        }
    }
    return true;
}

/*============================================================================
 * 寄存器读写
 *============================================================================*/
std::optional<uint8_t> SW3538::readReg(uint8_t reg)
{
    uint8_t v = 0;
    if (!readRegs(reg, &v, 1)) return std::nullopt;
    return v;
}

bool SW3538::readRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (m_i2c == nullptr || buf == nullptr || len == 0u || len > 16u) {
        m_last = I2cStatus::PARAM_ERROR;
        return false;
    }
    m_last = i2cMemRead(m_i2c, m_addr7, reg, buf, len, m_timeoutMs);
    if (m_last != I2cStatus::OK) {
        maybeRecover();
        return false;
    }
    return true;
}

bool SW3538::writeReg(uint8_t reg, uint8_t value)
{
    if (m_i2c == nullptr) {
        m_last = I2cStatus::PARAM_ERROR;
        return false;
    }
    if (!armWriteEnable()) return false;

    m_last = i2cMemWrite(m_i2c, m_addr7, reg, &value, 1, m_timeoutMs);
    if (m_last != I2cStatus::OK) {
        maybeRecover();
        return false;
    }
    return true;
}

bool SW3538::writeRegForce(uint8_t reg, uint8_t value)
{
    /* 强制操作使能: 写 Reg0x15 = 0x20 → 0x40 → 0x80 (每次写都自带
       Reg0x10 解锁), 然后【不再重新解锁】直接写目标 —— 免得 Reg0x10
       序列把 0x15 的使能窗口冲掉。 */
    static constexpr uint8_t kForceSeq[3] = { 0x20u, 0x40u, 0x80u };
    for (const uint8_t v : kForceSeq) {
        if (!writeReg(static_cast<uint8_t>(e_SW3538_Reg::FORCE_EN), v)) return false;
    }
    if (!armWriteEnable()) return false;

    m_last = i2cMemWrite(m_i2c, m_addr7, reg, &value, 1, m_timeoutMs);
    if (m_last != I2cStatus::OK) {
        maybeRecover();
        return false;
    }
    return true;
}

/*============================================================================
 * 探测
 *============================================================================*/
std::optional<uint8_t> SW3538::getVersion()
{
    const std::optional<uint8_t> v = readReg(e_SW3538_Reg::VER);
    m_online = v.has_value();
    return v.has_value() ? std::optional<uint8_t>(static_cast<uint8_t>(*v & 0x03u))
                         : std::nullopt;
}

bool SW3538::probe()
{
    return getVersion().has_value();
}

uint8_t SW3538::getMaxPowerWatt()
{
    const std::optional<uint8_t> v = readReg(e_SW3538_Reg::MAX_POWER);
    return v.has_value() ? static_cast<uint8_t>(*v & 0x7Fu) : 0u;
}

/*============================================================================
 * ADC
 *============================================================================*/
bool SW3538::enableAdcSampling()
{
    /* Reg0x18 = VIN(6) + VOUT(7) + 通路1电流(5) 三路强制打开。
       VIN ADC 手册明确默认关闭, 不打开就读不到输入电压。 */
    return writeRegForce(static_cast<uint8_t>(e_SW3538_Reg::FORCE_ADC), 0xE0u);
}

bool SW3538::readAdc(e_SW3538_AdcCh ch, uint32_t &raw)
{
    if (!writeReg(static_cast<uint8_t>(e_SW3538_Reg::ADC_CFG),
                  static_cast<uint8_t>(ch))) {
        return false;
    }

    uint8_t d[2] = { 0u, 0u };
    if (!readRegs(static_cast<uint8_t>(e_SW3538_Reg::ADC_DATA_L), d, 2)) {
        return false;
    }
    raw = (static_cast<uint32_t>(d[1] & 0x7Fu) << 8) | d[0];
    return true;
}

/*============================================================================
 * 测量
 *============================================================================*/
bool SW3538::readMeasurement(SW3538_Measurement &m)
{
    m = SW3538_Measurement{};
    if (m_i2c == nullptr) {
        m_last = I2cStatus::PARAM_ERROR;
        return false;
    }

    /* ---- 1. 状态块: Reg0x09 (快充) + Reg0x0A (系统), 一次读 2 字节 ---- */
    uint8_t st[2] = { 0u, 0u };
    if (!readRegs(static_cast<uint8_t>(e_SW3538_Reg::FAST_STATUS), st, 2)) {
        return false;                       /* 通讯失败 → 整次无效 */
    }
    const uint8_t fast = st[0];             /* Reg0x09 */
    const uint8_t sys  = st[1];             /* Reg0x0A */

    /* ---- 2. Reg0x0D 设备在线 (与 0x09 之间隔着未定义寄存器, 单独读) ---- */
    uint8_t online = 0u;
    if (!readRegs(static_cast<uint8_t>(e_SW3538_Reg::DEV_ONLINE), &online, 1)) {
        return false;
    }

    const bool useP1 = hasPort(e_SW3538_Port::PATH1);
    const bool useP2 = hasPort(e_SW3538_Port::PATH2);

    m.fast        = ((fast & 0x80u) != 0u) || ((fast & 0x40u) != 0u);
    m.fastProt    = static_cast<uint8_t>(fast & 0x0Fu);
    m.buckOn      = (sys & 0x04u) != 0u;
    /* 没接的通路强制 false —— 硬件没接, 寄存器读回来的位没有意义,
       否则 UI 会误报"通路2 有设备"这类根本不存在的状态。 */
    m.path1On     = useP1 && ((sys  & 0x01u) != 0u);
    m.path2On     = useP2 && ((sys  & 0x02u) != 0u);
    m.path1Online = useP1 && ((online & 0x02u) != 0u);   /* Reg0x0D[1] = 通路1 */
    m.path2Online = useP2 && ((online & 0x01u) != 0u);   /* Reg0x0D[0] = 通路2 */
    m.ok = true;

    /* ---- 3. ADC: VIN (10mV/bit) / VOUT (1mV/bit), 再按使能的通路读一路电流 ---- */
    uint32_t raw = 0u;
    if (readAdc(e_SW3538_AdcCh::VIN, raw)) {
        const uint32_t mv = raw * 10u;
        m.vin_mv    = (mv > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(mv);
        m.vinValid  = true;
    }
    if (readAdc(e_SW3538_AdcCh::VOUT_CONV, raw)) {
        m.vout_mv   = (raw > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(raw);
        m.voutValid = true;
    }
    if (useP1 || useP2) {
        const e_SW3538_AdcCh iCh = useP1 ? e_SW3538_AdcCh::IOUT_PATH1
                                         : e_SW3538_AdcCh::IOUT_PATH2;
        if (readAdc(iCh, raw)) {
            /* 2.5mA/bit @5mΩ → 一般式 mA = raw * 12.5 / R(mΩ) = raw*125 / (R*10) */
            const uint32_t ma = (raw * 125u) / (static_cast<uint32_t>(m_senseMilliOhm) * 10u);
            m.iout_ma    = (ma > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(ma);
            m.ioutValid  = true;
        }
    }

    /* ---- 4. VBUS —— 屏幕要显示的是这个, 不是 VOUT ----
       VOUT = BUCK 输出 (空载约 5.1V); VBUS = 它经过 SW3538 控制的 NMOS 之后
       送到口上的电压。按 PD 规范, 没有 SINK 接入时不许 VBUS 上电 → 空载 VBUS = 0V。
       手册 ADC 通道表里没有独立的 VBUS 通道, 而 NMOS 之后的电压物理上就是
       「导通 ≈ VOUT(差 mV 级) / 关断 = 0」→ 用通路导通状态门控 VOUT 得到。

       ★ 判据当前取 Reg0x0A 的通路导通位 (path1On / path2On)。
         上板若看到恒 5.1V (说明该位恒 1) 或恒 0.00 (恒 0), 把 conducting 换成
         `m.path1Online`(有设备接入) 或 `(m.buckOn && m.path1On)` 两选一即可,
         只动这一处, 不用碰别的逻辑。 */
    const bool conducting = useP1 ? m.path1On : (useP2 ? m.path2On : false);

    if (conducting) {
        if (m.voutValid) {                /* 导通: VBUS 就是 VOUT */
            m.vbus_mv    = m.vout_mv;
            m.vbusValid  = true;
        }                                 /* 导通但 VOUT 没读到 → vbusValid=false */
    } else {
        m.vbus_mv   = 0u;                 /* 关断: 口上没电, 0V 是确定值 */
        m.vbusValid = true;
    }

    /* 功率仍按 VOUT × IOUT (BUCK 侧口径): 通路关断时 IOUT 本就为 0, 功率自然 0。
       不改的原因是 vbus 受导通位影响, 万一该位语义与预期不符会连累功率显示。 */
    m.power_mw = (static_cast<uint32_t>(m.vout_mv) * m.iout_ma) / 1000u;
    return true;
}

/*============================================================================
 * 工具
 *============================================================================*/
const char *SW3538::getProtocolName(e_SW3538_FastProt prot)
{
    /* ≤4 字: 框内 9x16 字形最多摆 4 个 (VOOC/SFCP/TFCP 已是 4 字)。
       QC3+ 的 '+' 在 9x16 字库里没有 → 与 QC3 同显 "QC3";
       PD FIX / PPS 统一显 "PD" (与 POT6 同一取舍)。 */
    switch (prot) {
    case e_SW3538_FastProt::QC2:    return "QC2";
    case e_SW3538_FastProt::QC3:    return "QC3";
    case e_SW3538_FastProt::QC3P:   return "QC3";
    case e_SW3538_FastProt::FCP:    return "FCP";
    case e_SW3538_FastProt::SCP:    return "SCP";
    case e_SW3538_FastProt::PD_FIX: return "PD";
    case e_SW3538_FastProt::PD_PPS: return "PD";
    case e_SW3538_FastProt::PE11:   return "PE1";
    case e_SW3538_FastProt::PE20:   return "PE2";
    case e_SW3538_FastProt::VOOC10: return "VOOC";
    case e_SW3538_FastProt::VOOC40: return "VOOC";
    case e_SW3538_FastProt::SFCP:   return "SFCP";
    case e_SW3538_FastProt::AFC:    return "AFC";
    case e_SW3538_FastProt::TFCP:   return "TFCP";
    case e_SW3538_FastProt::NONE:   return "";
    case e_SW3538_FastProt::UNKNOWN:
    default:                        return "";
    }
}

} // namespace LoveFinderLib
