/**
 * @file    SW2305.cpp
 * @brief   SW2305 驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * 寄存器映射依据 SW2305_REG V1.2 (珠海智融科技)。
 */
#include "SW2305.hpp"

extern "C" {
#include "wk_system.h"      /* wk_delay_ms (WorkBench BSP, 仿 PY32 的 LL_mDelay) */
}

namespace LoveFinderLib {

/*============================================================================
 * 寄存器读写 (统一走 i2c_master 硬件总线 / i2c_sw 软件位敲)
 *============================================================================*/
bool SW2305::writeReg(uint8_t reg, uint8_t value)
{
    if (m_useSw) {
        return swI2cMemWrite(m_sw, m_addr7, reg, &value, 1u, m_timeoutMs) == I2cStatus::OK;
    }
    if (m_i2c == nullptr) return false;
    return i2cMemWrite(m_i2c, m_addr7, reg, &value, 1u, m_timeoutMs) == I2cStatus::OK;
}

std::optional<uint8_t> SW2305::readReg(uint8_t reg)
{
    if (m_useSw) {
        return swI2cReadReg8(m_sw, m_addr7, reg, m_timeoutMs);
    }
    if (m_i2c == nullptr) return std::nullopt;
    return i2cReadReg8(m_i2c, m_addr7, reg, m_timeoutMs);
}

/*============================================================================
 * 位域辅助
 *============================================================================*/
bool SW2305::modifyRegBit(e_SW2305_Reg reg, uint8_t bit, bool value)
{
    const auto v = readReg(reg);
    if (!v) return false;

    uint8_t r = *v;
    if (value) {
        r = static_cast<uint8_t>(r | static_cast<uint8_t>(1u << bit));
    } else {
        r = static_cast<uint8_t>(r & static_cast<uint8_t>(~(1u << bit)));
    }
    return writeReg(reg, r);
}

bool SW2305::modifyRegField(e_SW2305_Reg reg, uint8_t shift, uint8_t mask, uint8_t value)
{
    const auto v = readReg(reg);
    if (!v) return false;

    uint8_t r = static_cast<uint8_t>(*v & static_cast<uint8_t>(~mask));
    r = static_cast<uint8_t>(r | static_cast<uint8_t>((value << shift) & mask));
    return writeReg(reg, r);
}

/*============================================================================
 * 写保护解锁
 *
 * 数据手册 2.10 / 2.12: 要操作 reg0x14 与 reg0xA0~0xBF 需先向 reg0x12 依次
 * 写入 0x20 → 0x40 → 0x80；操作 reg0x16 需先向 reg0x15 写同样的序列。
 *============================================================================*/
bool SW2305::unlockSequence(e_SW2305_Reg unlockReg)
{
    if (!writeReg(unlockReg, 0x20)) return false;
    wk_delay_ms(1);
    if (!writeReg(unlockReg, 0x40)) return false;
    wk_delay_ms(1);
    if (!writeReg(unlockReg, 0x80)) return false;
    wk_delay_ms(1);
    return true;
}

bool SW2305::unlockForConfig()
{
    return unlockSequence(e_SW2305_Reg::UNLOCK_0);
}

bool SW2305::unlockForForceCtrl()
{
    return unlockSequence(e_SW2305_Reg::UNLOCK_1);
}

/*============================================================================
 * 芯片信息
 *============================================================================*/
uint8_t SW2305::getChipVersion()
{
    const auto v = readReg(e_SW2305_Reg::CHIP_VER);
    return v ? static_cast<uint8_t>(*v & 0x03u) : 0u;
}

bool SW2305::probe()
{
    return readReg(e_SW2305_Reg::CHIP_VER).has_value();
}

/*============================================================================
 * ADC 12-bit
 *
 * 流程: 写 0x3B 选通道 → 数据锁存到 0x3C(高8) / 0x3D(低4)。
 * 必须先写 0x3B，否则 0x3C/0x3D 的高低字节可能不对应。
 *============================================================================*/
bool SW2305::selectADCChannel(e_SW2305_ADC_Channel ch)
{
    return writeReg(e_SW2305_Reg::ADC_CFG, static_cast<uint8_t>(ch) & 0x07u);
}

uint16_t SW2305::readADC12()
{
    const auto hi = readReg(e_SW2305_Reg::ADC_DATA_H);
    const auto lo = readReg(e_SW2305_Reg::ADC_DATA_L);
    if (!hi || !lo) return 0;

    return static_cast<uint16_t>((static_cast<uint16_t>(*hi) << 4) |
                                 (static_cast<uint16_t>(*lo) & 0x0Fu));
}

/*============================================================================
 * 测量
 *============================================================================*/
uint16_t SW2305::getVin()
{
    if (!selectADCChannel(e_SW2305_ADC_Channel::VIN)) return 0;
    /* 12-bit 模式: 7.5mV/bit */
    const float mv = static_cast<float>(readADC12()) * 7.5f;
    return static_cast<uint16_t>(mv + 0.5f);
}

uint16_t SW2305::getVbus()
{
    if (!selectADCChannel(e_SW2305_ADC_Channel::VBUS)) return 0;
    const float mv = static_cast<float>(readADC12()) * 7.5f;
    return static_cast<uint16_t>(mv + 0.5f);
}

/**
 * @brief 输出电流。
 *
 * 数据手册: adc_ich 12-bit 模式 3.125mA/bit —— 该值基于 **10mΩ** 检流电阻。
 * 本项目外挂 5mΩ, 同样电流下采样电压翻倍, ADC 读数翻倍,
 * 因此实际电流 = adc * 3.125 * (10 / Rsense_mΩ)。
 */
uint16_t SW2305::getIch()
{
    if (!selectADCChannel(e_SW2305_ADC_Channel::ICH)) return 0;

    const float adc = static_cast<float>(readADC12());
    const float refOhm = static_cast<float>(SW2305_SENSE_REFERENCE_MILLIOHM);
    const float scale = (m_senseMilliOhm != 0u)
                      ? refOhm / static_cast<float>(m_senseMilliOhm)
                      : 0.0f;

    const float ma = adc * 3.125f * scale;
    return static_cast<uint16_t>(ma + 0.5f);
}

/**
 * @brief Die 温度。
 *
 * 数据手册: adc_diet 0.1488℃/bit, Tdiet = (adc_diet[11:0] - 1848) / 6.72 ℃
 */
float SW2305::getTdiet()
{
    if (!selectADCChannel(e_SW2305_ADC_Channel::TDIET)) return 0.0f;

    const float adc = static_cast<float>(readADC12());
    return (adc - 1848.0f) / 6.72f;
}

SW2305_Measurement SW2305::getAllMeasurements()
{
    SW2305_Measurement m{};

    if (!selectADCChannel(e_SW2305_ADC_Channel::VIN)) return m;
    m.vin_mv = static_cast<uint16_t>(static_cast<float>(readADC12()) * 7.5f + 0.5f);

    if (!selectADCChannel(e_SW2305_ADC_Channel::VBUS)) return m;
    m.vbus_mv = static_cast<uint16_t>(static_cast<float>(readADC12()) * 7.5f + 0.5f);

    if (!selectADCChannel(e_SW2305_ADC_Channel::ICH)) return m;
    {
        const float scale = (m_senseMilliOhm != 0u)
                          ? static_cast<float>(SW2305_SENSE_REFERENCE_MILLIOHM) /
                            static_cast<float>(m_senseMilliOhm)
                          : 0.0f;
        m.ich_ma = static_cast<uint16_t>(static_cast<float>(readADC12()) * 3.125f * scale + 0.5f);
    }

    if (!selectADCChannel(e_SW2305_ADC_Channel::TDIET)) return m;
    m.tdiet_c = (static_cast<float>(readADC12()) - 1848.0f) / 6.72f;

    m.valid = true;
    return m;
}

/**
 * @brief 只读 VBUS + 电流 —— 对 SW2305 的写入压到"必需的选通道"。
 *
 * 本项目对 SW2305 只要快充协议状态 (纯读 0x??) + VBUS/电流,
 * 所以不选 VIN / TDIET 通道: 每轮询比 getAllMeasurements 少写 2 次 ADC_CFG(0x3B)。
 * (选通道的那次写是硬件必需的 —— 不先写 0x3B, 0x3C/0x3D 的高低字节不对应。)
 */
SW2305_Measurement SW2305::getVbusIch()
{
    SW2305_Measurement m{};

    if (!selectADCChannel(e_SW2305_ADC_Channel::VBUS)) return m;
    m.vbus_mv = static_cast<uint16_t>(static_cast<float>(readADC12()) * 7.5f + 0.5f);

    if (!selectADCChannel(e_SW2305_ADC_Channel::ICH)) return m;
    {
        const float scale = (m_senseMilliOhm != 0u)
                          ? static_cast<float>(SW2305_SENSE_REFERENCE_MILLIOHM) /
                            static_cast<float>(m_senseMilliOhm)
                          : 0.0f;
        m.ich_ma = static_cast<uint16_t>(static_cast<float>(readADC12()) * 3.125f * scale + 0.5f);
    }

    m.valid = true;
    return m;
}

/*============================================================================
 * 设置电压 / 限流
 *============================================================================*/
uint16_t SW2305::getTargetVoltage()
{
    const auto h = readReg(e_SW2305_Reg::DAC_VOL_H);
    const auto l = readReg(e_SW2305_Reg::DAC_VOL_L);
    if (!h || !l) return 0;

    /* dac_vol[11:0] = REG0x03[7:0] << 4 | REG0x04[7:4] */
    const uint16_t dac = static_cast<uint16_t>((static_cast<uint16_t>(*h) << 4) |
                                               ((*l >> 4) & 0x0Fu));
    return static_cast<uint16_t>(dac * 10u);
}

uint16_t SW2305::getCCLimit()
{
    const auto v = readReg(e_SW2305_Reg::CTRL_ICC);
    if (!v) return 0;
    /* 限流 = 1000mA + ctrl_icc[6:0] * 50mA */
    return static_cast<uint16_t>(1000u + (*v & 0x7Fu) * 50u);
}

bool SW2305::setCCLimit(uint16_t limitMa)
{
    if (limitMa < 1000u || limitMa > 4100u) return false;

    const auto v = readReg(e_SW2305_Reg::CTRL_ICC);
    if (!v) return false;

    const uint8_t ctrl = static_cast<uint8_t>((limitMa - 1000u) / 50u);
    return writeReg(e_SW2305_Reg::CTRL_ICC,
                    static_cast<uint8_t>((*v & 0x80u) | (ctrl & 0x7Fu)));
}

/*============================================================================
 * 快充状态
 *============================================================================*/
e_SW2305_FastProt SW2305::getFastProtocol()
{
    const auto v = readReg(e_SW2305_Reg::FAST_STATUS);
    if (!v) return e_SW2305_FastProt::NONE;

    uint8_t prot = static_cast<uint8_t>(*v & 0x0Fu);
    /* 有效值: 0,1,2,3,5,6,7,8,9,A,C,D; 其余为 reserved */
    const bool valid = (prot <= 0x03u) || (prot >= 0x05u && prot <= 0x0Au) ||
                       (prot == 0x0Cu) || (prot == 0x0Du);
    if (!valid) {
        wk_delay_ms(1);
        const auto retry = readReg(e_SW2305_Reg::FAST_STATUS);
        if (retry) {
            prot = static_cast<uint8_t>(*retry & 0x0Fu);
        }
        const bool valid2 = (prot <= 0x03u) || (prot >= 0x05u && prot <= 0x0Au) ||
                            (prot == 0x0Cu) || (prot == 0x0Du);
        if (!valid2) prot = 0u;
    }
    return static_cast<e_SW2305_FastProt>(prot);
}

e_SW2305_PD_Version SW2305::getPDVersion()
{
    const auto v = readReg(e_SW2305_Reg::FAST_STATUS);
    if (!v) return e_SW2305_PD_Version::NONE;
    return static_cast<e_SW2305_PD_Version>((*v >> 4) & 0x03u);
}

bool SW2305::isFastCharging()
{
    const auto v = readReg(e_SW2305_Reg::FAST_STATUS);
    return v.has_value() && ((*v & 0x80u) != 0u);
}

bool SW2305::isVoltageFast()
{
    const auto v = readReg(e_SW2305_Reg::FAST_STATUS);
    return v.has_value() && ((*v & 0x40u) != 0u);
}

const char *SW2305::getProtocolName(e_SW2305_FastProt prot)
{
    switch (prot) {
        case e_SW2305_FastProt::NONE:   return "NONE";
        case e_SW2305_FastProt::QC2:    return "QC2 ";
        case e_SW2305_FastProt::QC3:    return "QC3 ";
        case e_SW2305_FastProt::FCP:    return "FCP ";
        case e_SW2305_FastProt::SCP:    return "SCP ";
        case e_SW2305_FastProt::PD_FIX: return "PDFX";
        case e_SW2305_FastProt::PD_PPS: return "PPS ";
        case e_SW2305_FastProt::PE11:   return "PE11";
        case e_SW2305_FastProt::PE20:   return "PE20";
        case e_SW2305_FastProt::VOOC:   return "VOOC";
        case e_SW2305_FastProt::SFCP:   return "SFCP";
        case e_SW2305_FastProt::AFC:    return "AFC ";
        case e_SW2305_FastProt::UNKNOWN:return "UNKN";
    }
    return "UNKN";
}

/*============================================================================
 * 系统状态
 *============================================================================*/
SW2305_SysStatus0 SW2305::getSysStatus0()
{
    SW2305_SysStatus0 s{};
    const auto v = readReg(e_SW2305_Reg::SYS_STATUS0);
    if (v) {
        s.pull_photo_abnormal = (*v & 0x10u) != 0u;
        s.cc_loop_closed      = (*v & 0x04u) != 0u;
        s.cable_comp_active   = (*v & 0x02u) != 0u;
        s.path_open           = (*v & 0x01u) != 0u;
    }
    return s;
}

SW2305_SysStatus1 SW2305::getSysStatus1()
{
    SW2305_SysStatus1 s{};
    const auto v = readReg(e_SW2305_Reg::SYS_STATUS1);
    if (v) {
        s.vin_over_25v = (*v & 0x20u) != 0u;
        s.overcurrent  = (*v & 0x10u) != 0u;
        s.die_overheat = (*v & 0x08u) != 0u;
        s.vin_ov       = (*v & 0x02u) != 0u;
        s.vin_uv       = (*v & 0x01u) != 0u;
    }
    return s;
}

SW2305_SysStatus2 SW2305::getSysStatus2()
{
    SW2305_SysStatus2 s{};
    const auto v = readReg(e_SW2305_Reg::SYS_STATUS2);
    if (v) {
        s.cc1_ov    = (*v & 0x80u) != 0u;
        s.cc2_ov    = (*v & 0x40u) != 0u;
        s.dp_ov     = (*v & 0x20u) != 0u;
        s.dm_ov     = (*v & 0x10u) != 0u;
        s.low_power = (*v & 0x04u) != 0u;
        s.ocp       = (*v & 0x01u) != 0u;
    }
    return s;
}

bool SW2305::isOnline()
{
    const auto v = readReg(e_SW2305_Reg::SYS_STATUS3);
    return v.has_value() && ((*v & 0x80u) != 0u);
}

/*============================================================================
 * 强制控制 (REG 0x16)
 *
 * 注意: 操作 0x16 前必须先 unlockForForceCtrl()。
 *============================================================================*/
bool SW2305::forcePathOn()
{
    return modifyRegBit(e_SW2305_Reg::FORCE_CTRL, 3, true);
}

bool SW2305::forcePathOff()
{
    return modifyRegBit(e_SW2305_Reg::FORCE_CTRL, 2, true);
}

bool SW2305::forceDacControl(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FORCE_CTRL, 1, enable);
}

bool SW2305::forceIccControl(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FORCE_CTRL, 0, enable);
}

bool SW2305::clearForceCtrl()
{
    return writeReg(e_SW2305_Reg::FORCE_CTRL, 0x00);
}

/*============================================================================
 * 线补 (REG 0x14)
 *
 * Bit2: 0 = 打开线补, 1 = 关闭线补
 * Bit1: 写 1 触发 CC un-driving 1s, 之后自动清零
 *============================================================================*/
bool SW2305::setCableCompEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::CONN_CTRL, 2, !enable);
}

bool SW2305::triggerCCUnDriving()
{
    return modifyRegBit(e_SW2305_Reg::CONN_CTRL, 1, true);
}

/*============================================================================
 * OTP 配置 (REG 0xA0~0xBF, 需 unlockForConfig)
 *============================================================================*/
bool SW2305::setWireComp(e_SW2305_WireComp comp)
{
    return modifyRegField(e_SW2305_Reg::WIRE_COMP, 6, 0xC0u, static_cast<uint8_t>(comp));
}

bool SW2305::setOtpThreshold(e_SW2305_OtpThreshold th)
{
    return modifyRegField(e_SW2305_Reg::PROT_CFG, 2, 0x0Cu, static_cast<uint8_t>(th));
}

bool SW2305::setVoutOffset(e_SW2305_VoutOffset offset)
{
    return modifyRegField(e_SW2305_Reg::VOUT_OFFSET, 6, 0xC0u, static_cast<uint8_t>(offset));
}

bool SW2305::setOnlineCurrent(e_SW2305_OnlineCurrent current)
{
    return modifyRegField(e_SW2305_Reg::ONLINE_CFG, 0, 0x03u, static_cast<uint8_t>(current));
}

/**
 * @brief 最大功率 (REG 0xAF)。
 * Bit7 = 1 选择寄存器配置方式, [6:0] = 功率 W/bit。
 */
bool SW2305::setMaxPowerByRegister(uint8_t watts)
{
    const auto v = readReg(e_SW2305_Reg::POWER_CFG);
    if (!v) return false;

    const uint8_t r = static_cast<uint8_t>(0x80u | (watts & 0x7Fu));
    return writeReg(e_SW2305_Reg::POWER_CFG, r);
}

/*============================================================================
 * PD / 快充协议配置 (需 unlockForConfig)
 *
 * 这些寄存器的位语义为「0 = 使能, 1 = 不使能」。
 *============================================================================*/

/** REG 0xB3 Bit0: PD 使能 */
bool SW2305::setPDEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::PD_CFG0, 0, !enable);
}

/** REG 0xB5 Bit[3:0]: PD fixed 20V/15V/12V/9V 使能 */
bool SW2305::setPDVoltageEnable(bool v9, bool v12, bool v15, bool v20)
{
    const auto v = readReg(e_SW2305_Reg::PD_CFG2);
    if (!v) return false;

    uint8_t r = *v;
    r = v9  ? static_cast<uint8_t>(r & ~0x01u) : static_cast<uint8_t>(r | 0x01u);
    r = v12 ? static_cast<uint8_t>(r & ~0x02u) : static_cast<uint8_t>(r | 0x02u);
    r = v15 ? static_cast<uint8_t>(r & ~0x04u) : static_cast<uint8_t>(r | 0x04u);
    r = v20 ? static_cast<uint8_t>(r & ~0x08u) : static_cast<uint8_t>(r | 0x08u);
    return writeReg(e_SW2305_Reg::PD_CFG2, r);
}

/** REG 0xB5 Bit[7:4]: PPS3/PPS2/PPS1/PPS0 使能 */
bool SW2305::setPPSEnable(bool pps0, bool pps1, bool pps2, bool pps3)
{
    const auto v = readReg(e_SW2305_Reg::PD_CFG2);
    if (!v) return false;

    uint8_t r = *v;
    r = pps0 ? static_cast<uint8_t>(r & ~0x10u) : static_cast<uint8_t>(r | 0x10u);
    r = pps1 ? static_cast<uint8_t>(r & ~0x20u) : static_cast<uint8_t>(r | 0x20u);
    r = pps2 ? static_cast<uint8_t>(r & ~0x40u) : static_cast<uint8_t>(r | 0x40u);
    r = pps3 ? static_cast<uint8_t>(r & ~0x80u) : static_cast<uint8_t>(r | 0x80u);
    return writeReg(e_SW2305_Reg::PD_CFG2, r);
}

/** REG 0xB0 Bit1: 快充使能总开关 */
bool SW2305::setFastChargeEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG2, 1, !enable);
}

/** REG 0xB0 Bit2: QC2.0 使能 */
bool SW2305::setQC2Enable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG2, 2, !enable);
}

/** REG 0xB0 Bit3: QC3.0 使能 */
bool SW2305::setQC3Enable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG2, 3, !enable);
}

/** REG 0xB0 Bit5/Bit4: 高压/低压 SCP 使能 */
bool SW2305::setSCPEnable(bool hv, bool lv)
{
    const auto v = readReg(e_SW2305_Reg::FAST_CFG2);
    if (!v) return false;

    uint8_t r = *v;
    r = hv ? static_cast<uint8_t>(r & ~0x20u) : static_cast<uint8_t>(r | 0x20u);
    r = lv ? static_cast<uint8_t>(r & ~0x10u) : static_cast<uint8_t>(r | 0x10u);
    return writeReg(e_SW2305_Reg::FAST_CFG2, r);
}

/** REG 0xB0 Bit0: BC1.2 使能 */
bool SW2305::setBC12Enable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG2, 0, !enable);
}

/** REG 0xB1 Bit3: FCP 使能 */
bool SW2305::setFCPEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG3, 3, !enable);
}

/** REG 0xB1 Bit2: AFC 使能 */
bool SW2305::setAFCEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG3, 2, !enable);
}

/** REG 0xB1 Bit1: PE 使能 */
bool SW2305::setPEEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG3, 1, !enable);
}

/** REG 0xB1 Bit4: SFCP 使能 */
bool SW2305::setSFCPEnable(bool enable)
{
    return modifyRegBit(e_SW2305_Reg::FAST_CFG3, 4, !enable);
}

} // namespace LoveFinderLib
