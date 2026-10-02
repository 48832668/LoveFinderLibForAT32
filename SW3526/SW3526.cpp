/**
 * @file    SW3526.cpp
 * @brief   SW3526S 驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * I2C 通讯经库内 I2C/i2c_master 阻塞驱动直发 (实例 m_i2c), 本文件不含位操作。
 */
#include "SW3526.hpp"

extern "C" {
#include "wk_system.h"      /* wk_delay_ms (WorkBench BSP, 仿 PY32 的 LL_mDelay) */
}

namespace LoveFinderLib {

/*============================================================================
 * 寄存器读写 (统一走 i2c_master 硬件总线 / i2c_sw 软件位敲)
 *============================================================================*/
bool SW3526::writeReg(uint8_t reg, uint8_t value)
{
    if (m_useSw) {
        return swI2cMemWrite(m_sw, m_addr7, reg, &value, 1u, m_timeoutMs) == I2cStatus::OK;
    }
    if (m_i2c == nullptr) return false;
    return i2cMemWrite(m_i2c, m_addr7, reg, &value, 1u, m_timeoutMs) == I2cStatus::OK;
}

std::optional<uint8_t> SW3526::readReg(uint8_t reg)
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
bool SW3526::modifyRegBit(e_SW3526_Reg reg, uint8_t bit, bool value)
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

bool SW3526::modifyRegField(e_SW3526_Reg reg, uint8_t shift, uint8_t mask, uint8_t value)
{
    const auto v = readReg(reg);
    if (!v) return false;

    uint8_t r = static_cast<uint8_t>(*v & static_cast<uint8_t>(~mask));
    r = static_cast<uint8_t>(r | static_cast<uint8_t>((value << shift) & mask));
    return writeReg(reg, r);
}

/*============================================================================
 * 芯片信息
 *============================================================================*/
uint8_t SW3526::getChipVersion()
{
    const auto v = readReg(e_SW3526_Reg::CHIP_VER);
    return v ? static_cast<uint8_t>(*v & 0x03u) : 0u;
}

bool SW3526::probe()
{
    return readReg(e_SW3526_Reg::CHIP_VER).has_value();
}

/*============================================================================
 * 测量
 *
 * ADC 为 12-bit: 高 8 位在 VIN_L/VOUT_L/IOUT_L, 低 4 位在 ADC_LSB[3:0]。
 * 分辨率 (参考 10mΩ 检流电阻):
 *   Vin  : 160mV/LSB(高8位) + 10mV/LSB(低4位)  → 10mV/bit @12bit
 *   Vout :  96mV/LSB        +  6mV/LSB         →  6mV/bit @12bit
 *   Iout :  40mA/LSB        + 2.5mA/LSB        → 2.5mA/bit @12bit
 *============================================================================*/
void SW3526::setADCLane(e_SW3526_ADC_Lane lane)
{
    const auto v = readReg(e_SW3526_Reg::ADC_SENSE);
    if (v) {
        writeReg(e_SW3526_Reg::ADC_SENSE,
                 static_cast<uint8_t>((*v & 0xF8u) | static_cast<uint8_t>(lane)));
    }
}

uint16_t SW3526::getVin()
{
    setADCLane(e_SW3526_ADC_Lane::VIN);
    const auto hi = readReg(e_SW3526_Reg::VIN_L);
    const auto lo = readReg(e_SW3526_Reg::ADC_LSB);
    if (!hi || !lo) return 0;
    return static_cast<uint16_t>(*hi * 160u + (*lo & 0x0Fu) * 10u);
}

uint16_t SW3526::getVout()
{
    setADCLane(e_SW3526_ADC_Lane::VOUT);
    const auto hi = readReg(e_SW3526_Reg::VOUT_L);
    const auto lo = readReg(e_SW3526_Reg::ADC_LSB);
    if (!hi || !lo) return 0;
    return static_cast<uint16_t>(*hi * 96u + (*lo & 0x0Fu) * 6u);
}

uint16_t SW3526::getIout()
{
    setADCLane(e_SW3526_ADC_Lane::IOUT);
    const auto hi = readReg(e_SW3526_Reg::IOUT_L);
    const auto lo = readReg(e_SW3526_Reg::ADC_LSB);
    if (!hi || !lo) return 0;

    /* 原始值按 10mΩ 基准计算, 再按实际检流电阻缩放 */
    const float raw = static_cast<float>(*hi) * 40.0f + static_cast<float>(*lo) * 2.5f;
    const float scaled = (m_senseMilliOhm != 0u)
                       ? raw * 10.0f / static_cast<float>(m_senseMilliOhm)
                       : 0.0f;
    return static_cast<uint16_t>(scaled + 0.5f);
}

SW3526_Measurement SW3526::getAllMeasurements()
{
    SW3526_Measurement m{};
    m.vin_mv  = getVin();
    m.vout_mv = getVout();
    m.iout_ma = getIout();
    m.power_mw = static_cast<uint16_t>((static_cast<uint32_t>(m.vout_mv) * m.iout_ma) / 1000u);
    return m;
}

uint16_t SW3526::getBuckTargetVoltage()
{
    const auto h = readReg(e_SW3526_Reg::BUCK_VOL_H);
    const auto l = readReg(e_SW3526_Reg::BUCK_VOL_L);
    if (!h || !l) return 0;
    const uint16_t dac = static_cast<uint16_t>((static_cast<uint16_t>(*h) << 4) |
                                               ((*l >> 4) & 0x0Fu));
    return static_cast<uint16_t>(dac * 10u);
}

uint16_t SW3526::getCCLimit()
{
    const auto v = readReg(e_SW3526_Reg::CC_LIMIT);
    return v ? static_cast<uint16_t>(1000u + (*v & 0x3Fu) * 50u) : 0u;
}

bool SW3526::setCCLimit(uint16_t limitMa)
{
    if (limitMa < 1000u || limitMa > 4100u) return false;

    const uint8_t ctrl = static_cast<uint8_t>((limitMa - 1000u) / 50u);
    const auto v = readReg(e_SW3526_Reg::CC_LIMIT);
    if (!v) return false;
    return writeReg(e_SW3526_Reg::CC_LIMIT,
                    static_cast<uint8_t>((*v & 0xC0u) | (ctrl & 0x3Fu)));
}

/*============================================================================
 * 系统与故障状态
 *============================================================================*/
SW3526_SysStatus SW3526::getSysStatus()
{
    SW3526_SysStatus s{};
    const auto v = readReg(e_SW3526_Reg::SYS_STATUS);
    if (v) {
        s.port_open = (*v & 0x02u) != 0u;
        s.buck_on   = (*v & 0x01u) != 0u;
    }
    return s;
}

uint8_t SW3526::getCCConnection()
{
    const auto v = readReg(e_SW3526_Reg::CC_CONN);
    if (!v) return 0;
    return static_cast<uint8_t>(((*v & 0x80u) ? 0x01u : 0u) | ((*v & 0x40u) ? 0x02u : 0u));
}

SW3526_FaultFlags SW3526::getFaultStatus()
{
    SW3526_FaultFlags f{};
    const auto v = readReg(e_SW3526_Reg::FAULT_STATUS);
    if (v) {
        f.output_uv   = (*v & 0x80u) != 0u;
        f.input_uv    = (*v & 0x20u) != 0u;
        f.input_ov    = (*v & 0x10u) != 0u;
        f.die_ot_warn = (*v & 0x04u) != 0u;
        f.die_ot_shdn = (*v & 0x02u) != 0u;
        f.output_sc   = (*v & 0x01u) != 0u;
    }
    return f;
}

uint8_t SW3526::getMaxPowerReadback()
{
    const auto v = readReg(e_SW3526_Reg::MAX_POWER_RD);
    return v ? static_cast<uint8_t>(*v & 0x7Fu) : 0u;
}

/*============================================================================
 * 协议状态
 *============================================================================*/
e_SW3526_FastProt SW3526::getFastStatus()
{
    const auto v = readReg(e_SW3526_Reg::FAST_STATUS);
    if (!v) return e_SW3526_FastProt::NONE;

    uint8_t protVal = static_cast<uint8_t>(*v & 0x0Fu);
    if (protVal > 11u) {
        wk_delay_ms(1);
        const auto retry = readReg(e_SW3526_Reg::FAST_STATUS);
        if (retry) protVal = static_cast<uint8_t>(*retry & 0x0Fu);
        if (protVal > 11u) protVal = 0u;
    }
    return static_cast<e_SW3526_FastProt>(protVal);
}

e_SW3526_PD_Version SW3526::getPDVersion()
{
    const auto v = readReg(e_SW3526_Reg::FAST_STATUS);
    if (!v) return e_SW3526_PD_Version::NONE;
    return static_cast<e_SW3526_PD_Version>((*v >> 4) & 0x03u);
}

bool SW3526::isFastCharging()
{
    const auto v = readReg(e_SW3526_Reg::FAST_STATUS);
    return v.has_value() && ((*v & 0x80u) != 0u);
}

bool SW3526::isVoltageFast()
{
    const auto v = readReg(e_SW3526_Reg::FAST_STATUS);
    return v.has_value() && ((*v & 0x40u) != 0u);
}

const char *SW3526::getProtocolName(e_SW3526_FastProt prot)
{
    switch (prot) {
        case e_SW3526_FastProt::NONE:    return "NONE";
        case e_SW3526_FastProt::QC2:     return "QC2 ";
        case e_SW3526_FastProt::QC3:     return "QC3 ";
        case e_SW3526_FastProt::FCP:     return "FCP ";
        case e_SW3526_FastProt::SCP:     return "SCP ";
        case e_SW3526_FastProt::PD_FIX:  return "PDFX";
        case e_SW3526_FastProt::PD_PPS:  return "PPS ";
        case e_SW3526_FastProt::PE11:    return "PE11";
        case e_SW3526_FastProt::PE20:    return "PE20";
        case e_SW3526_FastProt::VOOC:    return "VOOC";
        case e_SW3526_FastProt::SFCP:    return "SFCP";
        case e_SW3526_FastProt::AFC:     return "AFC ";
        case e_SW3526_FastProt::UNKNOWN: return "UNKN";
    }
    return "UNKN";
}

/*============================================================================
 * 功率配置
 *============================================================================*/
uint8_t SW3526::getWattConfig()
{
    const auto v = readReg(e_SW3526_Reg::WATT_CONFIG);
    return v ? static_cast<uint8_t>(*v & 0x7Fu) : 0u;
}

bool SW3526::setMaxPower(uint8_t watts)
{
    return writeReg(e_SW3526_Reg::POWER_CFG, static_cast<uint8_t>(watts & 0x3Fu));
}

bool SW3526::setPowerSource(bool useRegister)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG5, 2, useRegister);
}

bool SW3526::setPortType(e_SW3526_PortType portType)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG5, 0, portType == e_SW3526_PortType::TYPE_A);
}

/*============================================================================
 * Buck 控制
 *============================================================================*/
void SW3526::unlockRegisters()
{
    writeReg(e_SW3526_Reg::UNLOCK_1, 0x20);
    wk_delay_ms(1);
    writeReg(e_SW3526_Reg::UNLOCK_1, 0x40);
    wk_delay_ms(1);
    writeReg(e_SW3526_Reg::UNLOCK_1, 0x80);
    wk_delay_ms(1);
}

void SW3526::forceBuckOff1S()
{
    const auto v = readReg(e_SW3526_Reg::BUCK_CTRL);
    if (v) writeReg(e_SW3526_Reg::BUCK_CTRL, static_cast<uint8_t>(*v | 0x80u));
}

bool SW3526::setBuckOffCCDisconnect(bool disconnect)
{
    return modifyRegBit(e_SW3526_Reg::BUCK_CTRL, 6, disconnect);
}

bool SW3526::setWireCompEnable(bool enable)
{
    /* 寄存器位为 0=使能, 1=关闭 */
    return modifyRegBit(e_SW3526_Reg::BUCK_CTRL, 5, !enable);
}

bool SW3526::setBuckFreq(uint16_t freqKhz)
{
    return modifyRegBit(e_SW3526_Reg::FREQ_CONFIG, 7, freqKhz >= 312u);
}

bool SW3526::setWireCompSize(e_SW3526_WireComp comp)
{
    return modifyRegBit(e_SW3526_Reg::FREQ_CONFIG, 4,
                        comp == e_SW3526_WireComp::MV_60_PER_A);
}

bool SW3526::setNoLoadVoltage(e_SW3526_NoLoadVolt volt)
{
    const auto v = readReg(e_SW3526_Reg::NOLOAD_VOLT);
    if (!v) return false;
    return writeReg(e_SW3526_Reg::NOLOAD_VOLT,
                    static_cast<uint8_t>((*v & 0x3Fu) | (static_cast<uint8_t>(volt) << 6)));
}

/*============================================================================
 * 快充协议控制
 *============================================================================*/
const char *SW3526::setFastMode(e_SW3526_FastMode mode)
{
    unlockRegisters();

    const auto cfg2 = readReg(e_SW3526_Reg::CHG_CFG2);
    const auto cfg3 = readReg(e_SW3526_Reg::CHG_CFG3);
    if (!cfg2 || !cfg3) return "Error";

    const auto cfg6 = readReg(e_SW3526_Reg::CHG_CFG6);

    switch (mode) {
        case e_SW3526_FastMode::FAST_ON:
            writeReg(e_SW3526_Reg::CHG_CFG2, static_cast<uint8_t>(*cfg2 & 0x40u));
            writeReg(e_SW3526_Reg::CHG_CFG3, static_cast<uint8_t>(*cfg3 & 0x02u));
            if (cfg6) writeReg(e_SW3526_Reg::CHG_CFG6, static_cast<uint8_t>(*cfg6 & ~0x04u));
            return "FastOn";

        case e_SW3526_FastMode::ONLY_PD:
            writeReg(e_SW3526_Reg::CHG_CFG2, static_cast<uint8_t>(*cfg2 | 0x3Fu));
            writeReg(e_SW3526_Reg::CHG_CFG3, static_cast<uint8_t>(*cfg3 & 0x02u));
            if (cfg6) writeReg(e_SW3526_Reg::CHG_CFG6, static_cast<uint8_t>(*cfg6 & ~0x04u));
            return "OnlyPD";

        case e_SW3526_FastMode::NO_FAST:
            writeReg(e_SW3526_Reg::CHG_CFG2, static_cast<uint8_t>(*cfg2 | 0x3Fu));
            writeReg(e_SW3526_Reg::CHG_CFG3, static_cast<uint8_t>(*cfg3 | 0x01u));
            writeReg(e_SW3526_Reg::CHG_CFG3, static_cast<uint8_t>(*cfg3 | 0xC0u));
            if (cfg6) writeReg(e_SW3526_Reg::CHG_CFG6, static_cast<uint8_t>(*cfg6 | 0x04u));
            return "NoFast";
    }
    return "";
}

/* 使能类寄存器: 位为 0 表示使能, 1 表示关闭 */
#define LF_SW3526_IMPL_SET_ENABLE(NAME, REG, BIT)                       \
    bool SW3526::set##NAME##Enable(bool enable)                          \
    {                                                                    \
        return modifyRegBit(e_SW3526_Reg::REG, BIT, !enable);            \
    }

LF_SW3526_IMPL_SET_ENABLE(FastCharge, CHG_CFG6, 2)
LF_SW3526_IMPL_SET_ENABLE(PD,         CHG_CFG3, 0)
LF_SW3526_IMPL_SET_ENABLE(SCP,        CHG_CFG2, 7)
LF_SW3526_IMPL_SET_ENABLE(QC2,        CHG_CFG2, 4)
LF_SW3526_IMPL_SET_ENABLE(QC3,        CHG_CFG2, 3)
LF_SW3526_IMPL_SET_ENABLE(FCP,        CHG_CFG2, 2)
LF_SW3526_IMPL_SET_ENABLE(AFC,        CHG_CFG2, 1)
LF_SW3526_IMPL_SET_ENABLE(PE,         CHG_CFG2, 0)
LF_SW3526_IMPL_SET_ENABLE(SFCP,       CHG_CFG2, 5)

#undef LF_SW3526_IMPL_SET_ENABLE

bool SW3526::setPDVoltageEnable(bool v9, bool v12, bool v15, bool v20)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG3);
    if (!v) return false;

    uint8_t r = *v;
    r = v9  ? static_cast<uint8_t>(r & ~0x04u) : static_cast<uint8_t>(r | 0x04u);
    r = v12 ? static_cast<uint8_t>(r & ~0x08u) : static_cast<uint8_t>(r | 0x08u);
    r = v15 ? static_cast<uint8_t>(r & ~0x10u) : static_cast<uint8_t>(r | 0x10u);
    r = v20 ? static_cast<uint8_t>(r & ~0x20u) : static_cast<uint8_t>(r | 0x20u);
    return writeReg(e_SW3526_Reg::CHG_CFG3, r);
}

bool SW3526::setPPSEnable(bool pps0, bool pps1)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG3);
    if (!v) return false;

    uint8_t r = *v;
    r = pps0 ? static_cast<uint8_t>(r & ~0x40u) : static_cast<uint8_t>(r | 0x40u);
    r = pps1 ? static_cast<uint8_t>(r & ~0x80u) : static_cast<uint8_t>(r | 0x80u);
    return writeReg(e_SW3526_Reg::CHG_CFG3, r);
}

bool SW3526::setSCPMaxPower(e_SW3526_SCP_MaxPower power)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG0, 7, power == e_SW3526_SCP_MaxPower::W40);
}

bool SW3526::setHVSCPEnable(bool enable)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG1, 6, enable);
}

bool SW3526::setPE20HighVoltEnable(bool enable)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG1, 5, enable);
}

bool SW3526::setDPDMEnable(bool enable)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG4, 5, !enable);
}

bool SW3526::setSCPMaxCurrent(e_SW3526_SCP_MaxCurrent current)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG4);
    if (!v) return false;

    const uint8_t val = static_cast<uint8_t>(current);
    uint8_t r = static_cast<uint8_t>(*v & static_cast<uint8_t>(~0x88u));
    if ((val & 0x02u) != 0u) r = static_cast<uint8_t>(r | 0x80u);
    if ((val & 0x01u) != 0u) r = static_cast<uint8_t>(r | 0x08u);
    return writeReg(e_SW3526_Reg::CHG_CFG4, r);
}

bool SW3526::setNonPdMaxVoltage(e_SW3526_NonPdMaxVolt volt)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG4);
    if (!v) return false;
    return writeReg(e_SW3526_Reg::CHG_CFG4,
                    static_cast<uint8_t>((*v & 0xFCu) | (static_cast<uint8_t>(volt) & 0x03u)));
}

bool SW3526::setQCWireCompEnable(bool enable)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG0, 6, !enable);
}

bool SW3526::setPdoVinLink(bool link)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG0, 5, link);
}

bool SW3526::setPdo5V2ARebroadcast(bool enable)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG6, 0, enable);
}

/*============================================================================
 * PD 命令
 *============================================================================*/
bool SW3526::sendPDCmd(e_SW3526_PDCmd cmd)
{
    return writeReg(e_SW3526_Reg::PD_CMD,
                    static_cast<uint8_t>(0x80u | (static_cast<uint8_t>(cmd) & 0x0Fu)));
}

bool SW3526::sendPDSourceCap()
{
    return writeReg(e_SW3526_Reg::PD_SRC_CAP, 0x80);
}

/*============================================================================
 * 双芯片动态功率
 *============================================================================*/
bool SW3526::setDualChipResetPower(bool resetAll, bool enable)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG7);
    if (!v) return false;

    uint8_t r = static_cast<uint8_t>(*v & 0x3Fu);
    if (resetAll) r = static_cast<uint8_t>(r | 0x80u);
    if (!enable)  r = static_cast<uint8_t>(r | 0x40u);
    return writeReg(e_SW3526_Reg::CHG_CFG7, r);
}

/*============================================================================
 * Apple / Samsung
 *============================================================================*/
bool SW3526::setAppleModeEnable(bool enable)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG8, 2, enable);
}

bool SW3526::setNonPdPowerLimit18W(bool limit)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG8, 6, limit);
}

bool SW3526::setPDResponseTime(bool fast)
{
    return modifyRegBit(e_SW3526_Reg::CHG_CFG8, 0, fast);
}

bool SW3526::setAppleSamsungRefVolt(uint8_t voltage)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG9);
    if (!v) return false;
    return writeReg(e_SW3526_Reg::CHG_CFG9,
                    static_cast<uint8_t>((*v & 0xCFu) | ((voltage & 0x03u) << 2)));
}

bool SW3526::setAppleDriveCurrent(uint8_t current)
{
    const auto v = readReg(e_SW3526_Reg::CHG_CFG9);
    if (!v) return false;
    return writeReg(e_SW3526_Reg::CHG_CFG9,
                    static_cast<uint8_t>((*v & 0xFCu) | (current & 0x03u)));
}

} // namespace LoveFinderLib
