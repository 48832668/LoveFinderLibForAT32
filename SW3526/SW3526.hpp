/**
 * @file    SW3526.hpp
 * @brief   SW3526S USB PD / 快充控制器驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计要点 (仿 LoveFinderLibForPY32_LL: 直绑工程头, 无平台抽象层)
 * ============================================================================
 *  - I2C 收发经库内 I2C/i2c_master 阻塞驱动直发, 本驱动只持有
 *    `i2c_type *` 外设实例 (工程 lf_config.h 提供实例宏, 如 LF_I2C_BUS0)。
 *    → CH1 传 I2C1 (LF_I2C_BUS0)
 *    → 多条总线时传对应外设 (I2C1/I2C2...)
 *  - 无硬件 I2C 可用时走软件位敲: `initBitbang(SDA, SCL, ...)` (I2C/i2c_sw)。
 *    AT32F421 只有 I2C1/I2C2, 第三条总线 (如 907 的 IIC3) 必须用它。
 *  - 寄存器映射基于 SW3526S_REG V1.0.0 (2023-01-05)
 *
 * ============================================================================
 *  典型用法
 * ============================================================================
 *    LoveFinderLib::SW3526 ch1;
 *    ch1.init(LF_I2C_BUS0, 0x3C, "CH1");  // LF_I2C_BUS0 = I2C1 (lf_config.h)
 *    ch1.setSenseResistor(10);          // 10mΩ
 *    if (ch1.getChipVersion() == 0) ... // 探测
 *
 *    // 软件 I2C (第三条总线):
 *    LoveFinderLib::SW3526 a1;
 *    a1.initBitbang(IIC3S_SDA_GPIO_PORT, IIC3S_SDA_PIN,
 *                   IIC3S_SCL_GPIO_PORT, IIC3S_SCL_PIN, 0x3C, "A1");
 */
#ifndef SW3526_HPP
#define SW3526_HPP

#include "lf_config.h"       /* ★ 工程侧: I2C 实例宏 + lf_get_tick() 时基 */
#include "i2c_master.hpp"    /* 阻塞式 I2C 主机驱动 (I2cStatus / i2cMemRead...) */
#include "i2c_sw.hpp"        /* 软件位敲 I2C (SwI2cBus / swI2cMemRead...) */

#include <cstdint>
#include <cstddef>
#include <optional>
#include <array>
#include <functional>

namespace LoveFinderLib {

/*============================================================================
 * 编译期默认值 (可在 lf_config.h 中覆盖)
 *============================================================================*/
#ifndef SW3526_I2C_ADDR_7BIT
#define SW3526_I2C_ADDR_7BIT    0x3C    /* 7-bit 地址 */
#endif

#ifndef SW3526_I2C_TIMEOUT_MS
#define SW3526_I2C_TIMEOUT_MS   100
#endif

#ifndef SW3526_SENSE_MILLIOHM
#define SW3526_SENSE_MILLIOHM   10      /* 默认检流电阻 10mΩ */
#endif

/** 软件 I2C 半周期延时 (µs) → 标称 100kHz, 可在 lf_config.h 覆盖。 */
#ifndef SW3526_SW_I2C_HALF_PERIOD_US
#define SW3526_SW_I2C_HALF_PERIOD_US    5
#endif

/*============================================================================
 * 枚举
 *============================================================================*/
enum class e_SW3526_ADC_Lane : uint8_t {
    VIN  = 1,
    VOUT = 2,
    IOUT = 3
};

enum class e_SW3526_FastProt : uint8_t {
    NONE    = 0,
    QC2     = 1,
    QC3     = 2,
    FCP     = 3,
    SCP     = 4,
    PD_FIX  = 5,
    PD_PPS  = 6,
    PE11    = 7,
    PE20    = 8,
    VOOC    = 9,
    SFCP    = 10,
    AFC     = 11,
    UNKNOWN = 12
};

enum class e_SW3526_FastMode : uint8_t {
    FAST_ON  = 0,
    ONLY_PD  = 1,
    NO_FAST  = 2
};

enum class e_SW3526_PD_Version : uint8_t {
    NONE   = 0,
    PD2_0  = 1,
    PD3_0  = 2
};

enum class e_SW3526_NoLoadVolt : uint8_t {
    V5_12 = 0,
    V5_08 = 1,
    V5_04 = 2,
    V5_00 = 3
};

enum class e_SW3526_SCP_MaxPower : uint8_t {
    W20 = 0,
    W40 = 1
};

enum class e_SW3526_SCP_MaxCurrent : uint8_t {
    A2_0 = 0,
    A2_2 = 1,
    A3_5 = 2,
    A4_0 = 3
};

enum class e_SW3526_NonPdMaxVolt : uint8_t {
    SAME_AS_PD = 0,
    V9  = 1,
    V12 = 2,
    V20 = 3
};

enum class e_SW3526_PortType : uint8_t {
    TYPE_C = 0,
    TYPE_A = 1
};

enum class e_SW3526_WireComp : uint8_t {
    MV_120_PER_A = 0,
    MV_60_PER_A  = 1
};

enum class e_SW3526_PDCmd : uint8_t {
    HARD_RESET = 1
};

/*============================================================================
 * 寄存器映射 (SW3526S_REG V1.0.0)
 *============================================================================*/
enum class e_SW3526_Reg : uint8_t {
    CHIP_VER      = 0x01,
    BUCK_VOL_H    = 0x03,
    BUCK_VOL_L    = 0x04,
    CC_LIMIT      = 0x05,
    FAST_STATUS   = 0x06,
    SYS_STATUS    = 0x07,
    CC_CONN       = 0x0A,
    FAULT_STATUS  = 0x0B,
    MAX_POWER_RD  = 0x0D,

    UNLOCK_1      = 0x12,
    BUCK_CTRL     = 0x13,

    VIN_L         = 0x30,
    VOUT_L        = 0x31,
    IOUT_L        = 0x33,
    ADC_SENSE     = 0x3A,
    ADC_DATA_H    = 0x3B,
    ADC_LSB       = 0x3C,

    WATT_CONFIG   = 0x68,
    PD_CMD        = 0x70,
    PD_SRC_CAP    = 0x73,

    NOLOAD_VOLT   = 0xA0,
    CHG_CFG0      = 0xA2,
    CHG_CFG1      = 0xA4,
    FREQ_CONFIG   = 0xA6,
    POWER_CFG     = 0xA7,
    CHG_CFG2      = 0xA8,
    CHG_CFG3      = 0xA9,
    CHG_CFG4      = 0xAA,
    CHG_CFG5      = 0xAB,
    CHG_CFG6      = 0xAC,
    CHG_CFG7      = 0xAD,
    CHG_CFG8      = 0xAE,
    CHG_CFG9      = 0xAF
};

/*============================================================================
 * 状态结构
 *============================================================================*/
struct SW3526_FaultFlags {
    bool output_uv   : 1;
    bool input_uv    : 1;
    bool input_ov    : 1;
    bool die_ot_warn : 1;
    bool die_ot_shdn : 1;
    bool output_sc   : 1;
};

struct SW3526_SysStatus {
    bool port_open : 1;
    bool buck_on   : 1;
};

struct SW3526_Measurement {
    uint16_t vin_mv  = 0;
    uint16_t vout_mv = 0;
    uint16_t iout_ma = 0;
    uint16_t power_mw = 0;
};

/*============================================================================
 * SW3526 驱动
 *============================================================================*/
class SW3526 {
public:
    SW3526() = default;

    /**
     * @param bus   I2C 外设实例 (由工程 lf_config.h 提供, 如 LF_I2C_BUS0 = I2C1)
     * @param addr7 7-bit 从机地址 (SW3526S 为 0x3C)
     * @param name  设备名 (调试用)
     */
    SW3526(i2c_type *bus, uint8_t addr7 = SW3526_I2C_ADDR_7BIT, const char *name = "SW3526")
    {
        init(bus, addr7, name);
    }

    void init(i2c_type *bus, uint8_t addr7 = SW3526_I2C_ADDR_7BIT, const char *name = "SW3526") noexcept
    {
        m_i2c   = bus;
        m_useSw = false;
        m_addr7 = addr7;
        m_name  = (name != nullptr) ? name : "SW3526";
        m_senseMilliOhm = SW3526_SENSE_MILLIOHM;
    }

    /*---------- 软件位敲总线的运行期调节 ----------*/
    /** 改半周期 (µs): 线长/容性大、或从机挑剔时可放慢 (5µs≈100kHz, 20µs≈25kHz)。 */
    void setSwHalfPeriod(uint16_t us) noexcept { m_sw.halfPeriodUs = us; }
    [[nodiscard]] const SwI2cBus &getSwBus() const noexcept { return m_sw; }

    /**
     * @brief 软件位敲 (Bit-Bang) 初始化 —— 用于无硬件 I2C 可用的总线。
     *
     * AT32F421 只有 I2C1 / I2C2；出现第三条总线时 (如 907 的 IIC3:
     * PA7=SCL / PA6=SDA 挂 A1 的 SW3526S) 走 GPIO 位敲。
     * 内部会把两脚配成开漏输出 + 上拉 (swI2cConfigPins)。
     *
     * @param sdaPort/sdaPin  SDA 引脚
     * @param sclPort/sclPin  SCL 引脚
     */
    void initBitbang(gpio_type *sdaPort, uint16_t sdaPin,
                     gpio_type *sclPort, uint16_t sclPin,
                     uint8_t addr7 = SW3526_I2C_ADDR_7BIT, const char *name = "SW3526") noexcept
    {
        m_sw  = SwI2cBus{ sclPort, sclPin, sdaPort, sdaPin, SW3526_SW_I2C_HALF_PERIOD_US };
        m_useSw = true;
        m_i2c   = nullptr;
        m_addr7 = addr7;
        m_name  = (name != nullptr) ? name : "SW3526";
        m_senseMilliOhm = SW3526_SENSE_MILLIOHM;
        swI2cConfigPins(m_sw);
    }

    /*---------- 配置 ----------*/
    void setSenseResistor(uint16_t milliOhm) noexcept { m_senseMilliOhm = milliOhm; }
    [[nodiscard]] uint16_t getSenseResistor() const noexcept { return m_senseMilliOhm; }
    void setTimeout(uint32_t ms) noexcept { m_timeoutMs = ms; }

    /*---------- 寄存器操作 ----------*/
    bool writeReg(e_SW3526_Reg reg, uint8_t value) { return writeReg(static_cast<uint8_t>(reg), value); }
    bool writeReg(uint8_t reg, uint8_t value);
    std::optional<uint8_t> readReg(e_SW3526_Reg reg) { return readReg(static_cast<uint8_t>(reg)); }
    std::optional<uint8_t> readReg(uint8_t reg);

    /*---------- 芯片信息 ----------*/
    uint8_t getChipVersion();
    /** 探测芯片是否在线 (读版本寄存器成功即在线)。 */
    [[nodiscard]] bool probe();

    /*---------- 测量 ----------*/
    uint16_t getVin();          // mV
    uint16_t getVout();         // mV
    uint16_t getIout();         // mA (已按检流电阻缩放)
    SW3526_Measurement getAllMeasurements();
    uint16_t getBuckTargetVoltage();   // mV
    uint16_t getCCLimit();             // mA
    bool setCCLimit(uint16_t limitMa);

    /*---------- 状态 ----------*/
    SW3526_SysStatus getSysStatus();
    uint8_t getCCConnection();
    SW3526_FaultFlags getFaultStatus();
    uint8_t getMaxPowerReadback();

    /*---------- 协议状态 ----------*/
    e_SW3526_FastProt getFastStatus();
    e_SW3526_PD_Version getPDVersion();
    bool isFastCharging();
    bool isVoltageFast();
    static const char *getProtocolName(e_SW3526_FastProt prot);

    /*---------- 功率配置 ----------*/
    uint8_t getWattConfig();
    bool setMaxPower(uint8_t watts);
    bool setPowerSource(bool useRegister);
    bool setPortType(e_SW3526_PortType portType);

    /*---------- Buck 控制 ----------*/
    void unlockRegisters();
    void forceBuckOff1S();
    bool setBuckOffCCDisconnect(bool disconnect);
    bool setWireCompEnable(bool enable);
    bool setBuckFreq(uint16_t freqKhz);
    bool setWireCompSize(e_SW3526_WireComp comp);

    /*---------- 空载电压 ----------*/
    bool setNoLoadVoltage(e_SW3526_NoLoadVolt volt);

    /*---------- 快充协议控制 ----------*/
    const char *setFastMode(e_SW3526_FastMode mode);
    bool setFastChargeEnable(bool enable);
    bool setPDEnable(bool enable);
    bool setPDVoltageEnable(bool voltage9, bool voltage12, bool voltage15, bool voltage20);
    bool setPPSEnable(bool pps0, bool pps1);
    bool setSCPEnable(bool enable);
    bool setSCPMaxPower(e_SW3526_SCP_MaxPower power);
    bool setHVSCPEnable(bool enable);
    bool setPE20HighVoltEnable(bool enable);
    bool setQC2Enable(bool enable);
    bool setQC3Enable(bool enable);
    bool setFCPEnable(bool enable);
    bool setAFCEnable(bool enable);
    bool setPEEnable(bool enable);
    bool setSFCPEnable(bool enable);
    bool setDPDMEnable(bool enable);
    bool setSCPMaxCurrent(e_SW3526_SCP_MaxCurrent current);
    bool setNonPdMaxVoltage(e_SW3526_NonPdMaxVolt volt);
    bool setQCWireCompEnable(bool enable);
    bool setPdoVinLink(bool link);
    bool setPdo5V2ARebroadcast(bool enable);

    /*---------- PD 命令 ----------*/
    bool sendPDCmd(e_SW3526_PDCmd cmd);
    bool sendPDSourceCap();

    /*---------- 双芯片动态功率 ----------*/
    bool setDualChipResetPower(bool resetAll, bool enable);

    /*---------- Apple/Samsung ----------*/
    bool setAppleModeEnable(bool enable);
    bool setNonPdPowerLimit18W(bool limit18W);
    bool setPDResponseTime(bool fast);
    bool setAppleSamsungRefVolt(uint8_t voltage);
    bool setAppleDriveCurrent(uint8_t current);

    /*---------- ADC ----------*/
    void setADCLane(e_SW3526_ADC_Lane lane);

    /*---------- 工具 ----------*/
    [[nodiscard]] const char *getName() const noexcept { return m_name; }
    [[nodiscard]] bool isInitialized() const noexcept { return m_i2c != nullptr || m_useSw; }
    [[nodiscard]] uint8_t getAddress() const noexcept { return m_addr7; }

private:
    i2c_type     *m_i2c  = nullptr;
    SwI2cBus      m_sw{};          /* 软件位敲总线描述 (m_useSw 时有效) */
    bool          m_useSw = false; /* true = 走 i2c_sw, false = 走 i2c_master */
    uint8_t     m_addr7 = SW3526_I2C_ADDR_7BIT;
    const char *m_name  = "SW3526";
    uint16_t    m_senseMilliOhm = SW3526_SENSE_MILLIOHM;
    uint32_t    m_timeoutMs     = SW3526_I2C_TIMEOUT_MS;

    /*---------- 位域辅助 ----------*/
    bool modifyRegBit(e_SW3526_Reg reg, uint8_t bit, bool value);
    bool modifyRegField(e_SW3526_Reg reg, uint8_t shift, uint8_t mask, uint8_t value);
};

/*============================================================================
 * SW3526_Manager — 多实例管理
 *============================================================================*/
class SW3526_Manager {
public:
    static constexpr std::size_t MAX_DEVICES = 4;

    std::int8_t addDevice(SW3526 *device) noexcept
    {
        if (m_count >= MAX_DEVICES || device == nullptr) return -1;
        m_devices[m_count] = device;
        return static_cast<std::int8_t>(m_count++);
    }

    [[nodiscard]] SW3526 *getDevice(std::size_t index) noexcept
    {
        return (index < m_count) ? m_devices[index] : nullptr;
    }

    [[nodiscard]] std::size_t getDeviceCount() const noexcept { return m_count; }

    /** 按名字查找 (大小写敏感)。 */
    [[nodiscard]] SW3526 *findByName(const char *name) noexcept
    {
        if (name == nullptr) return nullptr;
        for (std::size_t i = 0; i < m_count; ++i) {
            const char *n = m_devices[i]->getName();
            const char *a = n;
            const char *b = name;
            while (*a != '\0' && *a == *b) { ++a; ++b; }
            if (*a == *b) return m_devices[i];
        }
        return nullptr;
    }

    void forEach(const std::function<void(SW3526 *, std::size_t)> &callback)
    {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_devices[i] != nullptr) callback(m_devices[i], i);
        }
    }

private:
    std::array<SW3526 *, MAX_DEVICES> m_devices{};
    std::size_t m_count = 0;
};

} // namespace LoveFinderLib

#endif /* SW3526_HPP */
