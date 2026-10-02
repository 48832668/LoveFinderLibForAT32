/**
 * @file    SW2305.hpp
 * @brief   SW2305 USB PD 协议芯片驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  寄存器映射依据: SW2305_REG V1.2 (珠海智融科技 ZHUHAI ISMARTWARE)
 * ============================================================================
 *
 *  关键点
 *  ------
 *  1. 写保护: 写 reg0x14 与 reg0xA0~0xBF 前必须解锁 REG0x12 (0x20→0x40→0x80);
 *     写 reg0x16 前必须解锁 REG0x15 (同样的三写序列)。
 *     → unlockForConfig() / unlockForForceCtrl()
 *  2. ADC 是 12-bit, 但通过 0x3B 选通道后锁存到 0x3C/0x3D。
 *     读一次 0x3B 后 0x3C/0x3D 即被锁存，保证高低位对应。
 *  3. 电流标定: 0x3B 通道 3 给出 adc_ich[11:0], 3.125mA/bit 是 **10mΩ 基准**。
 *     本项目外挂 5mΩ 检流电阻 → 实际电流 = adc * 3.125 * (10 / Rsense_mΩ)。
 *
 * ============================================================================
 *  典型用法 (本项目 CH3)
 * ============================================================================
 *    LoveFinderLib::SW2305 ch3;
 *    ch3.init(LF_I2C_BUS0, 0x28, "CH3");  // 实例来自 lf_config.h (I2C1)
 *    ch3.setSenseResistor(5);            // 5mΩ
 *    if (ch3.probe()) { ... }
 *    uint16_t vbus = ch3.getVbus();      // mV
 *    uint16_t ich  = ch3.getIch();       // mA
 */
#ifndef SW2305_HPP
#define SW2305_HPP

#include "lf_config.h"       /* ★ 工程侧: I2C 实例宏 + lf_get_tick() 时基 */
#include "i2c_master.hpp"    /* 阻塞式 I2C 主机驱动 (I2cStatus / i2cMemRead...) */
#include "i2c_sw.hpp"        /* 软件位敲 I2C (SwI2cBus / swI2cMemRead...): 硬件 I2C 用不了时走它 */

#include <cstdint>
#include <optional>

namespace LoveFinderLib {

/*============================================================================
 * 编译期默认值 (可在 lf_config.h 中覆盖)
 *============================================================================*/
#ifndef SW2305_I2C_ADDR_7BIT
#define SW2305_I2C_ADDR_7BIT        0x28    /* 7-bit 地址 → 8-bit 0x50 */
#endif

#ifndef SW2305_I2C_TIMEOUT_MS
#define SW2305_I2C_TIMEOUT_MS       100
#endif

/** 软件 I2C 半周期延时 (µs) → 标称 100kHz, 可在 lf_config.h 覆盖。 */
#ifndef SW2305_SW_I2C_HALF_PERIOD_US
#define SW2305_SW_I2C_HALF_PERIOD_US    5
#endif

#ifndef SW2305_SENSE_MILLIOHM
#define SW2305_SENSE_MILLIOHM       5       /* 本项目 5mΩ */
#endif

/* ADC 标定基准: 数据手册的 3.125mA/bit 对应 10mΩ 检流电阻 */
#define SW2305_SENSE_REFERENCE_MILLIOHM  10

/*============================================================================
 * 枚举
 *============================================================================*/

/** ADC 通道选择 (写 REG 0x3B, 结果锁存到 0x3C/0x3D)。 */
enum class e_SW2305_ADC_Channel : uint8_t {
    NONE   = 0,
    VIN    = 1,    // adc_vin[11:0],   7.5mV/bit
    VBUS   = 2,    // adc_vbus[11:0],  7.5mV/bit
    ICH    = 3,    // adc_ich[11:0],   3.125mA/bit @10mΩ
    TDIET  = 4     // adc_diet[11:0],  Tdiet = (adc-1848)/6.72 ℃
};

/** 快充协议指示 (REG 0x06 [3:0])。 */
enum class e_SW2305_FastProt : uint8_t {
    NONE    = 0x0,
    QC2     = 0x1,
    QC3     = 0x2,
    FCP     = 0x3,
    SCP     = 0x5,
    PD_FIX  = 0x6,
    PD_PPS  = 0x7,
    PE11    = 0x8,
    PE20    = 0x9,
    VOOC    = 0xA,
    SFCP    = 0xC,
    AFC     = 0xD,
    UNKNOWN = 0xF
};

/** PD 协议版本 (REG 0x06 [5:4])。 */
enum class e_SW2305_PD_Version : uint8_t {
    NONE   = 0,
    PD2_0  = 1,
    PD3_0  = 2
};

/** 线补阻抗 (REG 0xA4 [7:6])。 */
enum class e_SW2305_WireComp : uint8_t {
    OHM_50  = 0,
    OHM_0   = 1,
    OHM_100 = 2,
    OHM_150 = 3
};

/** Die 过温保护门限 (REG 0xAB [3:2])。 */
enum class e_SW2305_OtpThreshold : uint8_t {
    C105 = 0,
    C115 = 1,
    C125 = 2,
    C135 = 3
};

/** 输出电压固定偏移 (REG 0xA3 [7:6])。 */
enum class e_SW2305_VoutOffset : uint8_t {
    MV_200 = 0,
    MV_0   = 1,
    MV_250 = 2,
    MV_100 = 3
};

/** A 口在线判别电流门限 (REG 0xAC [1:0])。 */
enum class e_SW2305_OnlineCurrent : uint8_t {
    MA_200 = 0,
    MA_100 = 1,
    MA_300 = 2,
    MA_400 = 3
};

/** SCP 广播电流 (REG 0xB2 [5:4])。 */
enum class e_SW2305_ScpCurrent : uint8_t {
    A5_0 = 0,
    A4_0 = 1,
    A2_0 = 2
};

/*============================================================================
 * 寄存器映射 (SW2305_REG V1.2)
 *============================================================================*/
enum class e_SW2305_Reg : uint8_t {
    /* --- 只读状态 --- */
    CHIP_VER      = 0x01,  // [1:0] 芯片版本号
    DAC_VOL_H     = 0x03,  // dac_vol[11:4],  设置电压 = dac_vol*10mV
    DAC_VOL_L     = 0x04,  // dac_vol[3:0] 在 [7:4]
    CTRL_ICC      = 0x05,  // [6:0] ctrl_icc, 限流 = 1000mA + ctrl*50mA
    FAST_STATUS   = 0x06,  // 快充指示
    SYS_STATUS0   = 0x07,  // 系统状态 0
    SYS_STATUS1   = 0x0B,  // 系统状态 1
    SYS_STATUS2   = 0x0C,  // 系统状态 2
    SYS_STATUS3   = 0x0D,  // 系统状态 3 (在线)

    /* --- 写保护控制 --- */
    UNLOCK_0      = 0x12,  // 解锁 0x14 / 0xA0~0xBF (0x20→0x40→0x80)
    CONN_CTRL     = 0x14,  // [2] 线补控制, [1] CC un-driving
    UNLOCK_1      = 0x15,  // 解锁 0x16
    FORCE_CTRL    = 0x16,  // 强制控制

    /* --- ADC --- */
    ADC_VIN       = 0x30,  // Vin  高 8bit
    ADC_VBUS      = 0x31,  // Vbus 高 8bit
    ADC_ICH       = 0x33,  // Ich  高 8bit
    ADC_TDIET     = 0x36,  // Die 温度高 8bit
    ADC_CFG       = 0x3B,  // [2:0] ADC 数据选择 (锁存到 0x3C/0x3D)
    ADC_DATA_H    = 0x3C,  // adc_data[11:4]
    ADC_DATA_L    = 0x3D,  // adc_data[3:0] 在 [3:0]

    /* --- OTP 配置 --- */
    FAULT_CFG     = 0xA1,  // [7] Die 过温异常处理使能 (0=使能)
    VOUT_OFFSET   = 0xA3,  // [7:6] 输出电压偏移, [5] Vin>25V 异常处理使能
    WIRE_COMP     = 0xA4,  // [7:6] 线补阻抗
    OVP_CFG       = 0xA5,  // [7] DP, [6] DM, [5] CC1/CC2 过压保护 (0=使能)
    PD_CFG3       = 0xA6,  // [6] PD 5A 是否需 emark
    BRDCST_CURR   = 0xA8,  // [7] PPS3 广播 5A, [6] Type-C 固定广播 1.5A
    PROT_CFG      = 0xAB,  // [3:2] Die 过温保护门限
    ONLINE_CFG    = 0xAC,  // [2] debounce, [1:0] A 口在线门限
    FAST_CFG0     = 0xAD,  // 快充配置 0
    FAST_CFG1     = 0xAE,  // 快充配置 1
    POWER_CFG     = 0xAF,  // [7] 功率配置方式, [6:0] 最大功率 W/bit
    FAST_CFG2     = 0xB0,  // 快充配置 2
    FAST_CFG3     = 0xB1,  // 快充配置 3
    FAST_CFG4     = 0xB2,  // 快充配置 4
    PD_CFG0       = 0xB3,  // PD 配置 0
    PD_CFG1       = 0xB4,  // PD 配置 1
    PD_CFG2       = 0xB5,  // PD 配置 2
    VID0          = 0xB6,  // VID[15:8]
    VID1          = 0xB7,  // VID[7:0]
    XID0          = 0xB8,  // XID[31:24]
    XID1          = 0xB9,  // XID[23:16]
    XID2          = 0xBA,  // XID[15:8]
    XID3          = 0xBB,  // XID[7:0]
    PID0          = 0xBC,  // PID[15:8]
    PID1          = 0xBD,  // PID[7:0]
    SVID0         = 0xBE,  // SVID[15:8]
    SVID1         = 0xBF   // SVID[7:0]
};

/*============================================================================
 * 状态结构
 *============================================================================*/

/** REG 0x07 系统状态 0。 */
struct SW2305_SysStatus0 {
    bool pull_photo_abnormal = false;  // [4] 异常拉光耦
    bool cc_loop_closed      = false;  // [2] CC 环路关闭
    bool cable_comp_active   = false;  // [1] 线补打开
    bool path_open           = false;  // [0] 通路管打开
};

/** REG 0x0B 系统状态 1。 */
struct SW2305_SysStatus1 {
    bool vin_over_25v = false;  // [5]
    bool overcurrent  = false;  // [4] 电流超过 112.5%
    bool die_overheat = false;  // [3]
    bool vin_ov       = false;  // [1] Vin > DAC+20%
    bool vin_uv       = false;  // [0] Vin < 4V
};

/** REG 0x0C 系统状态 2。 */
struct SW2305_SysStatus2 {
    bool cc1_ov    = false;  // [7]
    bool cc2_ov    = false;  // [6]
    bool dp_ov     = false;  // [5]
    bool dm_ov     = false;  // [4]
    bool low_power = false;  // [2] Vin < 3V
    bool ocp       = false;  // [0] 过流保护
};

/** 一次读取得到的完整测量结果。 */
struct SW2305_Measurement {
    uint16_t vin_mv  = 0;
    uint16_t vbus_mv = 0;
    uint16_t ich_ma  = 0;
    float    tdiet_c = 0.0f;
    bool     valid   = false;
};

/*============================================================================
 * SW2305 驱动
 *============================================================================*/
class SW2305 {
public:
    SW2305() = default;

    /**
     * @param bus   I2C 外设实例 (由工程 lf_config.h 提供, 如 LF_I2C_BUS0 = I2C1)
     * @param addr7 7-bit 从机地址 (SW2305 为 0x28)
     * @param name  设备名 (调试用)
     */
    SW2305(i2c_type *bus, uint8_t addr7 = SW2305_I2C_ADDR_7BIT, const char *name = "SW2305")
    {
        init(bus, addr7, name);
    }

    void init(i2c_type *bus, uint8_t addr7 = SW2305_I2C_ADDR_7BIT, const char *name = "SW2305") noexcept
    {
        m_i2c   = bus;
        m_useSw = false;
        m_addr7 = addr7;
        m_name  = (name != nullptr) ? name : "SW2305";
        m_senseMilliOhm = SW2305_SENSE_MILLIOHM;
    }

    /**
     * @brief 软件位敲 (Bit-Bang) 初始化 —— 与 SW3526::initBitbang 同一套。
     *
     * 本板 C2 的 SW2305 走 PA1=SCL / PA0=SDA, 而硬件 I2C2 的复用分配正好相反
     * (PA0=I2C2_SCL / PA1=I2C2_SDA), 硬件外设用不了 → 走 GPIO 位敲。
     * 内部会把两脚配成开漏输出 (swI2cConfigPins)。
     */
    void initBitbang(gpio_type *sdaPort, uint16_t sdaPin,
                     gpio_type *sclPort, uint16_t sclPin,
                     uint8_t addr7 = SW2305_I2C_ADDR_7BIT, const char *name = "SW2305") noexcept
    {
        m_sw    = SwI2cBus{ sclPort, sclPin, sdaPort, sdaPin, SW2305_SW_I2C_HALF_PERIOD_US };
        m_useSw = true;
        m_i2c   = nullptr;
        m_addr7 = addr7;
        m_name  = (name != nullptr) ? name : "SW2305";
        m_senseMilliOhm = SW2305_SENSE_MILLIOHM;
        swI2cConfigPins(m_sw);
    }

    /*---------- 软件位敲总线的运行期调节 ----------*/
    void setSwHalfPeriod(uint16_t us) noexcept { m_sw.halfPeriodUs = us; }
    [[nodiscard]] const SwI2cBus &getSwBus() const noexcept { return m_sw; }

    /*---------- 配置 ----------*/
    void setSenseResistor(uint16_t milliOhm) noexcept { m_senseMilliOhm = milliOhm; }
    [[nodiscard]] uint16_t getSenseResistor() const noexcept { return m_senseMilliOhm; }
    void setTimeout(uint32_t ms) noexcept { m_timeoutMs = ms; }

    /*---------- 寄存器操作 ----------*/
    bool writeReg(e_SW2305_Reg reg, uint8_t value) { return writeReg(static_cast<uint8_t>(reg), value); }
    bool writeReg(uint8_t reg, uint8_t value);
    std::optional<uint8_t> readReg(e_SW2305_Reg reg) { return readReg(static_cast<uint8_t>(reg)); }
    std::optional<uint8_t> readReg(uint8_t reg);

    /*---------- 解锁 (写保护寄存器前置操作) ----------*/
    /** 解锁 REG 0x14 与 REG 0xA0~0xBF (写 REG0x12 = 0x20→0x40→0x80)。 */
    bool unlockForConfig();
    /** 解锁 REG 0x16 (写 REG0x15 = 0x20→0x40→0x80)。 */
    bool unlockForForceCtrl();

    /*---------- 芯片信息 ----------*/
    uint8_t getChipVersion();
    /** 探测芯片是否在线。 */
    [[nodiscard]] bool probe();

    /*---------- ADC 12-bit ----------*/
    /** 选择 ADC 通道并触发锁存。 */
    bool selectADCChannel(e_SW2305_ADC_Channel ch);
    /** 读取锁存的 12-bit ADC 数据 (需先 selectADCChannel)。 */
    uint16_t readADC12();

    /*---------- 测量 (mV / mA / ℃) ----------*/
    uint16_t getVin();      // mV
    uint16_t getVbus();     // mV
    uint16_t getIch();      // mA (已按检流电阻缩放)
    float    getTdiet();    // ℃ (Die 温度)
    SW2305_Measurement getAllMeasurements();
    /** 只读 VBUS + 电流 (ADC_CFG 只写 VBUS/ICH 两个通道, vin/tdiet 字段保持 0)。 */
    SW2305_Measurement getVbusIch();

    /*---------- 设置电压 / 限流 (只读回读) ----------*/
    uint16_t getTargetVoltage();   // mV = dac_vol * 10
    uint16_t getCCLimit();         // mA = 1000 + ctrl_icc * 50
    bool     setCCLimit(uint16_t limitMa);

    /*---------- 快充状态 ----------*/
    e_SW2305_FastProt   getFastProtocol();
    e_SW2305_PD_Version getPDVersion();
    bool isFastCharging();     // [7] 处于快充协议
    bool isVoltageFast();      // [6] 处于快充电压
    static const char *getProtocolName(e_SW2305_FastProt prot);

    /*---------- 系统状态 ----------*/
    SW2305_SysStatus0 getSysStatus0();
    SW2305_SysStatus1 getSysStatus1();
    SW2305_SysStatus2 getSysStatus2();
    bool isOnline();           // REG 0x0D [7]

    /*---------- 强制控制 (REG 0x16, 需 unlockForForceCtrl) ----------*/
    bool forcePathOn();
    bool forcePathOff();
    bool forceDacControl(bool enable);
    bool forceIccControl(bool enable);
    /** 清除 REG 0x16 的全部强制位。 */
    bool clearForceCtrl();

    /*---------- 线补 (REG 0x14, 需 unlockForConfig) ----------*/
    bool setCableCompEnable(bool enable);
    /** Type-C CC un-driving 1s (REG 0x14 [1], 写 1 后自动清零)。 */
    bool triggerCCUnDriving();

    /*---------- OTP 配置 (需 unlockForConfig) ----------*/
    bool setWireComp(e_SW2305_WireComp comp);
    bool setOtpThreshold(e_SW2305_OtpThreshold th);
    bool setVoutOffset(e_SW2305_VoutOffset offset);
    bool setOnlineCurrent(e_SW2305_OnlineCurrent current);
    bool setMaxPowerByRegister(uint8_t watts);

    /*---------- PD 配置 (需 unlockForConfig) ----------*/
    bool setPDEnable(bool enable);
    bool setPDVoltageEnable(bool v9, bool v12, bool v15, bool v20);
    bool setPPSEnable(bool pps0, bool pps1, bool pps2, bool pps3);
    bool setFastChargeEnable(bool enable);
    bool setQC2Enable(bool enable);
    bool setQC3Enable(bool enable);
    bool setSCPEnable(bool hv, bool lv);
    bool setFCPEnable(bool enable);
    bool setAFCEnable(bool enable);
    bool setPEEnable(bool enable);
    bool setSFCPEnable(bool enable);
    bool setBC12Enable(bool enable);

    /*---------- 工具 ----------*/
    [[nodiscard]] const char *getName() const noexcept { return m_name; }
    [[nodiscard]] bool isInitialized() const noexcept { return m_i2c != nullptr || m_useSw; }
    [[nodiscard]] uint8_t getAddress() const noexcept { return m_addr7; }

private:
    i2c_type   *m_i2c  = nullptr;
    SwI2cBus    m_sw{};            /* 软件位敲总线描述 (m_useSw 时有效) */
    bool        m_useSw = false;   /* true = 走 i2c_sw, false = 走 i2c_master */
    uint8_t     m_addr7 = SW2305_I2C_ADDR_7BIT;
    const char *m_name  = "SW2305";
    uint16_t    m_senseMilliOhm = SW2305_SENSE_MILLIOHM;
    uint32_t    m_timeoutMs     = SW2305_I2C_TIMEOUT_MS;

    bool modifyRegBit(e_SW2305_Reg reg, uint8_t bit, bool value);
    bool modifyRegField(e_SW2305_Reg reg, uint8_t shift, uint8_t mask, uint8_t value);
    /** 写保护寄存器的三写解锁序列。 */
    bool unlockSequence(e_SW2305_Reg unlockReg);
};

} // namespace LoveFinderLib

#endif /* SW2305_HPP */
