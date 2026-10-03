/**
 * @file    SW3538.hpp
 * @brief   SW3538 多口快充控制器驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计要点 (与库内 SW3526 / SW2305 同一套模式: 直绑工程头, 无平台抽象层)
 * ============================================================================
 *  - 本驱动【不含 I2C 实现】, init 时收一个 `i2c_type *` 外设实例
 *    (工程 lf_config.h 提供的实例宏, 如 LF_I2C_BUS0 = I2C1),
 *    收发走库内 I2C/i2c_master 阻塞驱动; 地址一律 7-bit 形式。
 *  - 寄存器映射基于 SW3538_寄存器列表 Release_RG108_3_v1.2 (2022-08)。
 *  - 读为主: 本轮 (LoveFinder208 双 140W) 只用它在线探测 / 读电压电流功率
 *    与快充状态; 寄存器写只提供 ADC 采样强制打开这一组 (测量必需)。
 *
 * ============================================================================
 *  关键时序 (手册原文)
 * ============================================================================
 *  - 【写普通寄存器】先写 Reg0x10 = 0x20 → 0x40 → 0x80, 再写目标寄存器;
 *    操作 Reg0x100~Reg0x14F 时序不同 (先写 Reg0x10=0x81), 本驱动不涉及。
 *  - 【写 Reg0x16~Reg0x19】还要先写 Reg0x15 = 0x20 → 0x40 → 0x80
 *    (强制操作使能), 然后才写目标寄存器。writeRegForce() 负责这条时序。
 *  - 【读 ADC】写 Reg0x40 选通道 → 数据锁存到 Reg0x41 / Reg0x42:
 *        value = (Reg0x42[6:0] << 8) | Reg0x41        (12bit; 单位转换电压 15bit)
 *    通道: 1=通路1电流 2.5mA/bit@5mΩ   2=通路2电流
 *          5=输出电压 6mV/bit           6=输入电压 10mV/bit (默认关闭!)
 *          7=NTC 电压 1.2mV/bit         11=单位转换后输出电压 1mV/bit
 *    VIN ADC 默认关闭 → 须 Reg0x18[6]=1 (enableAdcSampling) 才能读到输入电压。
 *
 * ============================================================================
 *  通路使能 (init 形参) —— 这颗芯片实际接了哪几个口
 * ============================================================================
 *  SW3538 是 A+C 双口芯片:
 *    通路1 = Type-C 口   通路2 = Type-A 口
 *  init 时按板子实际情况声明, 没接的口 readMeasurement() 不读它的电流、
 *  状态位强制 false (硬件没接, 寄存器读回来的位没有意义, 免得 UI 误报
 *  "通路2有设备"之类)。POT1 (LoveFinder208) 每颗只用通路1:
 *      ch1.init(LF_I2C_BUS0, 0x3C, "IIC1", e_SW3538_Port::PATH1);
 *
 *  地址: 目前 SW3538 只有 7-bit 0x3C 一个, 所以一条 I2C 总线上只能挂一颗;
 *  多颗必须分总线 (208 用 I2C1 / I2C2 各一颗)。厂家若将来放出更多地址,
 *  换 init 的 addr7 即可 —— 每颗本来就是独立实例, 驱动不用改。
 *
 * ============================================================================
 *  VBUS vs VOUT —— 屏幕上要显示的是 VBUS
 * ============================================================================
 *  VOUT = BUCK 输出电压 (ADC 通道 11, 1mV/bit; 空载约 5.1V);
 *  VBUS = VOUT 经 SW3538 控制的 NMOS 开关之后、Type-C 口上的电压。
 *  按 PD 规范, 没有 SINK 接入时不许 VBUS 上电 → 空载时 VBUS = 0V 而 VOUT 仍 5.1V。
 *
 *  手册 ADC 通道表 (RG108_3_v1.2) 里【没有独立的 VBUS 通道】,
 *  而 NMOS 之后的电压物理上就是「导通 ≈ VOUT(差 mV 级) / 关断 = 0」,
 *  所以 vbus_mv 用【通路导通状态】门控 VOUT 得到 (readMeasurement 内算)。
 *  ★ 上板若发现恒 5.1V 或恒 0V, 说明 Reg0x0A[0] 的语义与预期不符,
 *    把判据换成 path1Online (Reg0x0D[1] 有设备) 或 buckOn && path1On 即可,
 *    见 SW3538.cpp 中 VBUS 段的注释。
 *
 * ============================================================================
 *  典型用法
 * ============================================================================
 *    LoveFinderLib::SW3538 ch1;
 *    ch1.init(LF_I2C_BUS0, 0x3C, "IIC1", LoveFinderLib::e_SW3538_Port::PATH1);
 *    if (ch1.probe()) {                     // 读 Reg0x00 成功 = 在线
 *        ch1.enableAdcSampling();           // 打开 VIN/VOUT/通路1电流 ADC
 *        SW3538_Measurement m;
 *        if (ch1.readMeasurement(m)) { ... m.vbus_mv / m.iout_ma / m.vin_mv ... }
 *    }
 */
#ifndef SW3538_HPP
#define SW3538_HPP

#include "lf_config.h"       /* ★ 工程侧: I2C 实例宏 + lf_get_tick() 时基 */
#include "i2c_master.hpp"    /* 阻塞式 I2C 主机驱动 (I2cStatus / i2cMemRead...) */

#include <cstdint>
#include <cstddef>
#include <optional>

namespace LoveFinderLib {

/*============================================================================
 * 编译期默认值 (可在 lf_config.h 中覆盖)
 *============================================================================*/
#ifndef SW3538_I2C_ADDR_7BIT
#define SW3538_I2C_ADDR_7BIT    0x3C    /* 7-bit 地址 (与 SW3526S 相同) */
#endif

#ifndef SW3538_I2C_TIMEOUT_MS
#define SW3538_I2C_TIMEOUT_MS   100
#endif

#ifndef SW3538_SENSE_MILLIOHM
#define SW3538_SENSE_MILLIOHM   5       /* 默认检流电阻 5mΩ (手册换算基准) */
#endif

/*============================================================================
 * 枚举
 *============================================================================*/
/**
 * @brief 通路使能 —— 这颗芯片实际接了哪几个口 (init 的形参)。
 *
 * 可用 `|` 组合: `e_SW3538_Port::PATH1 | e_SW3538_Port::PATH2` = BOTH。
 */
enum class e_SW3538_Port : uint8_t {
    NONE  = 0x00,   ///< 一个口都没接 (只当状态/版本读)
    PATH1 = 0x01,   ///< 通路1 = Type-C 口
    PATH2 = 0x02,   ///< 通路2 = Type-A 口
    BOTH  = 0x03    ///< 两个口都接 (SW3538 的默认形态)
};

constexpr e_SW3538_Port operator|(e_SW3538_Port a, e_SW3538_Port b) noexcept
{
    return static_cast<e_SW3538_Port>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
constexpr e_SW3538_Port operator&(e_SW3538_Port a, e_SW3538_Port b) noexcept
{
    return static_cast<e_SW3538_Port>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

/** Reg0x40 的 ADC 通道选择 (写后数据锁存到 Reg0x41/0x42)。 */
enum class e_SW3538_AdcCh : uint8_t {
    IOUT_PATH1 = 1,     ///< 通路1电流, 2.5mA/bit @5mΩ 检流
    IOUT_PATH2 = 2,     ///< 通路2电流, 2.5mA/bit @5mΩ 检流
    VOUT_RAW   = 5,     ///< 输出电压, 6mV/bit
    VIN        = 6,     ///< 输入电压, 10mV/bit (默认关闭, 需 Reg0x18[6])
    NTC        = 7,     ///< NTC 电压, 1.2mV/bit
    VOUT_CONV  = 11     ///< 单位转换后输出电压, 1mV/bit (15bit, 精度最高)
};

/** Reg0x09[3:0] 快充协议指示。 */
enum class e_SW3538_FastProt : uint8_t {
    NONE    = 0,
    QC2     = 1,
    QC3     = 2,
    QC3P    = 3,        ///< QC3+
    FCP     = 4,
    SCP     = 5,
    PD_FIX  = 6,
    PD_PPS  = 7,
    PE11    = 8,
    PE20    = 9,
    VOOC10  = 10,
    VOOC40  = 11,
    FAST    = 12,       ///< 原枚举缺号; 手册未列 (待核对), 显示 "FAST"
    SFCP    = 13,
    AFC     = 14,
    TFCP    = 15,
    UNKNOWN = 255
};

/** 常用寄存器 (本驱动用到的子集, 手册 SW3538_寄存器列表 RG108_3_v1.2)。 */
enum class e_SW3538_Reg : uint8_t {
    VER         = 0x00,   ///< 版本号 [1:0] (R)
    MAX_POWER   = 0x02,   ///< 系统最大功率指示, 1W/bit (R)
    FAST_STATUS = 0x09,   ///< 快充指示 (R)
    SYS_STATUS  = 0x0A,   ///< 系统状态: [2]BUCK [1]通路2 [0]通路1 (R)
    DEV_ONLINE  = 0x0D,   ///< 设备在线: [1]通路1 [0]通路2 (R)
    I2C_EN      = 0x10,   ///< I2C 写使能 (解锁序列 0x20/0x40/0x80)
    FORCE_EN    = 0x15,   ///< 强制操作使能 (写 Reg0x16~0x19 前的解锁序列)
    FORCE_ADC   = 0x18,   ///< 强制打开 ADC 采样 (bit7 VOUT / bit6 VIN / bit5 通路1电流)
    ADC_CFG     = 0x40,   ///< ADC 通道选择 (写后锁存)
    ADC_DATA_L  = 0x41,   ///< ADC 数据低 8 位
    ADC_DATA_H  = 0x42    ///< ADC 数据高 7 位
};

/*============================================================================
 * 状态结构
 *============================================================================*/
/** 一次 readMeasurement 的结果 (各 xxxValid 表示对应 ADC 这次读成功)。 */
struct SW3538_Measurement {
    bool    ok = false;          ///< 状态寄存器读取成功 (芯片在通讯)
    uint16_t vin_mv   = 0;       ///< 输入电压 (mV)
    uint16_t vout_mv  = 0;       ///< BUCK 输出电压 VOUT (mV, 单位转换通道)
    uint16_t vbus_mv  = 0;       ///< 口上的 VBUS (mV) = 通路导通时的 VOUT, 关断时 0
    uint16_t iout_ma  = 0;       ///< 主通路输出电流 (mA, 已按检流电阻缩放)
    uint32_t power_mw = 0;       ///< 主通路输出功率 (mW) = vout * iout / 1000

    bool fast     = false;       ///< Reg0x09[7] 或 [6]: 正处于快充
    uint8_t fastProt = 0;        ///< Reg0x09[3:0] 协议号

    bool buckOn       = false;   ///< Reg0x0A[2] BUCK 打开
    bool path1On      = false;   ///< Reg0x0A[0] 通路1 打开 (NMOS 导通)
    bool path2On      = false;   ///< Reg0x0A[1] 通路2 打开
    bool path1Online  = false;   ///< Reg0x0D[1] 通路1 有设备
    bool path2Online  = false;   ///< Reg0x0D[0] 通路2 有设备

    bool vinValid   = false;     ///< vin_mv 这次读到
    bool voutValid  = false;
    bool vbusValid  = false;     ///< vbus_mv 这次算出 (通路使能且状态读到)
    bool ioutValid  = false;
};

/*============================================================================
 * SW3538 驱动
 *============================================================================*/
class SW3538 {
public:
    SW3538() = default;

    /**
     * @param bus   I2C 外设实例 (工程 lf_config.h 提供, 如 LF_I2C_BUS0 = I2C1)
     * @param addr7 7-bit 从机地址 (SW3538 现为 0x3C; 将来厂家放出新地址只改这里)
     * @param name  设备名 (调试用)
     * @param ports 这颗芯片实际接了哪些口 —— 没接的口不读电流、状态位恒 false。
     *              POT1 每颗只用 Type-C 口: `e_SW3538_Port::PATH1`
     *              (缺省 BOTH, 老调用不用改)
     */
    void init(i2c_type *bus, uint8_t addr7 = SW3538_I2C_ADDR_7BIT,
              const char *name = "SW3538",
              e_SW3538_Port ports = e_SW3538_Port::BOTH) noexcept
    {
        m_i2c   = bus;
        m_addr7 = addr7;
        m_name  = (name != nullptr) ? name : "SW3538";
        m_ports = ports;
        m_senseMilliOhm = SW3538_SENSE_MILLIOHM;
        m_timeoutMs     = SW3538_I2C_TIMEOUT_MS;
        m_online        = false;
    }

    /*---------- 通路使能 ----------*/
    /** 这颗芯片使能了哪些通路 (init 时给的)。 */
    [[nodiscard]] e_SW3538_Port getPorts() const noexcept { return m_ports; }
    /** 某个通路是否使能, 例: hasPort(e_SW3538_Port::PATH1)。 */
    [[nodiscard]] bool hasPort(e_SW3538_Port p) const noexcept
    {
        if (p == e_SW3538_Port::NONE) return false;
        return (m_ports & p) == p;
    }

    /*---------- 配置 ----------*/
    /** 检流电阻 (mΩ) —— 电流换算基准, 手册 2.5mA/bit 是 @5mΩ。 */
    void setSenseResistor(uint16_t milliOhm) noexcept
    {
        m_senseMilliOhm = (milliOhm == 0u) ? 5u : milliOhm;
    }
    [[nodiscard]] uint16_t getSenseResistor() const noexcept { return m_senseMilliOhm; }
    void setTimeout(uint32_t ms) noexcept { m_timeoutMs = ms; }

    /*---------- 寄存器读写 ----------*/
    /** 读寄存器 (带 Reg0x10 解锁的写, 见文件头时序说明)。失败返回 nullopt。 */
    std::optional<uint8_t> readReg(uint8_t reg);
    std::optional<uint8_t> readReg(e_SW3538_Reg reg) { return readReg(static_cast<uint8_t>(reg)); }
    /** 连续读 len 字节 (len <= 16)。 */
    bool readRegs(uint8_t reg, uint8_t *buf, uint16_t len);
    /** 写寄存器: Reg0x10 解锁序列 (0x20/0x40/0x80) → 写目标。 */
    bool writeReg(uint8_t reg, uint8_t value);
    /** 写 Reg0x16~Reg0x19: 额外走 Reg0x15 强制操作解锁序列。 */
    bool writeRegForce(uint8_t reg, uint8_t value);

    /*---------- 探测 ----------*/
    /** 读版本寄存器 Reg0x00 —— 能读到就在线 (返回 [1:0] 版本号)。 */
    std::optional<uint8_t> getVersion();
    /** 在线探测: Reg0x00 可读 → true (结果缓存在 isOnline() / lastStatus())。 */
    [[nodiscard]] bool probe();
    [[nodiscard]] bool isOnline() const noexcept { return m_online; }
    /** 最近一次 I2C 操作的原始结果 (调试页显示用)。 */
    [[nodiscard]] I2cStatus lastStatus() const noexcept { return m_last; }

    /*---------- 测量 ----------*/
    /** 打开 VIN / VOUT / 通路1电流 ADC 采样 (Reg0x15 序列 + Reg0x18 = 0xE0)。 */
    bool enableAdcSampling();
    /** 选通道 + 读一次 ADC 原始值 (12bit, 单位转换通道 15bit)。 */
    bool readAdc(e_SW3538_AdcCh ch, uint32_t &raw);
    /** 状态块 + 三次 ADC 的组合读取 (UI 一次刷新只调它)。 */
    bool readMeasurement(SW3538_Measurement &m);

    /*---------- 工具 ----------*/
    /** 系统最大功率 (Reg0x02, 1W/bit; 读失败 = 0)。 */
    uint8_t getMaxPowerWatt();
    /** 协议短名 (≤4 字, 直接喂 UI 标签; 无协议返回 "" )。 */
    static const char *getProtocolName(e_SW3538_FastProt prot);

    [[nodiscard]] const char *getName() const noexcept { return m_name; }
    [[nodiscard]] bool isInitialized() const noexcept { return m_i2c != nullptr; }
    [[nodiscard]] uint8_t getAddress() const noexcept { return m_addr7; }

private:
    bool maybeRecover() noexcept;   /* TIMEOUT / BUS_ERROR → 复位 I2C 外设 */
    bool armWriteEnable();          /* Reg0x10 三写序列 */

    i2c_type   *m_i2c  = nullptr;
    uint8_t     m_addr7 = SW3538_I2C_ADDR_7BIT;
    const char *m_name  = "SW3538";
    e_SW3538_Port m_ports = e_SW3538_Port::BOTH;  ///< 使能的通路 (init 形参)
    uint16_t    m_senseMilliOhm = SW3538_SENSE_MILLIOHM;
    uint32_t    m_timeoutMs     = SW3538_I2C_TIMEOUT_MS;
    I2cStatus   m_last          = I2cStatus::PARAM_ERROR;
    bool        m_online        = false;
};

} // namespace LoveFinderLib

#endif /* SW3538_HPP */
