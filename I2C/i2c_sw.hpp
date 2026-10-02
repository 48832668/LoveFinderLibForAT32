/**
 * @file    i2c_sw.hpp
 * @brief   AT32 软件位敲 (Bit-Bang) I2C 主机 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  为什么需要它
 * ============================================================================
 *  AT32F421 只有 I2C1 / I2C2 两路硬件 I2C。当工程出现第三条 I2C 总线时
 *  (如 907 的 IIC3: PA7=SCL / PA6=SDA)，只能用 GPIO 位敲。
 *  与 i2c_master 的差别仅在「总线如何驱动」，时序语义完全一致:
 *    - 写: START + addr(W) + data[] + STOP
 *    - 读寄存器: START + addr(W) + reg + repeated START + addr(R) + data[] + STOP
 *    - 地址一律 7-bit 形式，驱动内部附加读写位
 *
 * ============================================================================
 *  硬件前提 (由工程侧 WorkBench 配置)
 * ============================================================================
 *  - SCL / SDA 配成 **开漏输出** (GPIO_OUTPUT_OPEN_DRAIN)，外部上拉
 *  - 读 SDA 时仍保持输出 1 (开漏释放)，再读 IDR —— 无需切换输入模式
 *  - 半周期延时用 WorkBench 的 wk_delay_us()，故位速率 ≈ 1 / (2 * halfPeriodUs)
 *
 * ============================================================================
 *  用法
 * ============================================================================
 *    using namespace LoveFinderLib;
 *    SwI2cBus bus{ IIC3S_SCL_GPIO_PORT, IIC3S_SCL_PIN,
 *                 IIC3S_SDA_GPIO_PORT, IIC3S_SDA_PIN };
 *    swI2cMemWrite(bus, 0x3C, 0xA0, &val, 1, 100);
 *    auto v = swI2cMemRead(bus, 0x3C, 0x00, 1, 100);
 *
 *  或经 SW3526::initBitbang() 一步到位 (驱动内部持有该总线描述)。
 */
#ifndef LOVE_FINDER_LIB_I2C_SW_HPP
#define LOVE_FINDER_LIB_I2C_SW_HPP

#include "lf_config.h"      /* gpio_type (AT32 BSP) + lf_get_tick() 声明 */
#include "i2c_master.hpp"   /* I2cStatus —— 与硬件 I2C 共用同一套结果码 */

#include <cstdint>
#include <optional>

namespace LoveFinderLib {

/*============================================================================
 * 总线描述 —— 纯数据，可放进驱动对象
 *============================================================================*/
struct SwI2cBus {
    gpio_type *sclPort = nullptr;
    uint16_t   sclPin  = 0;
    gpio_type *sdaPort = nullptr;
    uint16_t   sdaPin  = 0;
    /**
     * 半周期延时 (µs)。5µs → 标称 100kHz (含代码开销实际略低)。
     * 从机若要求更慢 (长线 / 强上拉)，调大即可。
     */
    uint16_t   halfPeriodUs = 5;

    [[nodiscard]] bool valid() const noexcept
    {
        return sclPort != nullptr && sdaPort != nullptr && sclPin != 0u && sdaPin != 0u;
    }
};

/** 便捷构造: SwI2cBus bus = swI2cMakeBus(SCL_PORT, SCL_PIN, SDA_PORT, SDA_PIN); */
[[nodiscard]] inline SwI2cBus swI2cMakeBus(gpio_type *sclPort, uint16_t sclPin,
                                           gpio_type *sdaPort, uint16_t sdaPin,
                                           uint16_t halfPeriodUs = 5) noexcept
{
    return SwI2cBus{ sclPort, sclPin, sdaPort, sdaPin, halfPeriodUs };
}

/*============================================================================
 * 裸收发
 *============================================================================*/
/** 裸写: START + addr(W) + data[] + STOP */
I2cStatus swI2cMasterWrite(const SwI2cBus &bus, uint8_t addr7, const uint8_t *data,
                           uint16_t len, uint32_t timeoutMs) noexcept;

/** 裸读: START + addr(R) + data[] + STOP */
I2cStatus swI2cMasterRead(const SwI2cBus &bus, uint8_t addr7, uint8_t *data,
                          uint16_t len, uint32_t timeoutMs) noexcept;

/*============================================================================
 * 寄存器访问
 *============================================================================*/
/** 写寄存器: START + addr(W) + reg + data[] + STOP */
I2cStatus swI2cMemWrite(const SwI2cBus &bus, uint8_t addr7, uint8_t reg,
                        const uint8_t *data, uint16_t len, uint32_t timeoutMs) noexcept;

/** 读寄存器: START + addr(W) + reg + repeated START + addr(R) + data[] + STOP */
I2cStatus swI2cMemRead(const SwI2cBus &bus, uint8_t addr7, uint8_t reg, uint8_t *data,
                       uint16_t len, uint32_t timeoutMs) noexcept;

/*============================================================================
 * 便捷封装
 *============================================================================*/
[[nodiscard]] bool swI2cWriteReg8(const SwI2cBus &bus, uint8_t addr7, uint8_t reg,
                                  uint8_t value, uint32_t timeoutMs) noexcept;

[[nodiscard]] std::optional<uint8_t> swI2cReadReg8(const SwI2cBus &bus, uint8_t addr7,
                                                   uint8_t reg,
                                                   uint32_t timeoutMs) noexcept;

/**
 * @brief 引脚初始化: SCL/SDA 配成开漏输出 + 上拉, 并置空闲态 (两线释放)。
 *
 * 幂等, 与 WorkBench 生成的 GPIO 配置叠加无冲突 (参数一致时结果相同)。
 * SW3526::initBitbang() 会自动调用它 —— 仿 BUTTON::configureExti 的模式。
 */
void swI2cConfigPins(const SwI2cBus &bus) noexcept;

/**
 * @brief 总线恢复: 从机把 SDA 拉死时，敲最多 9 个 SCL 脉冲 + STOP 复位总线。
 *
 * 硬件 I2C 的 i2cMasterRecover() 只能复位外设，敲脉冲必须在 GPIO 级做 ——
 * 软件总线天然具备该能力，故本函数提供完整恢复。
 */
void swI2cRecover(const SwI2cBus &bus) noexcept;

} // namespace LoveFinderLib

#endif /* LOVE_FINDER_LIB_I2C_SW_HPP */
