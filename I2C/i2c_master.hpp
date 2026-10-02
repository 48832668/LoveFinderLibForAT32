/**
 * @file    i2c_master.hpp
 * @brief   AT32 阻塞式硬件 I2C 主机驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计 (仿 LoveFinderLibForPY32_LL: 驱动直绑工程头 + 厂商 BSP, 无抽象层)
 * ============================================================================
 *  - 这是【具体驱动】而非抽象接口: 所有函数直接收 `i2c_type *` 外设实例,
 *    实例由【工程侧】lf_config.h 定义 (如 LF_I2C_BUS0 = I2C1),
 *    调用方在 init 时传入 —— 与 PY32 库直调 LL_* 同一模式。
 *  - 收发全程轮询标志位 + 工程时基 lf_get_tick() 超时
 *    (lf_get_tick 声明于工程 lf_config.h, 实现于工程 lf_irq.cpp —— 仿 BSP_GetTick)。
 *  - 地址一律 7-bit 形式 (SW3526 = 0x3C, SW2305 = 0x28), 驱动内部附加读写位。
 *
 * ============================================================================
 *  工程侧需要提供 (lf_config.h)
 * ============================================================================
 *    uint32_t lf_get_tick(void);    // SysTick 1ms 递增的时基
 *
 * ============================================================================
 *  用法
 * ============================================================================
 *    using namespace LoveFinderLib;
 *    i2cMemWrite(I2C1, 0x3C, reg, &value, 1, 100);          // 写寄存器
 *    std::optional<uint8_t> v = i2cMemRead(I2C1, 0x3C, reg, 100);
 *
 * ============================================================================
 *  寄存器读时序
 * ============================================================================
 *    memRead = START + addr(W) + reg + repeated START + addr(R) + data[] + STOP
 *    (不发中间 STOP —— I2C 规范与快充芯片手册要求的时序)
 */
#ifndef LOVE_FINDER_LIB_I2C_MASTER_HPP
#define LOVE_FINDER_LIB_I2C_MASTER_HPP

#include "lf_config.h"      /* i2c_type (AT32 BSP) + lf_get_tick() 声明 */

#include <cstdint>
#include <optional>

namespace LoveFinderLib {

/*============================================================================
 * 结果码
 *============================================================================*/
enum class I2cStatus : uint8_t {
    OK = 0,
    TIMEOUT,      ///< 等待标志超时
    NACK,         ///< 从机未应答 (地址或数据)
    BUS_ERROR,    ///< 总线错误 / 仲裁丢失
    PARAM_ERROR   ///< 参数非法
};

[[nodiscard]] const char *i2cStatusName(I2cStatus s) noexcept;

/*============================================================================
 * 裸收发
 *============================================================================*/
/** 裸写: START + addr(W) + data[] + STOP */
I2cStatus i2cMasterWrite(i2c_type *i2c, uint8_t addr7, const uint8_t *data,
                         uint16_t len, uint32_t timeoutMs) noexcept;

/** 裸读: START + addr(R) + data[] + STOP */
I2cStatus i2cMasterRead(i2c_type *i2c, uint8_t addr7, uint8_t *data,
                        uint16_t len, uint32_t timeoutMs) noexcept;

/*============================================================================
 * 寄存器访问
 *============================================================================*/
/** 写寄存器: START + addr(W) + reg + data[] + STOP (len <= 16) */
I2cStatus i2cMemWrite(i2c_type *i2c, uint8_t addr7, uint8_t reg,
                      const uint8_t *data, uint16_t len, uint32_t timeoutMs) noexcept;

/** 读寄存器: START + addr(W) + reg + repeated START + addr(R) + data[] + STOP */
I2cStatus i2cMemRead(i2c_type *i2c, uint8_t addr7, uint8_t reg, uint8_t *data,
                     uint16_t len, uint32_t timeoutMs) noexcept;

/*============================================================================
 * 便捷封装
 *============================================================================*/
[[nodiscard]] bool i2cWriteReg8(i2c_type *i2c, uint8_t addr7, uint8_t reg,
                                 uint8_t value, uint32_t timeoutMs) noexcept;

[[nodiscard]] std::optional<uint8_t> i2cReadReg8(i2c_type *i2c, uint8_t addr7,
                                                 uint8_t reg,
                                                 uint32_t timeoutMs) noexcept;

/** 总线恢复: 复位被从机拉死的 I2C 外设 (GPIO 级 SCL 敲脉冲由应用层自行实现)。 */
void i2cMasterRecover(i2c_type *i2c) noexcept;

} // namespace LoveFinderLib

#endif /* LOVE_FINDER_LIB_I2C_MASTER_HPP */
