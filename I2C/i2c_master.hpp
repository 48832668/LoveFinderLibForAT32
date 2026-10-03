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

/*============================================================================
 * I2cBus —— 把外设句柄 + 超时绑成一个对象, 便于直接读写
 *
 * 上面那些自由函数每次都要把 `i2c_type*` 和 `timeoutMs` 再传一遍; 本类
 * 只是它们的【便利外壳】(纯转发, 无状态机、无后端抽象), 给两种场景用:
 *   1. 应用/调试时飞线加了个 I2C 器件, 想在主循环里直接戳两下;
 *   2. 写一个还不值得单独建驱动的临时操作。
 * 产品代码里的芯片驱动仍应走设备语义 (I2cDevice / SW3538 等), 见 AGENTS §3。
 *
 * 工程侧定义一次 (可与 lf_config.h 的宏配合):
 *    inline LoveFinderLib::I2cBus g_i2c1{ LF_I2C_BUS0 };   // I2C1
 *    inline LoveFinderLib::I2cBus g_i2c2{ LF_I2C_BUS1 };   // I2C2
 *
 * 主循环里随便用:
 *    g_i2c1.writeReg8(0x50, 0x14, 0x14);             // 设备, 寄存器, 值
 *    if (auto v = g_i2c1.readReg8(0x50, 0x14)) { }   // 读一个寄存器
 *    uint8_t id[3];
 *    g_i2c1.readRegs(0x50, 0x00, id, 3);             // 连读多字节
 *    bool there = g_i2c1.ping(0x50);                 // 探测地址在不在
 *============================================================================*/
class I2cBus {
public:
    I2cBus() = default;

    explicit I2cBus(i2c_type *i2c, uint32_t timeoutMs = 100) noexcept
    {
        init(i2c, timeoutMs);
    }

    void init(i2c_type *i2c, uint32_t timeoutMs = 100) noexcept
    {
        m_i2c       = i2c;
        m_timeoutMs = timeoutMs;
    }

    [[nodiscard]] i2c_type *handle() const noexcept { return m_i2c; }
    [[nodiscard]] bool      valid()  const noexcept { return m_i2c != nullptr; }

    void                   setTimeout(uint32_t ms) noexcept { m_timeoutMs = ms; }
    [[nodiscard]] uint32_t timeout() const noexcept { return m_timeoutMs; }

    /*---- 单寄存器 (最常用) ----*/
    /** 写一个 8-bit 寄存器: START + addr(W) + reg + value + STOP */
    [[nodiscard]] bool writeReg8(uint8_t addr7, uint8_t reg, uint8_t value) noexcept
    {
        return i2cWriteReg8(m_i2c, addr7, reg, value, m_timeoutMs);
    }

    /** 读一个 8-bit 寄存器; 失败 (NACK / 超时 / 总线错误) 返回 nullopt。 */
    [[nodiscard]] std::optional<uint8_t> readReg8(uint8_t addr7, uint8_t reg) noexcept
    {
        return i2cReadReg8(m_i2c, addr7, reg, m_timeoutMs);
    }

    /*---- 多字节寄存器 ----*/
    I2cStatus writeRegs(uint8_t addr7, uint8_t reg,
                        const uint8_t *data, uint16_t len) noexcept
    {
        return i2cMemWrite(m_i2c, addr7, reg, data, len, m_timeoutMs);
    }

    I2cStatus readRegs(uint8_t addr7, uint8_t reg,
                       uint8_t *data, uint16_t len) noexcept
    {
        return i2cMemRead(m_i2c, addr7, reg, data, len, m_timeoutMs);
    }

    /*---- 裸收发 (不带寄存器地址) ----*/
    I2cStatus write(uint8_t addr7, const uint8_t *data, uint16_t len) noexcept
    {
        return i2cMasterWrite(m_i2c, addr7, data, len, m_timeoutMs);
    }

    I2cStatus read(uint8_t addr7, uint8_t *data, uint16_t len) noexcept
    {
        return i2cMasterRead(m_i2c, addr7, data, len, m_timeoutMs);
    }

    /*---- 诊断 ----*/
    /** 地址探测: 只发地址看从机 ACK 不 ACK (零长度写)。总线扫描用。 */
    [[nodiscard]] bool ping(uint8_t addr7) noexcept
    {
        return i2cMasterWrite(m_i2c, addr7, nullptr, 0u, m_timeoutMs) == I2cStatus::OK;
    }

    /** 总线恢复: 复位被从机拉死的 I2C 外设。 */
    void recover() noexcept { i2cMasterRecover(m_i2c); }

private:
    i2c_type *m_i2c       = nullptr;
    uint32_t  m_timeoutMs = 100;
};

} // namespace LoveFinderLib

#endif /* LOVE_FINDER_LIB_I2C_MASTER_HPP */
