/**
 * @file    i2c_device.hpp
 * @brief   I2C 从设备的「寄存器读写助手」— C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  它解决什么
 * ============================================================================
 *  一个 I2C 芯片驱动的本质 = 「若干寄存器的读写集合」。但每读写一个寄存器都要
 *  传 bus + 7-bit 地址 + 超时三样东西, 样板重复。本类把这三样绑成一个对象,
 *  驱动【组合】它, 而不是继承它:
 *
 *    class MyChip {
 *        I2cDevice m_dev;                     // 组合, 不是继承
 *    public:
 *        void init(i2c_type *bus) noexcept { m_dev.bind(bus, 0x3C); }
 *        std::optional<uint8_t> id() noexcept { return m_dev.readReg8(0x00); }
 *        bool enable(bool on) noexcept { return m_dev.writeReg8(0x01, on ? 1u : 0u); }
 *    };
 *
 * ============================================================================
 *  定位: 可选, 不是强制
 * ============================================================================
 *  - 这是 L1 总线原语 (i2c_master) 之上的一层【纯转发】, 没有状态机、没有后端
 *    抽象; 不加它, 直接调 i2cReadReg8(...) 自由函数同样正确 —— 这只是省样板。
 *  - 新驱动【建议】用它 (总线与地址只有一处); 已经上板验证过的驱动
 *    (SW3538 / SW3526 / SW2305 / FUSB302) **保持各自写法不动**, 不为统一而重测。
 *  - 只覆盖【硬件 I2C】。软件位敲 (i2c_sw) 有自己的总线描述 SwI2cBus, 不在此列
 *    —— 本库不做后端抽象层, 见 AGENTS.md §3。
 */
#ifndef LOVE_FINDER_LIB_I2C_DEVICE_HPP
#define LOVE_FINDER_LIB_I2C_DEVICE_HPP

#include <cstdint>
#include <optional>

#include "i2c_master.hpp"   /* I2cStatus + i2cMem* / i2cMaster* / i2cReadReg8 */

namespace LoveFinderLib {

class I2cDevice {
public:
    I2cDevice() = default;

    I2cDevice(i2c_type *bus, uint8_t addr7, uint32_t timeoutMs = 100) noexcept
    {
        bind(bus, addr7, timeoutMs);
    }

    /** 绑定总线与 7-bit 地址 (不含读写位, 如 0x3C)。 */
    void bind(i2c_type *bus, uint8_t addr7, uint32_t timeoutMs = 100) noexcept
    {
        m_bus       = bus;
        m_addr7     = addr7;
        m_timeoutMs = timeoutMs;
    }

    [[nodiscard]] bool      bound()   const noexcept { return m_bus != nullptr; }
    [[nodiscard]] i2c_type *bus()     const noexcept { return m_bus; }
    [[nodiscard]] uint8_t   address() const noexcept { return m_addr7; }

    void                   setTimeout(uint32_t ms) noexcept { m_timeoutMs = ms; }
    [[nodiscard]] uint32_t timeout() const noexcept { return m_timeoutMs; }

    /*---- 裸收发 (不经过寄存器地址) ----*/
    I2cStatus write(const uint8_t *data, uint16_t len) noexcept
    {
        if (m_bus == nullptr) return I2cStatus::PARAM_ERROR;
        return i2cMasterWrite(m_bus, m_addr7, data, len, m_timeoutMs);
    }

    I2cStatus read(uint8_t *data, uint16_t len) noexcept
    {
        if (m_bus == nullptr) return I2cStatus::PARAM_ERROR;
        return i2cMasterRead(m_bus, m_addr7, data, len, m_timeoutMs);
    }

    /*---- 寄存器 (带寄存器地址) ----*/
    I2cStatus writeRegs(uint8_t reg, const uint8_t *data, uint16_t len) noexcept
    {
        if (m_bus == nullptr) return I2cStatus::PARAM_ERROR;
        return i2cMemWrite(m_bus, m_addr7, reg, data, len, m_timeoutMs);
    }

    I2cStatus readRegs(uint8_t reg, uint8_t *data, uint16_t len) noexcept
    {
        if (m_bus == nullptr) return I2cStatus::PARAM_ERROR;
        return i2cMemRead(m_bus, m_addr7, reg, data, len, m_timeoutMs);
    }

    /** 写单个 8-bit 寄存器。 */
    [[nodiscard]] bool writeReg8(uint8_t reg, uint8_t value) noexcept
    {
        if (m_bus == nullptr) return false;
        return i2cWriteReg8(m_bus, m_addr7, reg, value, m_timeoutMs);
    }

    /** 读单个 8-bit 寄存器; 失败返回 nullopt (NACK / 超时 / 总线错误)。 */
    [[nodiscard]] std::optional<uint8_t> readReg8(uint8_t reg) noexcept
    {
        if (m_bus == nullptr) return std::nullopt;
        return i2cReadReg8(m_bus, m_addr7, reg, m_timeoutMs);
    }

private:
    i2c_type *m_bus       = nullptr;
    uint8_t   m_addr7     = 0;
    uint32_t  m_timeoutMs = 100;
};

} /* namespace LoveFinderLib */

#endif /* LOVE_FINDER_LIB_I2C_DEVICE_HPP */
