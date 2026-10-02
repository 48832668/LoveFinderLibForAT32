/**
 * @file    FUSB302.cpp
 * @brief   FUSB302 USB Type-C / PD 控制器驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * 全部走库内阻塞式硬件 I2C (I2C/i2c_master); 读时序为
 * START + addr(W) + reg + repeated START + addr(R) + data + STOP。
 */
#include "FUSB302.hpp"

namespace LoveFinderLib {

/*============================================================================
 * 寄存器访问
 *============================================================================*/
std::optional<uint8_t> FUSB302::readReg(uint8_t reg) noexcept
{
    if (m_i2c == nullptr) return std::nullopt;

    uint8_t v = 0;
    if (i2cMemRead(m_i2c, m_addr7, reg, &v, 1, m_timeoutMs) != I2cStatus::OK) {
        return std::nullopt;
    }
    return v;
}

bool FUSB302::writeReg(uint8_t reg, uint8_t value) noexcept
{
    if (m_i2c == nullptr) return false;
    return i2cWriteReg8(m_i2c, m_addr7, reg, value, m_timeoutMs);
}

bool FUSB302::writeBytes(uint8_t reg, const uint8_t *buf, std::size_t len) noexcept
{
    if (m_i2c == nullptr || (buf == nullptr && len != 0u)) return false;
    if (len == 0u) return true;

    /* i2cMemWrite 单事务上限 16B (库内静态 buf), 超了就分片 —— 每片重发一次
       寄存器地址; 对 FIFO (0x43) 而言地址固定, 分片只是把 TX FIFO 连续压满。 */
    std::size_t off = 0;
    while (off < len) {
        std::size_t n = len - off;
        if (n > 16u) n = 16u;
        if (i2cMemWrite(m_i2c, m_addr7, reg, buf + off,
                        static_cast<uint16_t>(n), m_timeoutMs) != I2cStatus::OK) {
            return false;
        }
        off += n;
    }
    return true;
}

bool FUSB302::writeBurst(uint8_t reg, const uint8_t *buf, std::size_t len) noexcept
{
    if (m_i2c == nullptr || (buf == nullptr && len != 0u)) return false;
    if (len + 1u > FUSB302_BURST_MAX) return false;

    uint8_t tmp[FUSB302_BURST_MAX];
    tmp[0] = reg;
    for (std::size_t i = 0; i < len; ++i) tmp[i + 1u] = buf[i];
    return i2cMasterWrite(m_i2c, m_addr7, tmp,
                          static_cast<uint16_t>(len + 1u), m_timeoutMs) == I2cStatus::OK;
}

bool FUSB302::readBytes(uint8_t reg, uint8_t *buf, std::size_t len) noexcept
{
    if (m_i2c == nullptr || (buf == nullptr && len != 0u)) return false;
    if (len == 0u) return true;
    return i2cMemRead(m_i2c, m_addr7, reg, buf, len, m_timeoutMs) == I2cStatus::OK;
}

/*============================================================================
 * 芯片信息 / 自检
 *============================================================================*/
std::optional<uint8_t> FUSB302::getDeviceId() noexcept
{
    return readReg(e_FUSB302_Reg::DEVICE_ID);   /* 0x01, 只读 */
}

bool FUSB302::probe() noexcept
{
    const auto id = getDeviceId();
    if (!id.has_value()) return false;
    return (*id != 0x00u) && (*id != 0xFFu);
}

} // namespace LoveFinderLib
