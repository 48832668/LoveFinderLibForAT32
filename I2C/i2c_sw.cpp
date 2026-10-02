/**
 * @file    i2c_sw.cpp
 * @brief   AT32 软件位敲 (Bit-Bang) I2C 主机实现 — C++17 (LoveFinderLibForAT32)
 *
 * 纯 GPIO 翻转 + 标准 I2C 时序，语义与 i2c_master.cpp (硬件) 一一对应:
 * 同样的 I2cStatus 结果码、同样的 7-bit 地址约定、同样的寄存器读时序
 * (repeated START，不发中间 STOP)。
 *
 * 位时序: 每个半周期 wk_delay_us(halfPeriodUs)；SCL 高电平期间被从机拉低
 * (clock stretching) 时按 lf_get_tick() 超时等待。
 */
#include "i2c_sw.hpp"
#include "wk_system.h"      /* wk_delay_us — WorkBench BSP */

namespace LoveFinderLib {

namespace {

/*============================================================================
 * 底层引脚操作 (开漏语义: 输出 1 = 释放)
 *============================================================================*/
inline void sclHigh(const SwI2cBus &b) noexcept { gpio_bits_set(b.sclPort, b.sclPin); }
inline void sclLow(const SwI2cBus &b) noexcept  { gpio_bits_reset(b.sclPort, b.sclPin); }
inline void sdaHigh(const SwI2cBus &b) noexcept { gpio_bits_set(b.sdaPort, b.sdaPin); }
inline void sdaLow(const SwI2cBus &b) noexcept  { gpio_bits_reset(b.sdaPort, b.sdaPin); }

/** 读 SCL 线电平 (用于检测 clock stretching)。 */
[[nodiscard]] inline bool sclLevel(const SwI2cBus &b) noexcept
{
    return gpio_input_data_bit_read(b.sclPort, b.sclPin) != RESET;
}

/** 读 SDA 线电平 (输出 1 时开漏已释放，IDR 即总线电平)。 */
[[nodiscard]] inline bool sdaLevel(const SwI2cBus &b) noexcept
{
    return gpio_input_data_bit_read(b.sdaPort, b.sdaPin) != RESET;
}

inline void halfDelay(const SwI2cBus &b) noexcept
{
    wk_delay_us(b.halfPeriodUs);
}

/**
 * @brief 置 SCL 高并等待线真正变高 (从机可能在拉长低电平做 clock stretching)。
 * @return true = 已高; false = 超时 (deadline 基于 lf_get_tick())。
 */
bool waitSclHigh(const SwI2cBus &b, uint32_t deadline) noexcept
{
    sclHigh(b);
    while (!sclLevel(b)) {
        if (lf_get_tick() >= deadline) return false;
    }
    return true;
}

/*============================================================================
 * 位 / 字节 原语
 *============================================================================*/
/** SCL 低电平期间改 SDA → 产生一个时钟位。 */
[[nodiscard]] bool clockOutBit(const SwI2cBus &b, bool bit, uint32_t deadline) noexcept
{
    if (bit) sdaHigh(b); else sdaLow(b);
    halfDelay(b);
    if (!waitSclHigh(b, deadline)) return false;
    halfDelay(b);
    sclLow(b);
    return true;
}

/** 释放 SDA 后采样一个位。@return -1 = 超时, 0/1 = 数据位。 */
[[nodiscard]] int clockInBit(const SwI2cBus &b, uint32_t deadline) noexcept
{
    sdaHigh(b);                 /* 释放 SDA 供从机驱动 */
    halfDelay(b);
    if (!waitSclHigh(b, deadline)) return -1;
    halfDelay(b);               /* 采样点放在 SCL 高电平【末尾】:
                                   线容性大/从机驱动慢时, 早采样会读到还没稳的值 */
    const int bit = sdaLevel(b) ? 1 : 0;
    sclLow(b);
    return bit;
}

/** 发送一个字节并读 ACK。@return OK=从机应答, NACK=未应答, TIMEOUT=超时。 */
[[nodiscard]] I2cStatus writeByte(const SwI2cBus &b, uint8_t byte, uint32_t deadline) noexcept
{
    for (int i = 7; i >= 0; --i) {
        if (!clockOutBit(b, ((byte >> i) & 0x01u) != 0u, deadline)) {
            return I2cStatus::TIMEOUT;
        }
    }

    /* ACK 位: 释放 SDA, 从机拉低表示应答 (同样在 SCL 高电平末尾采样) */
    sdaHigh(b);
    halfDelay(b);
    if (!waitSclHigh(b, deadline)) return I2cStatus::TIMEOUT;
    halfDelay(b);
    const bool acked = !sdaLevel(b);
    sclLow(b);
    return acked ? I2cStatus::OK : I2cStatus::NACK;
}

/** 接收一个字节并回 ACK/NACK。@return -1 = 超时, 0..255 = 数据。 */
[[nodiscard]] int readByte(const SwI2cBus &b, bool ack, uint32_t deadline) noexcept
{
    std::uint8_t byte = 0;
    for (int i = 7; i >= 0; --i) {
        const int bit = clockInBit(b, deadline);
        if (bit < 0) return -1;
        if (bit != 0) byte = static_cast<std::uint8_t>(byte | (1u << i));
    }

    /* 主机应答位: ACK = 拉低 SDA, NACK = 释放 */
    if (ack) sdaLow(b); else sdaHigh(b);
    halfDelay(b);
    if (!waitSclHigh(b, deadline)) return -1;
    halfDelay(b);
    sclLow(b);
    sdaHigh(b);
    return static_cast<int>(byte);
}

/** START: SDA 高→低 (SCL 高电平期间)。 */
void sendStart(const SwI2cBus &b) noexcept
{
    sdaHigh(b);
    sclHigh(b);
    halfDelay(b);
    sdaLow(b);
    halfDelay(b);
    sclLow(b);
    halfDelay(b);
}

/** STOP: SDA 低→高 (SCL 高电平期间)。 */
void sendStop(const SwI2cBus &b) noexcept
{
    sdaLow(b);
    halfDelay(b);
    sclHigh(b);
    halfDelay(b);
    sdaHigh(b);
    halfDelay(b);
}

/** 地址字节 = (7-bit 地址 << 1) | R/W。 */
[[nodiscard]] I2cStatus sendAddress(const SwI2cBus &b, uint8_t addr7, bool read,
                                    uint32_t deadline) noexcept
{
    const auto byte = static_cast<std::uint8_t>((addr7 << 1) | (read ? 0x01u : 0x00u));
    return writeByte(b, byte, deadline);
}

/** 参数检查 (与硬件版一致)。 */
[[nodiscard]] I2cStatus checkParams(const SwI2cBus &b, const void *data,
                                    uint16_t len) noexcept
{
    if (!b.valid())                return I2cStatus::PARAM_ERROR;
    if (data == nullptr && len != 0u) return I2cStatus::PARAM_ERROR;
    return I2cStatus::OK;
}

} // namespace

/*============================================================================
 * 引脚配置 (开漏输出) —— 仿 BUTTON::configureExti 的「init 即配引脚」
 *
 * ⚠ AT32F421 在【输出/复用】模式下内部上拉/下拉不生效 (AT32L021 才支持),
 *   所以这条总线必须有外部上拉 (本板 10K), 否则线永远拉不高。
 *   这里仍写上 GPIO_PULL_UP: 对本芯片是无害的空操作, 换到 L021 等型号才有效。
 *============================================================================*/
void swI2cConfigPins(const SwI2cBus &b) noexcept
{
    if (!b.valid()) return;

    gpio_init_type cfg;
    gpio_default_para_init(&cfg);
    cfg.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;
    cfg.gpio_pull           = GPIO_PULL_UP;
    cfg.gpio_mode           = GPIO_MODE_OUTPUT;
    cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;

    cfg.gpio_pins = b.sclPin;
    gpio_init(b.sclPort, &cfg);
    cfg.gpio_pins = b.sdaPin;
    gpio_init(b.sdaPort, &cfg);

    /* 空闲态: 两线都释放 */
    sdaHigh(b);
    sclHigh(b);
}

/*============================================================================
 * 裸写 / 裸读
 *============================================================================*/
I2cStatus swI2cMasterWrite(const SwI2cBus &b, uint8_t addr7, const uint8_t *data,
                           uint16_t len, uint32_t timeoutMs) noexcept
{
    if (const I2cStatus e = checkParams(b, data, len); e != I2cStatus::OK) return e;

    const uint32_t deadline = lf_get_tick() + timeoutMs;

    sendStart(b);
    if (const I2cStatus e = sendAddress(b, addr7, false, deadline); e != I2cStatus::OK) {
        sendStop(b);
        return e;
    }

    for (uint16_t i = 0; i < len; ++i) {
        if (const I2cStatus e = writeByte(b, data[i], deadline); e != I2cStatus::OK) {
            sendStop(b);
            return e;
        }
    }

    sendStop(b);
    return I2cStatus::OK;
}

I2cStatus swI2cMasterRead(const SwI2cBus &b, uint8_t addr7, uint8_t *data,
                          uint16_t len, uint32_t timeoutMs) noexcept
{
    if (const I2cStatus e = checkParams(b, data, len); e != I2cStatus::OK) return e;
    if (len == 0u) return I2cStatus::OK;

    const uint32_t deadline = lf_get_tick() + timeoutMs;

    sendStart(b);
    if (const I2cStatus e = sendAddress(b, addr7, true, deadline); e != I2cStatus::OK) {
        sendStop(b);
        return e;
    }

    for (uint16_t i = 0; i < len; ++i) {
        const bool ack = (i != static_cast<uint16_t>(len - 1u));  /* 末字节 NACK */
        const int  v   = readByte(b, ack, deadline);
        if (v < 0) {
            sendStop(b);
            return I2cStatus::TIMEOUT;
        }
        data[i] = static_cast<uint8_t>(v);
    }

    sendStop(b);
    return I2cStatus::OK;
}

/*============================================================================
 * 寄存器访问
 *============================================================================*/
I2cStatus swI2cMemWrite(const SwI2cBus &b, uint8_t addr7, uint8_t reg,
                        const uint8_t *data, uint16_t len, uint32_t timeoutMs) noexcept
{
    if (const I2cStatus e = checkParams(b, data, len); e != I2cStatus::OK) return e;
    if (len > 16u) return I2cStatus::PARAM_ERROR;   /* 与硬件版同一上限 */

    uint8_t buf[17];
    buf[0] = reg;
    for (uint16_t i = 0; i < len; ++i) {
        buf[i + 1u] = data[i];
    }
    return swI2cMasterWrite(b, addr7, buf, static_cast<uint16_t>(len + 1u), timeoutMs);
}

I2cStatus swI2cMemRead(const SwI2cBus &b, uint8_t addr7, uint8_t reg, uint8_t *data,
                       uint16_t len, uint32_t timeoutMs) noexcept
{
    if (const I2cStatus e = checkParams(b, data, len); e != I2cStatus::OK) return e;
    if (len == 0u) return I2cStatus::OK;

    const uint32_t deadline = lf_get_tick() + timeoutMs;

    /* ---- Phase 1: 写寄存器指针 (不发 STOP) ---- */
    sendStart(b);
    if (const I2cStatus e = sendAddress(b, addr7, false, deadline); e != I2cStatus::OK) {
        sendStop(b);
        return e;
    }
    if (const I2cStatus e = writeByte(b, reg, deadline); e != I2cStatus::OK) {
        sendStop(b);
        return e;
    }

    /* ---- Phase 2: repeated START + 读 ---- */
    sendStart(b);
    if (const I2cStatus e = sendAddress(b, addr7, true, deadline); e != I2cStatus::OK) {
        sendStop(b);
        return e;
    }

    for (uint16_t i = 0; i < len; ++i) {
        const bool ack = (i != static_cast<uint16_t>(len - 1u));
        const int  v   = readByte(b, ack, deadline);
        if (v < 0) {
            sendStop(b);
            return I2cStatus::TIMEOUT;
        }
        data[i] = static_cast<uint8_t>(v);
    }

    sendStop(b);
    return I2cStatus::OK;
}

/*============================================================================
 * 便捷封装
 *============================================================================*/
bool swI2cWriteReg8(const SwI2cBus &b, uint8_t addr7, uint8_t reg,
                    uint8_t value, uint32_t timeoutMs) noexcept
{
    return swI2cMemWrite(b, addr7, reg, &value, 1u, timeoutMs) == I2cStatus::OK;
}

std::optional<uint8_t> swI2cReadReg8(const SwI2cBus &b, uint8_t addr7, uint8_t reg,
                                     uint32_t timeoutMs) noexcept
{
    uint8_t v = 0;
    if (swI2cMemRead(b, addr7, reg, &v, 1u, timeoutMs) != I2cStatus::OK) {
        return std::nullopt;
    }
    return v;
}

/*============================================================================
 * 总线恢复
 *============================================================================*/
/**
 * @brief 从机把 SDA 拉死时的总线恢复 —— 最多 9 个 SCL 脉冲 + STOP。
 *
 * 这正是硬件 I2C 版 i2cMasterRecover() 注释里说「由应用层自行实现」的那部分;
 * 软件总线可以直接做，故这里给出完整实现。
 */
void swI2cRecover(const SwI2cBus &b) noexcept
{
    if (!b.valid()) return;

    sdaHigh(b);
    sclHigh(b);
    wk_delay_us(10);

    for (int i = 0; i < 9 && !sdaLevel(b); ++i) {
        sclLow(b);
        wk_delay_us(5);
        sclHigh(b);
        wk_delay_us(5);
    }

    sendStop(b);
}

} // namespace LoveFinderLib
