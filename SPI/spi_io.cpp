/**
 * @file    spi_io.cpp
 * @brief   AT32 阻塞式硬件 SPI 收发实现
 *
 * 设计要点见 spi_io.hpp。三处与 st7735.cpp 已上板实现保持一致的地方:
 *   - 写: 等 SPI_I2S_TDBE_FLAG -> spi_i2s_data_transmit, 不查超时
 *   - 等空: 等 SPI_I2S_BF_FLAG 复位 (翻 CS 前必须)
 *   - 只写路径不 drain 接收缓冲
 * 改动这三点前先上板回归一次 ST7735 整屏显示。
 */
#include "spi_io.hpp"

namespace LoveFinderLib {

/*============================================================================
 * 写路径 — 忙等, 无超时
 *============================================================================*/
void spiWriteByte(spi_type *spi, uint8_t b) noexcept
{
    /* busy wait —— 前提: 工程已 wk_xxx_init(), SPI 时钟已开 */
    while (spi_i2s_flag_get(spi, SPI_I2S_TDBE_FLAG) == RESET) {
    }
    spi_i2s_data_transmit(spi, b);
}

void spiWaitIdle(spi_type *spi) noexcept
{
    /* 等移位寄存器彻底空 —— 翻 CS 前必须等, 否则截断最后一字节 */
    while (spi_i2s_flag_get(spi, SPI_I2S_BF_FLAG) != RESET) {
    }
}

void spiWriteBlock(spi_type *spi, const uint8_t *data, uint16_t len) noexcept
{
    if (data == nullptr || len == 0u) return;
    for (uint16_t i = 0; i < len; ++i) {
        spiWriteByte(spi, data[i]);
    }
    spiWaitIdle(spi);
}

/*============================================================================
 * 读 / 全双工 — 带超时
 *============================================================================*/
bool spiDrainRx(spi_type *spi, uint32_t timeoutMs) noexcept
{
    const uint32_t deadline = lf_get_tick() + timeoutMs;

    /* 把接收缓冲读空: RDBF 置位就说明还有字节躺在数据寄存器里 */
    while (spi_i2s_flag_get(spi, SPI_I2S_RDBF_FLAG) != RESET) {
        (void)spi_i2s_data_receive(spi);
        if (lf_get_tick() >= deadline) return false;
    }

    /* 清溢出错标志 —— 只写期间数据被覆盖是预期行为, 读之前必须清掉,
       否则首字节可能拿到旧数据。 */
    spi_i2s_flag_clear(spi, SPI_I2S_ROERR_FLAG);
    return true;
}

/** 等 TDBE (带超时)。 */
static bool waitTdbe(spi_type *spi, uint32_t deadline) noexcept
{
    while (spi_i2s_flag_get(spi, SPI_I2S_TDBE_FLAG) == RESET) {
        if (lf_get_tick() >= deadline) return false;
    }
    return true;
}

/** 等收到一个字节 (带超时), 到手后从数据寄存器取走。 */
static bool waitRdbf(spi_type *spi, uint32_t deadline, uint8_t &out) noexcept
{
    while (spi_i2s_flag_get(spi, SPI_I2S_RDBF_FLAG) == RESET) {
        if (lf_get_tick() >= deadline) return false;
    }
    out = static_cast<uint8_t>(spi_i2s_data_receive(spi));
    return true;
}

bool spiReadBlock(spi_type *spi, uint8_t *rx, uint16_t len,
                  uint8_t dummyTx, uint32_t timeoutMs) noexcept
{
    if (spi == nullptr || rx == nullptr || len == 0u) return false;
    if (!spiDrainRx(spi, timeoutMs)) return false;

    for (uint16_t i = 0; i < len; ++i) {
        if (!waitTdbe(spi, lf_get_tick() + timeoutMs)) return false;
        spi_i2s_data_transmit(spi, dummyTx);
        if (!waitRdbf(spi, lf_get_tick() + timeoutMs, rx[i])) return false;
    }
    return true;
}

bool spiTransfer(spi_type *spi, const uint8_t *tx, uint8_t *rx, uint16_t len,
                 uint32_t timeoutMs) noexcept
{
    if (spi == nullptr || tx == nullptr || rx == nullptr || len == 0u) return false;
    if (!spiDrainRx(spi, timeoutMs)) return false;

    for (uint16_t i = 0; i < len; ++i) {
        if (!waitTdbe(spi, lf_get_tick() + timeoutMs)) return false;
        spi_i2s_data_transmit(spi, tx[i]);
        if (!waitRdbf(spi, lf_get_tick() + timeoutMs, rx[i])) return false;
    }
    return true;
}

} /* namespace LoveFinderLib */
