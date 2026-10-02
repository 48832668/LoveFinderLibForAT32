/**
 * @file    spi_io.hpp
 * @brief   AT32 阻塞式硬件 SPI 收发 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计: 库只做【运行时收发】, 初始化全部交给 ATWK
 * ============================================================================
 *  - 前置条件: 工程已在 ATWK 上配好 GPIO 复用与 SPI 外设参数 (CPOL / CPHA /
 *    分频 / 帧宽 / CS 模式), 并在 main 里调用过 wk_xxx_init()。
 *  - 本模块【不调用】 spi_init / gpio_init, 也【不在运行中修改】任何 SPI
 *    硬件参数 —— ATWK 设成什么样, 就按什么样收发。
 *  - 外设实例由调用方传入 (如 SPI1), 与 i2c_master 收 i2c_type 同一模式。
 *  - 【CS 不归本模块管】: 片选是设备驱动的事 (ST7735 有自己的 csLow / csHigh
 *    成员), 这里只搬字节。调用顺序由驱动负责: csLow -> 收发 -> spiWaitIdle
 *    -> csHigh, 漏掉中间的 spiWaitIdle 会截断最后一个字节。
 *
 * ============================================================================
 *  写路径【不带超时】, 读路径【带超时】—— 这个不对称是有意的
 * ============================================================================
 *  - 写: 忙等 TDBE, 与 st7735.cpp 已上板验证的实现逐位一致。若每字节都调一次
 *    lf_get_tick(), 160x80 整屏就是 25600 次额外函数调用, -O0 下明显拖慢刷屏。
 *    代价: SPI 时钟没开时会永久忙等 (这正是 st7735.hpp 警示过的场景),
 *    此时应当是看门狗/硬件问题, 靠驱动层超时兜不住。
 *  - 读: 从机可能根本不回数据 (未接从机 / 引脚配错), RDBF 永远不来,
 *    必须有界 —— 所以读路径逐字节查 lf_get_tick()。读的字节数通常是个位数到
 *    几百, 这点开销无感。
 *
 * ============================================================================
 *  只写不读时累积的 RDBF / ROERR 是【正常且无害】的
 * ============================================================================
 *  SPI 是全双工: 每发一个字节就产生一个接收字节。只写路径【刻意不清】接收
 *  缓冲 —— AT32 用 RDBF / ROERR 只管理接收路径, 置位不影响发送。本库实测:
 *  ST7735 全程只写 (含整屏 DMA), 上板验证正常。
 *  要读之前不必手工处理: spiReadBlock / spiTransfer 进函数先 drain 残留并清
 *  ROERR, 读完才开始收数据。
 *
 * ============================================================================
 *  用法
 * ============================================================================
 *    using namespace LoveFinderLib;
 *
 *    csLow();
 *    spiWriteByte(SPI1, 0x2A);              // 写命令字节
 *    spiWriteBlock(SPI1, frame, sizeof frame);
 *    spiWaitIdle(SPI1);                     // 翻 CS 前必须
 *    csHigh();
 *
 *    uint8_t id[3];
 *    if (spiReadBlock(SPI1, id, 3)) { ... } // 读 (发 0xFF dummy), 失败即超时
 *
 * ============================================================================
 *  工程侧需要提供 (lf_config.h)
 * ============================================================================
 *    uint32_t lf_get_tick(void);            // SysTick 1ms 递增的时基
 */
#ifndef LOVEFINDER_SPI_IO_HPP
#define LOVEFINDER_SPI_IO_HPP

#include <cstdint>

#include "lf_config.h"      /* 工程侧时基: lf_get_tick (读路径超时用) */

/* ★ 自带 BSP 头 (库 AGENTS.md 已知约束一节的要求): conf.h 里的模块宏决定
 *    哪些头被条件包含, 本模块自己把用到的头列全, 不指望宿主工程的 conf
 *    顺带带出来 —— 否则宿主没勾 SPI 就编不过。 */
#include "at32f421_spi.h"   /* spi_type / spi_i2s_flag_get / SPI_I2S_*_FLAG */

namespace LoveFinderLib {

/*============================================================================
 * 写路径 — 忙等, 无超时 (热路径, 与已验证的 ST7735 实现一致)
 *============================================================================*/

/** 发一个字节: 等 TDBE 然后写数据寄存器。 */
void spiWriteByte(spi_type *spi, uint8_t b) noexcept;

/** 等移位寄存器彻底空 —— 翻 CS 前必须调, 否则截断最后一个字节。 */
void spiWaitIdle(spi_type *spi) noexcept;

/** 连发一块并在结尾等空。data 为空或 len 为 0 时直接返回。 */
void spiWriteBlock(spi_type *spi, const uint8_t *data, uint16_t len) noexcept;

/*============================================================================
 * 读 / 全双工 — 带超时, 进入时先 drain 残留并清 ROERR
 *============================================================================*/

/** 把接收缓冲读空并清 ROERR。缓冲本来就是空的话立即返回 true。 */
bool spiDrainRx(spi_type *spi, uint32_t timeoutMs = 100) noexcept;

/**
 * @brief 纯读: 每字节发 dummyTx 一个字节, 把收到的填进 rx。
 * @retval true 全部收到; false 任一步超时 (此时 rx 内容不确定)。
 *
 * 典型用于读从机 ID / 寄存器: spiReadBlock(SPI1, id, 3) 发 3 个 0xFF 收 3 字节。
 */
bool spiReadBlock(spi_type *spi, uint8_t *rx, uint16_t len,
                  uint8_t dummyTx = 0xFF,
                  uint32_t timeoutMs = 100) noexcept;

/**
 * @brief 全双工: 每字节同时发 tx[i] 收进 rx[i]。
 * @retval true 全部完成; false 任一步超时。
 *
 * tx 与 rx 都必须有效 —— 纯写用 spiWriteBlock, 纯读用 spiReadBlock。
 */
bool spiTransfer(spi_type *spi, const uint8_t *tx, uint8_t *rx, uint16_t len,
                 uint32_t timeoutMs = 100) noexcept;

} /* namespace LoveFinderLib */

#endif /* LOVEFINDER_SPI_IO_HPP */
