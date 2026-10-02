/**
 * @file    spi_bus.hpp
 * @brief   SPI 总线 + 设备片选管理 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  它解决什么 (与 spi_io 的分工)
 * ============================================================================
 *  `spi_io` 只管【搬字节】, 不知道片选; 一条 SPI 总线上挂多个设备时的
 *  【CS 归属与互斥】由本模块负责:
 *
 *    SpiBus spi1;                       // 一条总线
 *    spi1.init(SPI1);
 *    spi1.addDevice({GPIOB, GPIO_PINS_0});   // ST7735 的 CS
 *    spi1.addDevice({GPIOB, GPIO_PINS_1});   // W25Q128 的 CS
 *
 *    SpiCsLock lock = spi1.select({GPIOB, GPIO_PINS_1});
 *    if (lock.held()) {                 // 拉低该设备 CS
 *        spiTransfer(spi1.spi(), tx, rx, len);
 *    }                                  // ~lock: 等空 + 拉高 + 释放
 *
 *  比"每个驱动自己 gpio_bits_reset/set"强的地方:
 *   - 漏放 CS 不可能 (RAII), 出作用域必拉高
 *   - 两个设备同时选中会被抓住并计数 (owner / conflictCount)
 *   - releaseAll() 能一次把所有设备置为不选中 (开机安全), 单个驱动做不到
 *
 * ============================================================================
 *  注册的是【片选】, 不是设备对象
 * ============================================================================
 *  总线不需要认识设备是谁, 只需要知道"现在谁的 CS 被拉低了" —— 冲突检测按
 *  (port, pin) 值比较就够。注册设备对象会让 SpiBus 反向依赖设备类型, 在本库
 *  "无平台抽象层 / 驱动直绑" 的形态里要长回一套基类, 不值得。
 *  DC 这类【设备私有控制脚】不进总线, 留在设备驱动里 (ST7735 的 dcData 即如此)。
 *
 * ============================================================================
 *  两条硬约束 (改之前先读)
 * ============================================================================
 *  1. **不递归选中**: 同一个 CS 在已选中状态下再次 select() 同样被拒 (记冲突)。
 *     需要长事务时请在【一个锁】里做完, 不要嵌套 —— 内层析构会把 CS 提前拉高。
 *  2. **DMA 必须在锁作用域内跑完**: 锁析构只 `spiWaitIdle` (等 SPI 移位寄存器
 *     空), **不等 DMA**。用 DMA 的驱动要在锁释放前自己轮询完 DMA 完成标志,
 *     否则截断。ST7735 的 dmaTransfer() 内部就等 dtcnt 归零, 天然满足。
 */
#ifndef LOVE_FINDER_LIB_SPI_BUS_HPP
#define LOVE_FINDER_LIB_SPI_BUS_HPP

#include <cstdint>

#include "at32f421_spi.h"    /* spi_type */
#include "at32f421_gpio.h"   /* gpio_type / gpio_bits_set / gpio_bits_reset */
#include "spi_io.hpp"        /* spiWaitIdle */

namespace LoveFinderLib {

/*============================================================================
 * 片选描述 —— 纯数据, 可放进设备驱动对象
 *============================================================================*/
struct SpiCs {
    gpio_type *port = nullptr;
    uint16_t   pin  = 0u;

    [[nodiscard]] bool valid() const noexcept { return port != nullptr && pin != 0u; }
    [[nodiscard]] bool same(const SpiCs &o) const noexcept
    {
        return port == o.port && pin == o.pin;
    }
};

class SpiBus;
class SpiCsLock;

/*============================================================================
 * SpiBus
 *============================================================================*/
class SpiBus {
public:
    /** 登记表容量。一条总线几个从设备, 4 个足够; 满了 addDevice 返回 false。 */
    static constexpr uint8_t kMaxDevices = 4;

    void init(spi_type *spi) noexcept { m_spi = spi; }

    [[nodiscard]] spi_type *spi() const noexcept { return m_spi; }

    /**
     * @brief 登记一个设备片选。
     * @retval true  登记成功 (或该片选已登记过 —— 幂等, 不算错)
     * @retval false 片选无效 (port/pin 缺) 或登记表已满
     */
    bool addDevice(const SpiCs &cs) noexcept
    {
        if (!cs.valid()) return false;
        for (uint8_t i = 0; i < m_count; ++i) {
            if (m_slots[i].same(cs)) return true;
        }
        if (m_count >= kMaxDevices) return false;
        m_slots[m_count] = cs;
        ++m_count;
        return true;
    }

    /** 开机安全: 把所有登记的设备片选拉高 (全部不选中)。 */
    void releaseAll() noexcept
    {
        for (uint8_t i = 0; i < m_count; ++i) {
            gpio_bits_set(m_slots[i].port, m_slots[i].pin);
        }
        m_busy  = false;
        m_owner = SpiCs{};
    }

    /** 当前占用者; 空闲返回 nullptr。 */
    [[nodiscard]] const SpiCs *owner() const noexcept { return m_busy ? &m_owner : nullptr; }

    /** 统计发生过的冲突次数 (同时选中 / 递归选中) —— 编码错误的诊断指标。 */
    [[nodiscard]] uint32_t conflictCount() const noexcept { return m_conflicts; }

    /** 选中片选。总线忙时返回【空锁】(held()==false) 且 conflictCount++, 不抢占。 */
    [[nodiscard]] SpiCsLock select(const SpiCs &cs) noexcept;

private:
    friend class SpiCsLock;

    /** 尝试占用: 成功=false 表示被别的片选占着或重入。 */
    bool trySelect(const SpiCs &cs) noexcept
    {
        if (m_spi == nullptr || !cs.valid()) return false;
        if (m_busy) { ++m_conflicts; return false; }
        gpio_bits_reset(cs.port, cs.pin);   /* 拉低片选 */
        m_busy  = true;
        m_owner = cs;
        return true;
    }

    /** 释放: 等移位寄存器空再拉高, 避免截断最后一个字节。 */
    void endSelect() noexcept
    {
        spiWaitIdle(m_spi);
        if (m_owner.valid()) {
            gpio_bits_set(m_owner.port, m_owner.pin);
        }
        m_busy  = false;
        m_owner = SpiCs{};
    }

    spi_type *m_spi      = nullptr;
    SpiCs     m_slots[kMaxDevices]{};
    uint8_t   m_count    = 0;
    SpiCs     m_owner{};
    bool      m_busy     = false;
    uint32_t  m_conflicts = 0;
};

/*============================================================================
 * SpiCsLock —— RAII 选中守卫
 *============================================================================*/
class SpiCsLock {
public:
    SpiCsLock() = default;

    SpiCsLock(SpiBus *bus, const SpiCs &cs) noexcept
    {
        if (bus != nullptr && bus->trySelect(cs)) {
            m_bus  = bus;
            m_cs   = cs;
        }
    }

    ~SpiCsLock() { release(); }

    /* 可移动 (工厂函数返回 / 放进容器); 禁拷贝 (否则会双重释放) */
    SpiCsLock(SpiCsLock &&o) noexcept { take(o); }

    SpiCsLock &operator=(SpiCsLock &&o) noexcept
    {
        if (this != &o) {
            release();
            take(o);
        }
        return *this;
    }

    SpiCsLock(const SpiCsLock &)            = delete;
    SpiCsLock &operator=(const SpiCsLock &) = delete;

    /** 是否真的选中了 (false = 总线被别的片选占着, 本次操作应放弃)。 */
    [[nodiscard]] bool held() const noexcept { return m_bus != nullptr; }

    /** 提前释放 (幂等)。 */
    void release() noexcept
    {
        if (m_bus != nullptr) {
            m_bus->endSelect();
            m_bus = nullptr;
            m_cs  = SpiCs{};
        }
    }

    [[nodiscard]] const SpiCs &cs() const noexcept { return m_cs; }

private:
    void take(SpiCsLock &o) noexcept
    {
        m_bus    = o.m_bus;
        m_cs     = o.m_cs;
        o.m_bus  = nullptr;      /* 搬走所有权, 被移动者析构时不释放 */
        o.m_cs   = SpiCs{};
    }

    SpiBus *m_bus = nullptr;
    SpiCs   m_cs{};
};

inline SpiCsLock SpiBus::select(const SpiCs &cs) noexcept
{
    return SpiCsLock(this, cs);
}

} /* namespace LoveFinderLib */

#endif /* LOVE_FINDER_LIB_SPI_BUS_HPP */
