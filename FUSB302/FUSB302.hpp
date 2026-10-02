/**
 * @file    FUSB302.hpp
 * @brief   FUSB302 USB Type-C / PD 控制器驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计 (与 SW3526 同款: 直绑工程头 + 厂商 BSP, 无抽象层)
 * ============================================================================
 *  - I2C 收发经库内 I2C/i2c_master 阻塞驱动直发, 本驱动只持有
 *    `i2c_type *` 外设实例 (工程 lf_config.h 提供实例宏, 如 LF_I2C_BUS1 = I2C2)。
 *  - 只支持硬件 I2C (907 的 FUSB302 挂 I2C2: PA0=SCL / PA1=SDA);
 *    若某工程没有空余硬件 I2C, 再仿 SW3526 加 initBitbang。
 *  - 7-bit 地址固定 0x22 (数据手册: 0b0100010, 无地址引脚变体除外)。
 *
 *  第一阶段 (当前): 通讯自检 —— 读 Device ID (0x01) 验证总线/芯片在线。
 *  CC 检测 / PD 报文等后续功能按数据手册在此基础上扩展。
 *
 * ============================================================================
 *  典型用法
 * ============================================================================
 *    LoveFinderLib::FUSB302 ct;
 *    ct.init(LF_I2C_BUS1, 0x22, "FUSB302");   // LF_I2C_BUS1 = I2C2 (lf_config.h)
 *    if (ct.probe()) { ... }                  // 读 Device ID 成功 = 在线
 *    auto id = ct.getDeviceId();              // 0x90 = FUSB302B revA 等
 *
 * ============================================================================
 *  Device ID (寄存器 0x01, 只读)
 * ============================================================================
 *    [7:4] Version ID   A=0x8 / B=0x9 / C=0xA (由 Trim 决定)
 *    [3:2] Product ID   00 = FUSB302BMPX/UCX, 01/10/11 = B01/B10/B11 (仅 MLP)
 *    [1:0] Revision ID  revA=00, revB=01, revC=10, revD=11
 *    → 复位值约 0x10xx (数据手册 Table 5), 实读 0x80~0xAF 属正常范围。
 */
#ifndef FUSB302_HPP
#define FUSB302_HPP

#include "lf_config.h"      /* ★ 工程侧: I2C 实例宏 + lf_get_tick() 时基 */
#include "i2c_master.hpp"   /* 阻塞式 I2C 主机驱动 (I2cStatus / i2cMemRead...) */

#include <cstddef>
#include <cstdint>
#include <optional>

namespace LoveFinderLib {

/*============================================================================
 * 编译期默认值 (可在 lf_config.h 中覆盖)
 *============================================================================*/
#ifndef FUSB302_I2C_ADDR_7BIT
#define FUSB302_I2C_ADDR_7BIT    0x22    /* 7-bit 地址 (固定) */
#endif

#ifndef FUSB302_I2C_TIMEOUT_MS
#define FUSB302_I2C_TIMEOUT_MS   50
#endif

/** writeBurst 单次 I2C 写的最大字节数 (1 字节寄存器 + 最长 40B PD 包)。 */
#ifndef FUSB302_BURST_MAX
#define FUSB302_BURST_MAX        48
#endif

/*============================================================================
 * 寄存器地址 (FUSB302 / FUSB302B 数据手册 Register Definitions)
 *============================================================================*/
enum class e_FUSB302_Reg : uint8_t {
    DEVICE_ID   = 0x01,   ///< R    Version/Product/Revision
    SWITCHES0   = 0x02,   ///< R/W  PU_EN / VCONN / MEAS_CC / PDWN
    SWITCHES1   = 0x03,   ///< R/W  POWERROLE / SPECREV / DATAROLE / TXCC
    MEASURE     = 0x04,   ///< R/W  MEAS_VBUS / MDAC
    SLICE       = 0x05,   ///< R/W  SDAC / hysteresis
    CONTROL0    = 0x06,   ///< R/W  INT/RANGE/MED/METAL DETECT
    CONTROL1    = 0x07,   ///< R/W  HOST_CUR / TGT_ID
    CONTROL2    = 0x08,   ///< R/W  TOGGLE / WAKE / LATCH
    CONTROL3    = 0x09,   ///< R/W  AUTO_CRC / host disable
    MASK1       = 0x0A,   ///< R/W  中断屏蔽 (VBus OK / BC_LVL / ... )
    POWER       = 0x0B,   ///< R/W  PWR[3:0] 内部模块上电 (PD 必开 = 0x07)
    RESET       = 0x0C,   ///< W    SW_RES / SW_RES_PIN
    MASKA       = 0x0E,   ///< R/W  Maska 中断屏蔽
    MASKB       = 0x0F,   ///< R/W  Maskb 中断屏蔽
    STATUS0A    = 0x3C,   ///< R    Maska 汇总的中断状态
    STATUS1A    = 0x3D,   ///< R    VBUS 中断状态
    INTERRUPTA  = 0x3E,   ///< R    Maska 屏蔽前的中断
    INTERRUPTB  = 0x3F,   ///< R    Maskb 屏蔽前的中断
    STATUS0     = 0x40,   ///< R    BC_LVL / VBUSOK / ACT / TOGDONE / SRTV
    STATUS1     = 0x41,   ///< R    TOGSS / BC_LVL / CC 状态
    INTERRUPT   = 0x42,   ///< R    I_VBUSOK ... I_TOGDONE
    FIFO        = 0x43    ///< R/W  TX/RX FIFO (FIFO 命令 + PD 载荷)
};

/*============================================================================
 * FUSB302
 *============================================================================*/
class FUSB302 {
public:
    FUSB302() = default;

    /**
     * @param bus   I2C 外设实例 (由工程 lf_config.h 提供, 如 LF_I2C_BUS1 = I2C2)
     * @param addr7 7-bit 从机地址 (FUSB302 为 0x22)
     * @param name  设备名 (调试用)
     */
    FUSB302(i2c_type *bus, uint8_t addr7 = FUSB302_I2C_ADDR_7BIT,
            const char *name = "FUSB302")
    {
        init(bus, addr7, name);
    }

    void init(i2c_type *bus, uint8_t addr7 = FUSB302_I2C_ADDR_7BIT,
              const char *name = "FUSB302") noexcept
    {
        m_i2c   = bus;
        m_addr7 = addr7;
        m_name  = (name != nullptr) ? name : "FUSB302";
    }

    /*---------- 配置 ----------*/
    void setTimeout(uint32_t ms) noexcept { m_timeoutMs = ms; }

    /*---------- 寄存器操作 ----------*/
    std::optional<uint8_t> readReg(e_FUSB302_Reg reg) noexcept
    {
        return readReg(static_cast<uint8_t>(reg));
    }
    std::optional<uint8_t> readReg(uint8_t reg) noexcept;

    bool writeReg(e_FUSB302_Reg reg, uint8_t value) noexcept
    {
        return writeReg(static_cast<uint8_t>(reg), value);
    }
    bool writeReg(uint8_t reg, uint8_t value) noexcept;

    /**
     * @brief 批量写 (起始寄存器 + 连续 payload, 一次 I2C 事务)。
     *
     * FIFO (0x43) 用: 一条 START + addr(W) + 0x43 + 字节流 —— FUSB302 的
     * FIFO 寄存器地址自增是"往 TX FIFO 里连续压包", PD 收发 (PDSink) 靠它
     * 把 SOP token / 报头 / 载荷 / EOP 一次写完。
     */
    bool writeBytes(e_FUSB302_Reg reg, const uint8_t *buf, std::size_t len) noexcept
    {
        return writeBytes(static_cast<uint8_t>(reg), buf, len);
    }
    bool writeBytes(uint8_t reg, const uint8_t *buf, std::size_t len) noexcept;

    /**
     * @brief 一次 I2C 写: reg + buf, **不分片** (允许 >16B)。
     *
     * FIFO (0x43) 整包压入用 —— 参考实现 (FUSB302-master) 就是把
     * "寄存器地址 + SOP token + 报头 + 载荷 + EOP + TXON" 一次 burst 写完;
     * i2cMemWrite 有 ≤16B 的缓冲限制, 不能用于最长 40B 的 PD 包。
     */
    bool writeBurst(e_FUSB302_Reg reg, const uint8_t *buf, std::size_t len) noexcept
    {
        return writeBurst(static_cast<uint8_t>(reg), buf, len);
    }
    bool writeBurst(uint8_t reg, const uint8_t *buf, std::size_t len) noexcept;

    /**
     * @brief 批量读 (起始寄存器 + 连续读取)。RX FIFO 用: 一条事务把
     *        token + 报头 + 载荷 + CRC 一次读出 (PDSink::rxPump)。
     */
    bool readBytes(e_FUSB302_Reg reg, uint8_t *buf, std::size_t len) noexcept
    {
        return readBytes(static_cast<uint8_t>(reg), buf, len);
    }
    bool readBytes(uint8_t reg, uint8_t *buf, std::size_t len) noexcept;

    /*---------- 芯片信息 / 自检 ----------*/
    /** 读 Device ID (0x01)。总线超时 / NACK → nullopt。 */
    std::optional<uint8_t> getDeviceId() noexcept;

    /**
     * @brief 探测芯片是否在线: Device ID 读得回且不是 0x00/0xFF 哨兵值。
     *
     * 读得回 = 地址被 ACK (有器件应答); 0x00/0xFF 再滤掉
     * 总线被拉死却凑巧读回全 0/全 1 的假成功。
     */
    [[nodiscard]] bool probe() noexcept;

    /*---------- 访问器 ----------*/
    [[nodiscard]] const char *getName() const noexcept { return m_name; }
    [[nodiscard]] uint8_t getAddr7() const noexcept { return m_addr7; }

private:
    i2c_type   *m_i2c       = nullptr;
    uint8_t     m_addr7     = FUSB302_I2C_ADDR_7BIT;
    uint32_t    m_timeoutMs = FUSB302_I2C_TIMEOUT_MS;
    const char *m_name      = "FUSB302";
};

} // namespace LoveFinderLib

#endif /* FUSB302_HPP */
