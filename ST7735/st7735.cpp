/**
 * @file    st7735.cpp
 * @brief   ST7735 LCD 驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  实现参考
 * ============================================================================
 *  初始化命令表与发送顺序逐条对照:
 *    · LoveFinder491_PowerOneT2 / LoveFinderLib / ST7735 / ST7735.cpp
 *    · C:\Debug\gitRepo\stm32-st7735\st7735\st7735.c   (Adafruit 派生)
 *    · C:\Debug\gitRepo\Adafruit-ST7735-Library
 *
 *  发送顺序 (与 491 的 begin() 一致):
 *      select → reset → cmds1 → MADCTL → cmds2 → cmds3 → INVON/INVOFF → deselect
 *
 *  硬件访问【直调 AT32 BSP】(仿 LoveFinderLibForPY32_LL 直调 LL_*):
 *    SPI = m_spi   GPIO = m_csPort/m_dcPort/m_resPort/m_enPort   DMA = m_dma
 *    延时 = wk_delay_ms (WorkBench)   时基 = lf_get_tick (工程 lf_irq.cpp)
 *  三者全部由 init() 绑定, 不传形参时取 lf_config.h 的 ST7735_* 宏。
 *
 *  ★ 本文件是【类成员实现】(原 60 个 extern "C" 全局函数已删除, API 只有
 *    实例方法一套)。只读的面板表 / 初始化命令表放在匿名 namespace 供所有
 *    实例共享; 可变状态 (旋转参数 / 面板索引 / 硬件句柄) 全是成员。
 */
#include "st7735.hpp"
#include "font.h"

#include <cmath>

namespace LoveFinderLib {

namespace {

/*============================================================================
 * 只读常量 — 所有实例共享 (面板表 + 初始化命令表)
 *============================================================================*/
constexpr uint8_t DELAY = 0x80;

/*============================================================================
 * 运行时面板批次表 —— A/B/C/D 四套参数全部内置, 可运行期切换
 *
 * 默认批次由 lf_config.h 的 ST7735_PANEL_A/B/C/D 决定 (ST7735_DEFAULT_PANEL_IDX);
 * 点屏排障时用 selectPanel(0..3) 切换, 再重调 init() 即生效。
 *============================================================================*/
struct PanelCfg {
    const char *name;      /* "PanelA".."PanelD" */
    bool        invert;    /* 初始化后是否需要 INVON */
    uint16_t    xoff;      /* DEG_0/180 的 X 偏移 */
    uint16_t    yoff;      /* DEG_0/180 的 Y 偏移 */
    uint8_t     madctl[4]; /* [DEG_0, DEG_90, DEG_180, DEG_270] */
};

const PanelCfg kPanels[4] = {
    /* 0: Panel A — 原批次 (BGR, 偏移 0/24, 不反色) */
    { "PanelA", false, 0, 24,
      { uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_BGR) } },
    /* 1: Panel B — 新批次 (RGB, 偏移 1/26, 需反色) */
    { "PanelB", true, 1, 26,
      { uint8_t(ST7735_MADCTL_MY | ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB),
        uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB),
        uint8_t(ST7735_MADCTL_MV | ST7735_MADCTL_RGB),
        uint8_t(ST7735_MADCTL_RGB) } },
    /* 2: Panel C — 新批次水平镜像 (BGR, 偏移 1/26, 需反色) */
    { "PanelC", true, 1, 26,
      { uint8_t(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MY | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_BGR) } },
    /* 3: Panel D — A 的变体 (BGR, 偏移 1/26, 需反色) */
    { "PanelD", true, 1, 26,
      { uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR),
        uint8_t(ST7735_MADCTL_BGR) } },
};

/*============================================================================
 * 初始化命令表 —— 逐条对照 LoveFinder491 / stm32-st7735
 *============================================================================*/
const uint8_t initCmds1[] = {
    14,
    ST7735::SWRESET, DELAY, 150,
    ST7735::SLPOUT,  DELAY, 255,
    ST7735::FRMCTR1, 3, 0x01, 0x2C, 0x2D,
    ST7735::FRMCTR2, 3, 0x01, 0x2C, 0x2D,
    ST7735::FRMCTR3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
    ST7735::INVCTR,  1, 0x07,
    ST7735::PWCTR1,  3, 0xA2, 0x02, 0x84,
    ST7735::PWCTR2,  1, 0xC5,
    ST7735::PWCTR3,  2, 0x0A, 0x00,
    ST7735::PWCTR4,  2, 0x8A, 0x2A,
    ST7735::PWCTR5,  2, 0x8A, 0xEE,
    ST7735::VMCTR1,  1, 0x0E,
    ST7735::INVOFF,  0,
    ST7735::COLMOD,  1, 0x05
};

const uint8_t initCmds2[] = {
    3,
    ST7735::CASET, 4, 0x00, 0x00, 0x00, 0x4F,
    ST7735::RASET, 4, 0x00, 0x00, 0x00, 0x9F,
    ST7735::INVOFF, 0
};

const uint8_t initCmds3[] = {
    4,
    ST7735::GMCTRP1, 16,
        0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
        0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
    ST7735::GMCTRN1, 16,
        0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
        0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
    ST7735::NORON, DELAY, 10,
    ST7735::DISPON, DELAY, 100
};

} // namespace

/*============================================================================
 * 旋转参数 — 按当前面板批次给出 MADCTL / 尺寸 / GRAM 偏移
 *
 * DEG_0 / DEG_180 为横屏 (W x H), DEG_90 / DEG_270 为竖屏 (H x W)。
 * 偏移按 491 的约定: 竖屏时 X/Y 偏移互换。
 *============================================================================*/
ST7735::RotationCfg ST7735::rotationCfgOf(uint8_t rot) const noexcept
{
    const PanelCfg &p = kPanels[m_panelIdx];
    switch (rot) {
    case ST7735::DEG_90:
        return { p.madctl[ST7735::DEG_90], ST7735::HEIGHT, ST7735::WIDTH,
                 p.yoff, p.xoff };
    case ST7735::DEG_180:
        return { p.madctl[ST7735::DEG_180], ST7735::WIDTH, ST7735::HEIGHT,
                 p.xoff, p.yoff };
    case ST7735::DEG_270:
        return { p.madctl[ST7735::DEG_270], ST7735::HEIGHT, ST7735::WIDTH,
                 p.yoff, p.xoff };
    case ST7735::DEG_0:
    default:
        return { p.madctl[ST7735::DEG_0], ST7735::WIDTH, ST7735::HEIGHT,
                 p.xoff, p.yoff };
    }
}

void ST7735::applyRotation(uint8_t rot) noexcept
{
    const RotationCfg cfg = rotationCfgOf(rot);
    m_rotation = rot;
    m_width    = cfg.width;
    m_height   = cfg.height;
    m_xStart   = cfg.xStart;
    m_yStart   = cfg.yStart;
}

/*============================================================================
 * 底层原语 — 直调 AT32 BSP (对应 PY32 库直调 LL_SPI_ / LL_GPIO_ 系列)
 *============================================================================*/
void ST7735::spiWriteByte(uint8_t b) noexcept
{
    while (spi_i2s_flag_get(m_spi, SPI_I2S_TDBE_FLAG) == RESET) {
        /* busy wait —— 前提: wk_spi1_init() 已完成, SPI 时钟已开 */
    }
    spi_i2s_data_transmit(m_spi, b);
}

/** 等 SPI 移位寄存器彻底空 —— 翻 CS 前必须等, 否则截断最后一字节。 */
void ST7735::spiWaitIdle() noexcept
{
    while (spi_i2s_flag_get(m_spi, SPI_I2S_BF_FLAG) != RESET) {
        /* busy wait */
    }
}

void ST7735::spiWriteBlock(const uint8_t *data, uint16_t len) noexcept
{
    for (uint16_t i = 0; i < len; ++i) {
        spiWriteByte(data[i]);
    }
    spiWaitIdle();
}

void ST7735::writeCommand(uint8_t cmd) noexcept
{
    dcCommand();
    spiWriteByte(cmd);
    spiWaitIdle();
}

void ST7735::writeData(const uint8_t *buff, std::size_t size) noexcept
{
    if (buff == nullptr || size == 0u) return;
    dcData();
    spiWriteBlock(buff, static_cast<uint16_t>(size));
}

/** 发送 16-bit 像素数组 (大端)。 */
void ST7735::writeData16(const uint16_t *buff, std::size_t count) noexcept
{
    if (buff == nullptr) return;
    dcData();
    for (std::size_t i = 0; i < count; ++i) {
        spiWriteByte(static_cast<uint8_t>(buff[i] >> 8));
        spiWriteByte(static_cast<uint8_t>(buff[i] & 0xFFu));
    }
    spiWaitIdle();
}

/*============================================================================
 * 设置绘图窗口 (含面板偏移)
 *============================================================================*/
void ST7735::setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) noexcept
{
    uint8_t data[4];

    writeCommand(ST7735::CASET);
    data[0] = static_cast<uint8_t>((x0 + m_xStart) >> 8);
    data[1] = static_cast<uint8_t>((x0 + m_xStart) & 0xFFu);
    data[2] = static_cast<uint8_t>((x1 + m_xStart) >> 8);
    data[3] = static_cast<uint8_t>((x1 + m_xStart) & 0xFFu);
    writeData(data, 4u);

    writeCommand(ST7735::RASET);
    data[0] = static_cast<uint8_t>((y0 + m_yStart) >> 8);
    data[1] = static_cast<uint8_t>((y0 + m_yStart) & 0xFFu);
    data[2] = static_cast<uint8_t>((y1 + m_yStart) >> 8);
    data[3] = static_cast<uint8_t>((y1 + m_yStart) & 0xFFu);
    writeData(data, 4u);

    writeCommand(ST7735::RAMWR);
}

void ST7735::executeCommandList(const uint8_t *addr) noexcept
{
    uint8_t numCommands = *addr++;
    while (numCommands-- != 0u) {
        const uint8_t cmd = *addr++;
        writeCommand(cmd);

        uint8_t numArgs = *addr++;
        const uint16_t ms = static_cast<uint16_t>(numArgs & DELAY);
        numArgs = static_cast<uint8_t>(numArgs & ~DELAY);

        if (numArgs != 0u) {
            writeData(addr, numArgs);
        }
        addr += numArgs;

        if (ms != 0u) {
            /* 延迟值字节紧跟命令表 (如 "SWRESET, DELAY, 150")。
             * ⚠ 之前漏读此字节: 下一轮循环把 150 当成命令发出,
             *   初始化序列整体错位 → 现象是"背光亮但永远没有画面"。
             *   参照 491 / PY32 / STM32_HAL: 255 → 500ms。 */
            uint16_t delayMs = *addr++;
            if (delayMs == 255u) {
                delayMs = 500u;
            }
            wk_delay_ms(delayMs);
        }
    }
}

void ST7735::resetPanel() noexcept
{
    gpio_bits_reset(m_resPort, m_resPin);
    /* RES 低电平脉宽 —— 之前 5ms 太短, 复位不彻底 -> 屏没被真正复位,
     * 旧 GRAM 内容残留 -> 切换配置时新旧文字重叠 (清屏"不生效")。
     * 按实测要求改为 120ms (>=100ms), 保证复位可靠、显存回到已知状态。 */
    wk_delay_ms(120);
    gpio_bits_set(m_resPort, m_resPin);
    wk_delay_ms(120);
}

/*============================================================================
 * 控制引脚初始化 — 把 CS/DC/RES 配成推挽输出并置初始电平
 * (仿 PY32: 驱动自己配置控制脚)
 *
 * ⚠ 逐实例一次 (原来是文件级 static, 多实例共用会漏配第二块屏的引脚)。
 *============================================================================*/
void ST7735::ensurePins() noexcept
{
    if (m_pinsCfgured) return;
    m_pinsCfgured = true;

    gpio_init_type cfg;
    gpio_default_para_init(&cfg);
    cfg.gpio_mode           = GPIO_MODE_OUTPUT;
    cfg.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    cfg.gpio_pull           = GPIO_PULL_NONE;
    cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;

    cfg.gpio_pins = m_csPin;
    gpio_init(m_csPort, &cfg);
    gpio_bits_set(m_csPort, m_csPin);

    cfg.gpio_pins = m_dcPin;
    gpio_init(m_dcPort, &cfg);
    gpio_bits_set(m_dcPort, m_dcPin);

    cfg.gpio_pins = m_resPin;
    gpio_init(m_resPort, &cfg);
    gpio_bits_set(m_resPort, m_resPin);
}

/** 像素坐标裁剪: 返回 false 表示完全在屏幕外。 */
bool ST7735::clipRect(uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h) const noexcept
{
    if (w == 0u || h == 0u) return false;
    if (x >= m_width || y >= m_height) return false;

    if (static_cast<uint32_t>(x) + w > m_width) {
        w = static_cast<uint16_t>(m_width - x);
    }
    if (static_cast<uint32_t>(y) + h > m_height) {
        h = static_cast<uint16_t>(m_height - y);
    }
    return true;
}

/** 绘制单个字形 (data 为 height 个 uint16_t, bit15 = 最左列)。 */
void ST7735::drawGlyph(uint16_t x, uint16_t y, const uint16_t *glyph,
                       uint8_t width, uint8_t height, uint16_t color, uint16_t bgcolor) noexcept
{
    for (uint8_t row = 0; row < height; ++row) {
        const uint16_t bits = glyph[row];
        for (uint8_t col = 0; col < width; ++col) {
            const bool on = (bits & static_cast<uint16_t>(0x8000u >> col)) != 0u;
            drawPixel(static_cast<uint16_t>(x + col),
                      static_cast<uint16_t>(y + row),
                      on ? color : bgcolor);
        }
    }
}

/*============================================================================
 * 初始化 — 绑定硬件 + 下发屏初始化序列
 *============================================================================*/
void ST7735::init(spi_type *spi, dma_channel_type *dma,
                  const PinSet &pins, uint8_t panelIdx) noexcept
{
    /*--------------------------------------------------------------------
     * 硬件绑定: 形参不给就用 lf_config.h 的宏 (与旧 ST7735_Init 一致)
     *--------------------------------------------------------------------*/
    if (spi != nullptr) m_spi = spi;

#if (ST7735_USE_DMA == 1)
    m_dma = (dma != nullptr) ? dma : static_cast<dma_channel_type *>(ST7735_DMA_CHANNEL);
#else
    (void)dma;
#endif

    if (pins.csPort != nullptr) { m_csPort = pins.csPort;  m_csPin  = pins.csPin;  }
    else                        { m_csPort = static_cast<gpio_type *>(ST7735_CS_PORT);
                                  m_csPin  = static_cast<uint16_t>(ST7735_CS_PIN);  }

    if (pins.dcPort != nullptr) { m_dcPort = pins.dcPort;  m_dcPin  = pins.dcPin;  }
    else                        { m_dcPort = static_cast<gpio_type *>(ST7735_DC_PORT);
                                  m_dcPin  = static_cast<uint16_t>(ST7735_DC_PIN);  }

    if (pins.resPort != nullptr){ m_resPort = pins.resPort; m_resPin = pins.resPin; }
    else                        { m_resPort = static_cast<gpio_type *>(ST7735_RES_PORT);
                                  m_resPin  = static_cast<uint16_t>(ST7735_RES_PIN); }

#ifdef ST7735_EN_PORT
    if (pins.enPort != nullptr) { m_enPort = pins.enPort;  m_enPin  = pins.enPin;  }
    else                        { m_enPort = static_cast<gpio_type *>(ST7735_EN_PORT);
                                  m_enPin  = static_cast<uint16_t>(ST7735_EN_PIN);  }
#else
    (void)pins;
#endif

    m_panelIdx = panelIdx;

    /*--------------------------------------------------------------------
     * 屏初始化序列 (与 491 的 begin() 一致)
     *--------------------------------------------------------------------*/
    ensurePins();
    applyRotation(ST7735_ROTATION);

    csHigh();
    wk_delay_ms(50);

    /* select → reset */
    csLow();
    resetPanel();

    /* cmds1 (末尾含 COLMOD, 不含 MADCTL) */
    executeCommandList(initCmds1);

    /* MADCTL —— 必须在 cmds1 之后、cmds2 之前 (与 491 begin() 一致) */
    writeCommand(ST7735::MADCTL);
    writeData8(rotationCfgOf(m_rotation).madctl);

    /* cmds2 / cmds3 */
    executeCommandList(initCmds2);
    executeCommandList(initCmds3);

    /* 面板反色极性 */
    writeCommand(kPanels[m_panelIdx].invert ? ST7735::INVON : ST7735::INVOFF);
    csHigh();

    m_initialized = true;
}

/*============================================================================
 * 面板自检 —— 点屏排查
 *============================================================================*/
void ST7735::panelSelfTest(uint16_t delayMs) noexcept
{
    if (!m_initialized) return;

    fillScreen(ST7735::RED);
    wk_delay_ms(delayMs);

    fillScreen(ST7735::GREEN);
    wk_delay_ms(delayMs);

    fillScreen(ST7735::BLUE);
    wk_delay_ms(delayMs);

    fillScreen(ST7735::WHITE);
    wk_delay_ms(delayMs);

    fillScreen(ST7735::BLACK);
}

/*============================================================================
 * 显示控制
 *============================================================================*/
void ST7735::invertColors(bool invert) noexcept
{
    writeCommand(invert ? ST7735::INVON : ST7735::INVOFF);
}

void ST7735::setGamma(GammaDef gamma) noexcept
{
    writeCommand(ST7735::GAMSET);
    writeData8(static_cast<uint8_t>(gamma));
}

/** 背光 (可选; 引脚由 init 的 PinSet 或 lf_config.h 的 ST7735_EN_* 提供)。 */
void ST7735::setBacklight(bool on) noexcept
{
    if (m_enPort == nullptr) return;

    gpio_init_type cfg;
    gpio_default_para_init(&cfg);
    cfg.gpio_pins           = m_enPin;
    cfg.gpio_mode           = GPIO_MODE_OUTPUT;
    cfg.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    cfg.gpio_pull           = GPIO_PULL_NONE;
    cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(m_enPort, &cfg);
    gpio_bits_write(m_enPort, m_enPin, on ? TRUE : FALSE);
}

/*============================================================================
 * 旋转 / 面板批次
 *============================================================================*/
void ST7735::setRotation(uint8_t rotation) noexcept
{
    if (rotation > ST7735::DEG_270) return;

    applyRotation(rotation);

    if (!m_initialized) return;

    csLow();
    writeCommand(ST7735::MADCTL);
    writeData8(rotationCfgOf(m_rotation).madctl);
    /* 重置整屏窗口到新尺寸 */
    setAddressWindow(0, 0, static_cast<uint16_t>(m_width - 1u),
                            static_cast<uint16_t>(m_height - 1u));
    csHigh();
}

void ST7735::selectPanel(uint8_t idx) noexcept
{
    if (idx >= 4u) idx = 0u;
    m_panelIdx    = idx;
    m_initialized = false;   /* 下次 init() 才按新面板重下发初始化 */
}

const char* ST7735::getPanelName() const noexcept
{
    return kPanels[m_panelIdx].name;
}

/*============================================================================
 * 基础绘制
 *============================================================================*/
void ST7735::drawPixel(uint16_t x, uint16_t y, uint16_t color) noexcept
{
    if (!m_initialized) return;
    if (x >= m_width || y >= m_height) return;

    csLow();
    setAddressWindow(x, y, x, y);
    dcData();
    spiWriteByte(static_cast<uint8_t>(color >> 8));
    spiWriteByte(static_cast<uint8_t>(color & 0xFFu));
    spiWaitIdle();
    csHigh();
}

void ST7735::fillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) noexcept
{
    if (!m_initialized) return;
    if (!clipRect(x, y, w, h)) return;

    const uint8_t hi = static_cast<uint8_t>(color >> 8);
    const uint8_t lo = static_cast<uint8_t>(color & 0xFFu);

    /* 单个地址窗口一次发完 —— 【不按 48x12 分块】。
     *
     * 历史记录 (旧批次屏实测, 保留作对照):
     *   - DrawPixel(1x1 窗口) 文字            -> 正常
     *   - FillRectangle(48x12 窗口) 蓝条       -> 正常
     *   - FillRectangle(160x80 窗口) 整屏      -> 必花屏, 且两种数据方式都花:
     *       * 整块一次性连续发 25600 字节      -> 花屏
     *       * 单窗口 + 按行分段发(每行等移位清空)-> 仍花屏
     *   - 降速到 1.875MHz (甚至更低)           -> 无效, 与 SPI 速度无关
     * => 旧批次的决定性变量是【单个地址窗口的尺寸】: >48x12 就出错。
     *   当时的对策是切成 48x12 小块, 每块独立 CS -> 小窗口 -> 数据 -> CS。
     *
     * 2026-10-02 实验: 换了一批模组, 按用户要求【取消该限制】走单窗口。
     * 若新屏复现"整屏花屏 / 无法通讯", 从 git 取回分块实现
     * (commit 25b3a95 之前的版本) 即可回退。 */
    csLow();
    setAddressWindow(x, y, static_cast<uint16_t>(x + w - 1u),
                     static_cast<uint16_t>(y + h - 1u));
    dcData();
    const uint32_t pixels = static_cast<uint32_t>(w) * h;
    for (uint32_t i = 0; i < pixels; ++i) {
        spiWriteByte(hi);
        spiWriteByte(lo);
    }
    spiWaitIdle();
    csHigh();
}

void ST7735::fillScreen(uint16_t color) noexcept
{
    fillRectangle(0, 0, m_width, m_height, color);
}

void ST7735::drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       const uint16_t *data) noexcept
{
    if (!m_initialized || data == nullptr) return;
    if (!clipRect(x, y, w, h)) return;

    csLow();
    setAddressWindow(x, y, static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + h - 1u));
    writeData16(data, static_cast<std::size_t>(w) * h);
    csHigh();
}

/*============================================================================
 * DMA 加速版本
 *
 * ST7735 是 16-bit 像素, 而 DMA 按 byte 传输, 因此先用行缓冲把 RGB565
 * 展开成大端字节流, 再分块交给 DMA。DMA 失败自动退回轮询。
 *============================================================================*/
#if (ST7735_USE_DMA == 1)
/**
 * @brief 阻塞式 DMA 发送 — 轮询通道剩余计数 + SPI 忙标志 (不依赖中断)。
 *
 * 工程 NVIC 未开 DMA 中断, 故与 PY32 一样纯轮询完成标志:
 *   1) 关通道 → 装 dtcnt/maddr → 开通道
 *   2) 等 dtcnt 归零 (DMA 搬完)
 *   3) 等 SPI BF 清零 (移位寄存器发完最后字节)
 * 超时则关通道返回 false, 上层退回轮询发送。
 */
bool ST7735::dmaTransfer(const uint8_t *data, uint16_t len, uint32_t timeoutMs) noexcept
{
    if (data == nullptr || len == 0u || m_dma == nullptr) return false;

    dma_channel_enable(m_dma, FALSE);
    m_dma->dtcnt = len;
    m_dma->maddr = reinterpret_cast<uint32_t>(data);
    dma_channel_enable(m_dma, TRUE);

    const uint32_t deadline = lf_get_tick() + timeoutMs;
    while (m_dma->dtcnt != 0u) {
        if (lf_get_tick() >= deadline) {
            dma_channel_enable(m_dma, FALSE);
            return false;
        }
    }
    while (spi_i2s_flag_get(m_spi, SPI_I2S_BF_FLAG) != RESET) {
        if (lf_get_tick() >= deadline) break;
    }

    /* 收尾必须关通道: 传输完 dtcnt 归零, 但 SPI 的 TXE 请求仍一直有效,
     * 若把通道留在使能态, 部分批次会把 CNT=0 当作 65536 再冲一轮垃圾数据。 */
    dma_channel_enable(m_dma, FALSE);
    return true;
}
#endif // ST7735_USE_DMA

void ST7735::fillRectangleDma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) noexcept
{
#if (ST7735_USE_DMA == 0)
    fillRectangle(x, y, w, h, color);
#else
    if (!m_initialized) return;
    if (!clipRect(x, y, w, h)) return;

    csLow();
    setAddressWindow(x, y, static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + h - 1u));

    const uint8_t hi = static_cast<uint8_t>(color >> 8);
    const uint8_t lo = static_cast<uint8_t>(color & 0xFFu);
    for (uint16_t i = 0; i < w; ++i) {
        m_rowBuf[i * 2u]      = hi;
        m_rowBuf[i * 2u + 1u] = lo;
    }

    dcData();
    const uint16_t rowBytes = static_cast<uint16_t>(w * 2u);
    for (uint16_t row = 0; row < h; ++row) {
        if (!dmaTransfer(m_rowBuf, rowBytes, ST7735_DMA_TIMEOUT_MS)) {
            spiWriteBlock(m_rowBuf, rowBytes);
        }
    }
    spiWaitIdle();
    csHigh();
#endif
}

void ST7735::fillScreenDma(uint16_t color) noexcept
{
    fillRectangleDma(0, 0, m_width, m_height, color);
}

void ST7735::drawImageDma(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                          const uint16_t *data) noexcept
{
#if (ST7735_USE_DMA == 0)
    drawImage(x, y, w, h, data);
#else
    if (!m_initialized || data == nullptr) return;
    if (!clipRect(x, y, w, h)) return;

    csLow();
    setAddressWindow(x, y, static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + h - 1u));

    dcData();
    const uint16_t rowBytes = static_cast<uint16_t>(w * 2u);
    for (uint16_t row = 0; row < h; ++row) {
        const uint16_t *src = data + static_cast<std::size_t>(row) * w;
        for (uint16_t i = 0; i < w; ++i) {
            m_rowBuf[i * 2u]      = static_cast<uint8_t>(src[i] >> 8);
            m_rowBuf[i * 2u + 1u] = static_cast<uint8_t>(src[i] & 0xFFu);
        }
        if (!dmaTransfer(m_rowBuf, rowBytes, ST7735_DMA_TIMEOUT_MS)) {
            spiWriteBlock(m_rowBuf, rowBytes);
        }
    }
    spiWaitIdle();
    csHigh();
#endif
}

/*============================================================================
 * 文本
 *============================================================================*/
void ST7735::writeChar(uint16_t x, uint16_t y, char ch,
                       const FontDef &font, uint16_t color, uint16_t bgcolor) noexcept
{
    if (!m_initialized) return;

    const uint8_t c = static_cast<uint8_t>(ch);
    if (c < 32u) return;

    const uint16_t *glyph = font_get_glyph(font, c);
    if (glyph == nullptr) return;

    drawGlyph(x, y, glyph, font.width, font.height, color, bgcolor);
}

void ST7735::writeString(uint16_t x, uint16_t y, const char *str,
                         const FontDef &font, uint16_t color, uint16_t bgcolor) noexcept
{
    if (!m_initialized || str == nullptr) return;

    while (*str != '\0') {
        const uint8_t ch = static_cast<uint8_t>(*str++);
        if (ch < 32u) continue;

        const uint16_t *glyph = font_get_glyph(font, ch);
        if (glyph != nullptr) {
            drawGlyph(x, y, glyph, font.width, font.height, color, bgcolor);
        } else {
            /* 工程子集里没编进来的字形: 只刷背景色,
               避免上一次显示的内容残留在该字符格里 */
            fillRectangle(x, y, font.width, font.height, bgcolor);
        }
        x = static_cast<uint16_t>(x + font.width);

        if (static_cast<uint32_t>(x) + font.width > m_width) break;
    }
}

void ST7735::writeStringUtf8(uint16_t x, uint16_t y, const char *str,
                             const FontDef &font, uint16_t color, uint16_t bgcolor) noexcept
{
    if (!m_initialized || str == nullptr) return;

    while (*str != '\0') {
        uint16_t uni = 0;
        const uint8_t b = static_cast<uint8_t>(*str);

        if (b < 0x80u) {
            uni = b;
            ++str;
        } else if ((b & 0xE0u) == 0xC0u) {
            uni = static_cast<uint16_t>((b & 0x1Fu) << 6);
            ++str;
            if (*str != '\0') uni = static_cast<uint16_t>(uni | (static_cast<uint8_t>(*str++) & 0x3Fu));
        } else if ((b & 0xF0u) == 0xE0u) {
            uni = static_cast<uint16_t>((b & 0x0Fu) << 12);
            ++str;
            if (*str != '\0') { uni = static_cast<uint16_t>(uni | ((static_cast<uint8_t>(*str) & 0x3Fu) << 6)); ++str; }
            if (*str != '\0') { uni = static_cast<uint16_t>(uni | (static_cast<uint8_t>(*str++) & 0x3Fu)); }
        } else {
            ++str;
            continue;
        }

        const uint16_t *glyph = (uni < 0x80u)
                              ? font_get_glyph(font, static_cast<uint8_t>(uni))
                              : font_get_glyph_unicode(font, uni);
        if (glyph != nullptr) {
            drawGlyph(x, y, glyph, font.width, font.height, color, bgcolor);
        }
        x = static_cast<uint16_t>(x + font.width);

        if (static_cast<uint32_t>(x) + font.width > m_width) break;
    }
}

void ST7735::print(uint16_t x, uint16_t y, const FontDef &font,
                   uint16_t color, uint16_t bgcolor, const char *format, ...)
{
    char buf[96];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    writeString(x, y, buf, font, color, bgcolor);
}

/*============================================================================
 * 图形原语
 *============================================================================*/
void ST7735::drawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) noexcept
{
    fillRectangle(x, y, w, 1, color);
}

void ST7735::drawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) noexcept
{
    fillRectangle(x, y, 1, h, color);
}

void ST7735::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) noexcept
{
    if (w == 0u || h == 0u) return;
    drawHLine(x, y, w, color);
    drawHLine(x, static_cast<uint16_t>(y + h - 1u), w, color);
    drawVLine(x, y, h, color);
    drawVLine(static_cast<uint16_t>(x + w - 1u), y, h, color);
}

void ST7735::drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                           uint8_t r, uint16_t color) noexcept
{
    if (w == 0u || h == 0u) return;
    if (r == 0u) { drawRect(x, y, w, h, color); return; }

    const uint16_t d = static_cast<uint16_t>(2u * r);
    if (w <= d || h <= d) { drawRect(x, y, w, h, color); return; }

    drawHLine(static_cast<uint16_t>(x + r), y, static_cast<uint16_t>(w - d), color);
    drawHLine(static_cast<uint16_t>(x + r), static_cast<uint16_t>(y + h - 1u),
              static_cast<uint16_t>(w - d), color);
    drawVLine(x, static_cast<uint16_t>(y + r), static_cast<uint16_t>(h - d), color);
    drawVLine(static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + r),
              static_cast<uint16_t>(h - d), color);

    const int16_t rr = static_cast<int16_t>(r);
    for (int16_t i = 0; i <= rr; ++i) {
        for (int16_t j = 0; j <= rr; ++j) {
            if ((i * i + j * j) <= rr * rr) {
                drawPixel(static_cast<uint16_t>(x + r - i), static_cast<uint16_t>(y + r - j), color);
                drawPixel(static_cast<uint16_t>(x + w - r - 1 + i), static_cast<uint16_t>(y + r - j), color);
                drawPixel(static_cast<uint16_t>(x + r - i), static_cast<uint16_t>(y + h - r - 1 + j), color);
                drawPixel(static_cast<uint16_t>(x + w - r - 1 + i), static_cast<uint16_t>(y + h - r - 1 + j), color);
            }
        }
    }
}

void ST7735::fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                           uint8_t r, uint16_t color) noexcept
{
    if (w == 0u || h == 0u) return;
    if (r == 0u) { fillRectangle(x, y, w, h, color); return; }

    const uint16_t d = static_cast<uint16_t>(2u * r);
    if (w <= d || h <= d) { fillRectangle(x, y, w, h, color); return; }

    fillRectangle(static_cast<uint16_t>(x + r), y, static_cast<uint16_t>(w - d), h, color);
    fillRectangle(x, static_cast<uint16_t>(y + r), r, static_cast<uint16_t>(h - d), color);
    fillRectangle(static_cast<uint16_t>(x + w - r), static_cast<uint16_t>(y + r),
                  r, static_cast<uint16_t>(h - d), color);

    const int16_t rr = static_cast<int16_t>(r);
    for (int16_t i = 0; i <= rr; ++i) {
        for (int16_t j = 0; j <= rr; ++j) {
            if ((i * i + j * j) <= rr * rr) {
                drawPixel(static_cast<uint16_t>(x + r - i), static_cast<uint16_t>(y + r - j), color);
                drawPixel(static_cast<uint16_t>(x + w - r - 1 + i), static_cast<uint16_t>(y + r - j), color);
                drawPixel(static_cast<uint16_t>(x + r - i), static_cast<uint16_t>(y + h - r - 1 + j), color);
                drawPixel(static_cast<uint16_t>(x + w - r - 1 + i), static_cast<uint16_t>(y + h - r - 1 + j), color);
            }
        }
    }
}

void ST7735::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) noexcept
{
    int16_t dx = static_cast<int16_t>(x1 - x0);
    int16_t dy = static_cast<int16_t>(y1 - y0);
    const int16_t stepx = (dx >= 0) ? 1 : -1;
    const int16_t stepy = (dy >= 0) ? 1 : -1;
    dx = (dx >= 0) ? dx : static_cast<int16_t>(-dx);
    dy = (dy >= 0) ? dy : static_cast<int16_t>(-dy);

    int16_t err = static_cast<int16_t>(dx - dy);
    for (;;) {
        if (x0 >= 0 && y0 >= 0) {
            drawPixel(static_cast<uint16_t>(x0), static_cast<uint16_t>(y0), color);
        }
        if (x0 == x1 && y0 == y1) break;

        const int16_t e2 = static_cast<int16_t>(2 * err);
        if (e2 > -dy) { err = static_cast<int16_t>(err - dy); x0 = static_cast<int16_t>(x0 + stepx); }
        if (e2 < dx)  { err = static_cast<int16_t>(err + dx); y0 = static_cast<int16_t>(y0 + stepy); }
    }
}

void ST7735::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) noexcept
{
    int16_t x = 0;
    int16_t y = r;
    int16_t err = static_cast<int16_t>(1 - r);

    while (x <= y) {
        const int16_t px[8] = { static_cast<int16_t>(x0 + x), static_cast<int16_t>(x0 - x),
                                static_cast<int16_t>(x0 + x), static_cast<int16_t>(x0 - x),
                                static_cast<int16_t>(x0 + y), static_cast<int16_t>(x0 - y),
                                static_cast<int16_t>(x0 + y), static_cast<int16_t>(x0 - y) };
        const int16_t py[8] = { static_cast<int16_t>(y0 + y), static_cast<int16_t>(y0 + y),
                                static_cast<int16_t>(y0 - y), static_cast<int16_t>(y0 - y),
                                static_cast<int16_t>(y0 + x), static_cast<int16_t>(y0 + x),
                                static_cast<int16_t>(y0 - x), static_cast<int16_t>(y0 - x) };
        for (int k = 0; k < 8; ++k) {
            if (px[k] >= 0 && py[k] >= 0) {
                drawPixel(static_cast<uint16_t>(px[k]), static_cast<uint16_t>(py[k]), color);
            }
        }

        ++x;
        if (err < 0) {
            err = static_cast<int16_t>(err + 2 * x + 1);
        } else {
            --y;
            err = static_cast<int16_t>(err + 2 * (x - y) + 1);
        }
    }
}

void ST7735::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) noexcept
{
    int16_t x = 0;
    int16_t y = r;
    int16_t err = static_cast<int16_t>(1 - r);

    while (x <= y) {
        fillRectangle(static_cast<uint16_t>(x0 - x), static_cast<uint16_t>(y0 + y),
                      static_cast<uint16_t>(2 * x + 1), 1, color);
        fillRectangle(static_cast<uint16_t>(x0 - x), static_cast<uint16_t>(y0 - y),
                      static_cast<uint16_t>(2 * x + 1), 1, color);
        fillRectangle(static_cast<uint16_t>(x0 - y), static_cast<uint16_t>(y0 + x),
                      static_cast<uint16_t>(2 * y + 1), 1, color);
        fillRectangle(static_cast<uint16_t>(x0 - y), static_cast<uint16_t>(y0 - x),
                      static_cast<uint16_t>(2 * y + 1), 1, color);

        ++x;
        if (err < 0) {
            err = static_cast<int16_t>(err + 2 * x + 1);
        } else {
            --y;
            err = static_cast<int16_t>(err + 2 * (x - y) + 1);
        }
    }
}

void ST7735::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                          int16_t x2, int16_t y2, uint16_t color) noexcept
{
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}

/** 扫描线填充三角形的一条水平边 (修正 491 参考中两侧共用同一边的退化问题)。 */
void ST7735::fillSpan(int16_t xa, int16_t ya, int16_t xb, int16_t yb,
                      int16_t xc, int16_t yc, uint16_t color) noexcept
{
    const int16_t dxa = static_cast<int16_t>(xb - xa);
    const int16_t dxc = static_cast<int16_t>(xc - xa);
    const int16_t dya = static_cast<int16_t>(yb - ya);
    const int16_t dyc = static_cast<int16_t>(yc - ya);
    if (dya == 0 || dyc == 0) return;

    for (int16_t y = ya; y <= yb; ++y) {
        int16_t xs = static_cast<int16_t>(xa + (y - ya) * dxa / dya);
        int16_t xe = static_cast<int16_t>(xa + (y - ya) * dxc / dyc);
        if (xs > xe) swapI16(xs, xe);
        if (xs < 0) xs = 0;
        fillRectangle(static_cast<uint16_t>(xs), static_cast<uint16_t>(y),
                      static_cast<uint16_t>(xe - xs + 1), 1, color);
    }
}

void ST7735::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                          int16_t x2, int16_t y2, uint16_t color) noexcept
{
    /* 按 y 升序排序三个顶点 */
    if (y0 > y1) { swapI16(y0, y1); swapI16(x0, x1); }
    if (y1 > y2) { swapI16(y1, y2); swapI16(x1, x2); }
    if (y0 > y1) { swapI16(y0, y1); swapI16(x0, x1); }

    if (y0 == y2) {
        int16_t xs = x0, xe = x0;
        if (x1 < xs) xs = x1;
        if (x2 < xs) xs = x2;
        if (x1 > xe) xe = x1;
        if (x2 > xe) xe = x2;
        if (xs < 0) xs = 0;
        fillRectangle(static_cast<uint16_t>(xs), static_cast<uint16_t>(y0),
                      static_cast<uint16_t>(xe - xs + 1), 1, color);
        return;
    }

    if (y1 == y2) {
        fillSpan(x0, y0, x2, y2, x1, y1, color);
    } else if (y0 == y1) {
        fillSpan(x0, y0, x1, y1, x2, y2, color);
    } else {
        const int16_t x3 = static_cast<int16_t>(x0 + (x2 - x0) * (y1 - y0) / (y2 - y0));
        fillSpan(x0, y0, x1, y1, x3, y1, color);
        fillSpan(x2, y2, x1, y1, x3, y1, color);
    }
}

} // namespace LoveFinderLib
