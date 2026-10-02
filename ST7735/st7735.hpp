/**
 * @file    st7735.hpp
 * @brief   ST7735 LCD 驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计要点 (实例化版, 与库内 SW3538 / FUSB302 同一套模式)
 * ============================================================================
 *  - **一个 ST7735 对象 = 一块屏**。init 时【全套绑定】:
 *        SPI 外设句柄 · DMA 通道 · CS/DC/RES/EN 四组引脚 · 面板批次
 *    形参全可缺省, 不传就回落到工程 lf_config.h 的 ST7735_* 宏 ——
 *    所以 lcd.init() 的行为与旧的 ST7735_Init() 完全一致。
 *  - 【SPI 句柄可选】: init(spi) 传 nullptr(或不传)则用 ST7735_SPI 宏;
 *    将来接第二块屏(另一路 SPI)只要再建一个对象、传另一路句柄即可。
 *  - 寄存器直调 AT32 BSP (spi / gpio / dma 三类调用), 无平台抽象层; 与 PY32 库
 *    直调 LL_* 同一模式。延时 wk_delay_ms, 时基 lf_get_tick()。
 *  - 面板批次 A/B/C/D 全部内置运行时表, init() 给初值、selectPanel() 可切。
 *
 * ============================================================================
 *  ★ 与旧版的唯一差异: 60 个 extern "C" 全局函数 (ST7735_Init 等) 已删除,
 *    API 只有一套 —— 实例方法。调用方需建实例:
 *        LoveFinderLib::ST7735 lcd;
 *        lcd.init();                      -- 绑 lf_config.h 的宏
 *        lcd.init(SPI1);                  -- 显式绑 SPI1
 * ============================================================================
 *
 *  ⚠ 前置条件: 调用 init() 前, 工程必须先完成对应 SPI 的 wk_xxx_init()
 *    (SPI 时钟未开时, 首字节发送会死等 TDBE 标志)。
 *
 * ============================================================================
 *  工程 lf_config.h 中需要配置的宏 (全部是 init() 的【缺省值】)
 * ============================================================================
 *   ST7735_SPI                        SPI 外设实例 (必须, 如 SPI1)
 *   ST7735_DMA_CHANNEL                DMA 通道 (ST7735_USE_DMA=1 时必须)
 *   ST7735_PANEL_A | _B | _C | _D     面板批次 (默认 A)
 *   ST7735_WIDTH / ST7735_HEIGHT      逻辑分辨率 (默认 160 x 80)
 *   ST7735_ROTATION                   ST7735::DEG_0 / 90 / 180 / 270
 *   ST7735_USE_DMA                    0 | 1  (DMA 用于大块填充/图像)
 *   ST7735_DMA_TIMEOUT_MS             DMA 超时
 *   ST7735_RES_PORT/PIN, ST7735_CS_PORT/PIN,
 *   ST7735_DC_PORT/PIN,  ST7735_EN_PORT/PIN (必须)
 *
 * ============================================================================
 *  用法
 * ============================================================================
 *    LoveFinderLib::ST7735 lcd;
 *    lcd.init();                                  // 全用 lf_config.h 的宏
 *    // lcd.init(SPI1);                          // 只绑 SPI 句柄
 *    // lcd.init(SPI1, DMA1_CHANNEL3, pins, 3);  // 全套 (引脚组 + 面板 D)
 *
 *    lcd.fillScreen(ST7735::RED);                 // 面板自检: 应显示红色
 *    lcd.writeString(4, 4, "Hello", FULLFONT_STY1_7x10,
 *                    ST7735::WHITE, ST7735::BLACK);
 *    lcd.setRotation(ST7735::DEG_90);             // 运行时换方向
 */
#ifndef ST7735_HPP
#define ST7735_HPP

/* 先于工程/BSP 头 —— 它们与 fonts.h 都直接使用 uintN_t/intN_t
 * (原平台层 lf_platform.hpp 隐式提供 <cstdint>, 直绑改写后在此显式给出)。 */
#include <cstdint>
#include <cstddef>
#include <cstdarg>

#include "lf_config.h"      /* ★ 工程侧: 引脚 / SPI / DMA / 面板选择 (作 init 缺省值) */

/* ★ 自带所用外设的 BSP 头 —— 不能只靠 at32f421_conf.h 条件包含
 *   (它是按 *_MODULE_ENABLED 宏展开的, 不用某外设的工程就没有该头,
 *    见库 AGENTS.md §7.8)。本类成员要用 spi/gpio/dma 三个类型。 */
#include "at32f421_spi.h"    /* spi_type */
#include "at32f421_gpio.h"   /* gpio_type / gpio_bits_set / gpio_bits_reset */
#include "at32f421_dma.h"    /* dma_channel_type / dma_channel_enable */

#include "wk_system.h"      /* wk_delay_ms/µs — WorkBench BSP (仿 PY32 的 LL_mDelay) */
#include "font.h"           /* FontDef + 字体名册 + font_get_glyph* (PickSoul 生成) */

/*============================================================================
 * MADCTL 位定义 (全局宏 — 面板表与类内常量都用它拼值)
 *============================================================================*/
#define ST7735_MADCTL_MY    0x80u
#define ST7735_MADCTL_MX    0x40u
#define ST7735_MADCTL_MV    0x20u
#define ST7735_MADCTL_ML    0x10u
#define ST7735_MADCTL_BGR   0x08u
#define ST7735_MADCTL_MH    0x04u
#define ST7735_MADCTL_RGB   0x00u

#if !defined(ST7735_PANEL_A) && !defined(ST7735_PANEL_B) && \
    !defined(ST7735_PANEL_C) && !defined(ST7735_PANEL_D)
#define ST7735_PANEL_A
#endif

/* 只允许选中一个 */
#if (defined(ST7735_PANEL_A) + defined(ST7735_PANEL_B) + \
     defined(ST7735_PANEL_C) + defined(ST7735_PANEL_D)) > 1
#error "只能选择一个 ST7735 面板 (ST7735_PANEL_A/B/C/D)"
#endif

/* 默认批次 —— 仅决定【上电初始】用哪套参数 (四套参数见 st7735.cpp 的 kPanels 表)。 */
#if defined(ST7735_PANEL_B)
#define ST7735_DEFAULT_PANEL_IDX   1
#elif defined(ST7735_PANEL_C)
#define ST7735_DEFAULT_PANEL_IDX   2
#elif defined(ST7735_PANEL_D)
#define ST7735_DEFAULT_PANEL_IDX   3
#else
#define ST7735_DEFAULT_PANEL_IDX   0   /* Panel A (默认) */
#endif

/*============================================================================
 * 编译期缺省配置 (可在 lf_config.h 中覆盖)
 *============================================================================*/
#ifndef ST7735_WIDTH
#define ST7735_WIDTH        160
#endif

#ifndef ST7735_HEIGHT
#define ST7735_HEIGHT       80
#endif

#ifndef ST7735_USE_DMA
#define ST7735_USE_DMA      1
#endif

#ifndef ST7735_DMA_TIMEOUT_MS
#define ST7735_DMA_TIMEOUT_MS  100
#endif

#ifndef ST7735_ROTATION
#define ST7735_ROTATION     ST7735::DEG_0
#endif

/*--------------------------------------------------------------------------
 * 外设与引脚 —— 必须由工程 lf_config.h 提供 (作 init 的缺省值)
 *--------------------------------------------------------------------------*/
#if !defined(ST7735_RES_PORT) || !defined(ST7735_RES_PIN) || \
    !defined(ST7735_CS_PORT)  || !defined(ST7735_CS_PIN)  || \
    !defined(ST7735_DC_PORT)  || !defined(ST7735_DC_PIN)  || \
    !defined(ST7735_SPI)
#error "lf_config.h 必须定义 ST7735_RES/CS/DC_PORT+PIN 与 ST7735_SPI (init 的缺省值)"
#endif

#if (ST7735_USE_DMA == 1) && !defined(ST7735_DMA_CHANNEL)
#error "ST7735_USE_DMA=1 时 lf_config.h 必须定义 ST7735_DMA_CHANNEL (如 DMA1_CHANNEL3)"
#endif

/*============================================================================
 * Gamma 档位 (全局 — 不随实例变化)
 *============================================================================*/
enum class GammaDef : uint8_t {
    GAMMA_10 = 0x01,
    GAMMA_25 = 0x02,
    GAMMA_22 = 0x04,
    GAMMA_18 = 0x08
};

namespace LoveFinderLib {

/*============================================================================
 * ST7735 引脚组 — init() 第 3 参; 留空 = 用 lf_config.h 的 ST7735_*_PORT/PIN
 *
 * ⚠ 定义在类【外】: 嵌套类型的默认成员初始化器 (NSDMI) 不能用作同类
 *   成员函数的默认实参 —— GCC 报 "default member initializer ... required
 *   before the end of its enclosing class"。提到类外即完整类型, 无此限制。
 *============================================================================*/
struct ST7735_PinSet {
    gpio_type *csPort  = nullptr;  uint16_t csPin  = 0;
    gpio_type *dcPort  = nullptr;  uint16_t dcPin  = 0;
    gpio_type *resPort = nullptr;  uint16_t resPin = 0;
    gpio_type *enPort  = nullptr;  uint16_t enPin  = 0;   /* 背光, 可不接 */
};

/*============================================================================
 * ST7735 — 一块屏一个实例
 *============================================================================*/
class ST7735 {
public:
    /*==========================================================================
     * 静态常量 (原 namespace ST7735 的内容, 语义不变)
     *==========================================================================*/
    /*---- 屏幕尺寸 ----*/
    static constexpr uint16_t WIDTH  = ST7735_WIDTH;
    static constexpr uint16_t HEIGHT = ST7735_HEIGHT;

    /*---- 旋转档位 (原 ST7735::Rotation, 枚举名与取值完全不变) ----*/
    enum Rotation : uint8_t {
        DEG_0   = 0,
        DEG_90  = 1,
        DEG_180 = 2,
        DEG_270 = 3
    };

    /*---- 命令 ----*/
    static constexpr uint8_t NOP     = 0x00;
    static constexpr uint8_t SWRESET = 0x01;
    static constexpr uint8_t RDDID   = 0x04;
    static constexpr uint8_t RDDST   = 0x09;
    static constexpr uint8_t SLPIN   = 0x10;
    static constexpr uint8_t SLPOUT  = 0x11;
    static constexpr uint8_t PTLON   = 0x12;
    static constexpr uint8_t NORON   = 0x13;
    static constexpr uint8_t INVOFF  = 0x20;
    static constexpr uint8_t INVON   = 0x21;
    static constexpr uint8_t GAMSET  = 0x26;
    static constexpr uint8_t DISPOFF = 0x28;
    static constexpr uint8_t DISPON  = 0x29;
    static constexpr uint8_t CASET   = 0x2A;
    static constexpr uint8_t RASET   = 0x2B;
    static constexpr uint8_t RAMWR   = 0x2C;
    static constexpr uint8_t RAMRD   = 0x2E;
    static constexpr uint8_t PTLAR   = 0x30;
    static constexpr uint8_t MADCTL  = 0x36;
    static constexpr uint8_t COLMOD  = 0x3A;
    static constexpr uint8_t FRMCTR1 = 0xB1;
    static constexpr uint8_t FRMCTR2 = 0xB2;
    static constexpr uint8_t FRMCTR3 = 0xB3;
    static constexpr uint8_t INVCTR  = 0xB4;
    static constexpr uint8_t DISSET5 = 0xB6;
    static constexpr uint8_t PWCTR1  = 0xC0;
    static constexpr uint8_t PWCTR2  = 0xC1;
    static constexpr uint8_t PWCTR3  = 0xC2;
    static constexpr uint8_t PWCTR4  = 0xC3;
    static constexpr uint8_t PWCTR5  = 0xC4;
    static constexpr uint8_t VMCTR1  = 0xC5;
    static constexpr uint8_t RDID1   = 0xDA;
    static constexpr uint8_t RDID2   = 0xDB;
    static constexpr uint8_t RDID3   = 0xDC;
    static constexpr uint8_t RDID4   = 0xDD;
    static constexpr uint8_t PWCTR6  = 0xFC;
    static constexpr uint8_t GMCTRP1 = 0xE0;
    static constexpr uint8_t GMCTRN1 = 0xE1;

    /*---- RGB565 颜色 ----*/
    static constexpr uint16_t BLACK   = 0x0000;
    static constexpr uint16_t BLUE    = 0x001F;
    static constexpr uint16_t RED     = 0xF800;
    static constexpr uint16_t GREEN   = 0x07E0;
    static constexpr uint16_t CYAN    = 0x07FF;
    static constexpr uint16_t MAGENTA = 0xF81F;
    static constexpr uint16_t YELLOW  = 0xFFE0;
    static constexpr uint16_t WHITE   = 0xFFFF;
    static constexpr uint16_t ORANGE  = 0xFC00;
    static constexpr uint16_t GRAY    = 0x8410;

    [[nodiscard]] static constexpr uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) noexcept
    {
        return static_cast<uint16_t>(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | ((b & 0xF8u) >> 3));
    }

    /*==========================================================================
     * 引脚组 — 见类外的 LoveFinderLib::ST7735_PinSet (默认实参需要完整类型)
     *==========================================================================*/
    using PinSet = ST7735_PinSet;

    /*==========================================================================
     * 构造 / 初始化
     *==========================================================================*/
    ST7735() = default;

    /**
     * @brief 绑定硬件并下发屏初始化序列 (复位 + 命令表 + 面板参数)。
     *
     * 全部形参可缺省, 缺省时回落到 lf_config.h 的对应宏 ——
     * `lcd.init()` 与旧 `ST7735_Init()` 行为完全一致。
     *
     * @param spi      SPI 外设句柄; nullptr = 用 ST7735_SPI (通常 SPI1)
     * @param dma      DMA 通道;     nullptr = 用 ST7735_DMA_CHANNEL
     *                 (ST7735_USE_DMA=0 时此参数被忽略)
     * @param pins     四组控制引脚; 全 nullptr = 用 ST7735_*_PORT/PIN 宏
     * @param panelIdx 面板批次 0..3 = A/B/C/D; 缺省取 lf_config.h 的选择
     *
     * ⚠ 必须先 wk_spi1_init() (SPI 时钟未开时首字节死等 TDBE)。
     */
    void init(spi_type *spi = nullptr,
              dma_channel_type *dma = nullptr,
              const PinSet &pins = PinSet{},
              uint8_t panelIdx = ST7735_DEFAULT_PANEL_IDX) noexcept;

    [[nodiscard]] bool isInitialized() const noexcept { return m_initialized; }

    /*---- 面板自检 (点屏排查用): 依次刷红/绿/蓝/白, 每次间隔 delayMs ----
     * 用于区分「SPI 不通」「颜色序反」「反色反」三种故障。 */
    void panelSelfTest(uint16_t delayMs) noexcept;

    /*---- 基础绘制 ----*/
    void drawPixel(uint16_t x, uint16_t y, uint16_t color) noexcept;
    void fillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) noexcept;
    void fillScreen(uint16_t color) noexcept;
    void drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const uint16_t *data) noexcept;

    /*---- DMA 加速版本 (大块数据; ST7735_USE_DMA=0 时自动退回轮询) ----*/
    void fillRectangleDma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) noexcept;
    void fillScreenDma(uint16_t color) noexcept;
    void drawImageDma(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      const uint16_t *data) noexcept;

    /*---- 文本 ----*/
    void writeChar(uint16_t x, uint16_t y, char ch,
                   const FontDef &font, uint16_t color, uint16_t bgcolor) noexcept;
    void writeString(uint16_t x, uint16_t y, const char *str,
                     const FontDef &font, uint16_t color, uint16_t bgcolor) noexcept;
    void writeStringUtf8(uint16_t x, uint16_t y, const char *str,
                         const FontDef &font, uint16_t color, uint16_t bgcolor) noexcept;
    void print(uint16_t x, uint16_t y, const FontDef &font,
               uint16_t color, uint16_t bgcolor, const char *format, ...);

    /*---- 显示控制 ----*/
    void invertColors(bool invert) noexcept;
    void setGamma(GammaDef gamma) noexcept;
    void displayOn() noexcept  { writeCommand(DISPON); }
    void displayOff() noexcept { writeCommand(DISPOFF); }
    void setBacklight(bool on) noexcept;

    /*---- 旋转 ----*/
    void setRotation(uint8_t rotation) noexcept;   /* ST7735::DEG_0 / 90 / 180 / 270 */
    [[nodiscard]] uint8_t  getRotation() const noexcept { return m_rotation; }
    [[nodiscard]] uint16_t getWidth()  const noexcept { return m_width;  }
    [[nodiscard]] uint16_t getHeight() const noexcept { return m_height; }

    /*---- 面板批次 — 运行时切换 (点屏排障: 无需重编译逐套对比 A/B/C/D) ----*/
    void selectPanel(uint8_t idx) noexcept;         /* 0=A 1=B 2=C 3=D; 随后须重调 init() */
    [[nodiscard]] uint8_t     getPanelIndex() const noexcept { return m_panelIdx; }
    [[nodiscard]] const char *getPanelName()  const noexcept;   /* "PanelA".."PanelD" */

    /*---- 图形原语 ----*/
    void drawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) noexcept;
    void drawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) noexcept;
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) noexcept;
    void drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint8_t r, uint16_t color) noexcept;
    void fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint8_t r, uint16_t color) noexcept;
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) noexcept;
    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) noexcept;
    void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) noexcept;
    void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                      int16_t x2, int16_t y2, uint16_t color) noexcept;
    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                      int16_t x2, int16_t y2, uint16_t color) noexcept;

private:
    /*==========================================================================
     * 硬件绑定 (init 填入; 不传形参时取 lf_config.h 的宏)
     *==========================================================================*/
    spi_type         *m_spi = ST7735_SPI;
    dma_channel_type *m_dma = nullptr;
    gpio_type *m_csPort  = nullptr;  uint16_t m_csPin  = 0;
    gpio_type *m_dcPort  = nullptr;  uint16_t m_dcPin  = 0;
    gpio_type *m_resPort = nullptr;  uint16_t m_resPin = 0;
    gpio_type *m_enPort  = nullptr;  uint16_t m_enPin  = 0;

    /*==========================================================================
     * 运行状态
     *==========================================================================*/
    bool     m_initialized = false;
    bool     m_pinsCfgured = false;
    uint8_t  m_panelIdx    = ST7735_DEFAULT_PANEL_IDX;

    /* 当前旋转档位与其对应参数 (由 applyRotation 按面板表设置) */
    uint8_t  m_rotation = ST7735_ROTATION;
    uint16_t m_width    = WIDTH;
    uint16_t m_height   = HEIGHT;
    uint16_t m_xStart   = 0;
    uint16_t m_yStart   = 0;

#if (ST7735_USE_DMA == 1)
    /* DMA 行缓冲: ST7735 是 16-bit 像素而 DMA 按 byte 传, 展开成大端字节流。
     * ⚠ 每实例一份 (320 B @160 宽), 非线程安全。 */
    static constexpr uint16_t kRowBufPixels = ST7735_WIDTH;
    alignas(4) uint8_t m_rowBuf[kRowBufPixels * 2u] = {};
#endif

    /*==========================================================================
     * 底层原语 (直调 AT32 BSP)
     *==========================================================================*/
    void csLow()  noexcept { gpio_bits_reset(m_csPort, m_csPin); }
    void csHigh() noexcept { gpio_bits_set(m_csPort, m_csPin); }

    void dcCommand() noexcept { gpio_bits_reset(m_dcPort, m_dcPin); }
    void dcData()    noexcept { gpio_bits_set(m_dcPort, m_dcPin); }

    void spiWriteByte(uint8_t b) noexcept;     /* 等 TDBE → 写数据寄存器 */
    void spiWaitIdle() noexcept;               /* 等移位寄存器空, 翻 CS 前必须 */
    void spiWriteBlock(const uint8_t *data, uint16_t len) noexcept;

    void writeCommand(uint8_t cmd) noexcept;
    void writeData(const uint8_t *buff, std::size_t size) noexcept;
    void writeData8(uint8_t v) noexcept { writeData(&v, 1u); }
    void writeData16(const uint16_t *buff, std::size_t count) noexcept;

    void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) noexcept;
    void executeCommandList(const uint8_t *addr) noexcept;
    void resetPanel() noexcept;
    void ensurePins() noexcept;                /* CS/DC/RES 配推挽 + 初始电平 */

    /*---- 旋转参数 ----*/
    struct RotationCfg {
        uint8_t  madctl;
        uint16_t width;
        uint16_t height;
        uint16_t xStart;
        uint16_t yStart;
    };
    [[nodiscard]] RotationCfg rotationCfgOf(uint8_t rot) const noexcept;
    void applyRotation(uint8_t rot) noexcept;

    /*---- 辅助 ----*/
    [[nodiscard]] bool clipRect(uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h) const noexcept;
    void drawGlyph(uint16_t x, uint16_t y, const uint16_t *glyph,
                   uint8_t width, uint8_t height, uint16_t color, uint16_t bgcolor) noexcept;

#if (ST7735_USE_DMA == 1)
    bool dmaTransfer(const uint8_t *data, uint16_t len, uint32_t timeoutMs) noexcept;
#endif

    static void swapI16(int16_t &a, int16_t &b) noexcept { const int16_t t = a; a = b; b = t; }
    void fillSpan(int16_t xa, int16_t ya, int16_t xb, int16_t yb,
                  int16_t xc, int16_t yc, uint16_t color) noexcept;
};

} // namespace LoveFinderLib

/*============================================================================
 * 兼容别名 —— 让 ST7735::WHITE / ST7735::DEG_0 / ST7735::Color565 这类
 * 原 namespace 写法继续可用 (常量已移入类, 限定关系不变)。
 *
 *   using ST7735 = LoveFinderLib::ST7735;
 *
 * 建实例仍推荐带命名空间限定: LoveFinderLib::ST7735 lcd;
 *============================================================================*/
using ST7735 = LoveFinderLib::ST7735;

#endif /* ST7735_HPP */
