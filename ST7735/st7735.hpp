/**
 * @file    st7735.hpp
 * @brief   ST7735 LCD 驱动 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计要点 (仿 LoveFinderLibForPY32_LL: 驱动直绑工程头 + AT32 BSP, 无平台抽象层)
 * ============================================================================
 *  - 驱动【直接操作 AT32 BSP】: SPI 用 ST7735_SPI (SPI1) 直收发, GPIO 用
 *    ST7735_*_PORT/PIN 直推挽, DMA 用 ST7735_DMA_CHANNEL 轮询完成 ——
 *    与 PY32 库 st7735.cpp 直调 LL_SPI_ / LL_GPIO_ 系列函数同一模式。
 *  - 外设 / 引脚 / 面板全部来自【工程侧】lf_config.h (仿 PY32 的 gpio.hpp)。
 *  - 延时用 WorkBench BSP 的 wk_delay_ms (仿 PY32 的 LL_mDelay);
 *    DMA/超时时基用工程 lf_get_tick() (仿 PY32 的 BSP_GetTick)。
 *  - **面板批次可切换**: 颜色序 / 反色极性 / RAM 偏移 由本头文件内的
 *    面板表集中管理 (Panel A/B/C/D), 工程侧 lf_config.h 用 ST7735_PANEL_A/B/C/D 选择。
 *  - **四档旋转**, 每档自带正确的 MADCTL 与偏移 (参考 LoveFinder491)。
 *  - 字模来自 FontLib (PickSoul 规范, 驱动无关)。
 *
 *  ⚠ 前置条件: 调用 ST7735_Init() 前, 工程必须先完成 wk_spi1_init()
 *    (SPI 时钟未开时, 首字节发送会死等 TDBE 标志)。
 *
 * ============================================================================
 *  工程 lf_config.h 中需要配置的宏
 * ============================================================================
 *   ST7735_SPI                        SPI 外设实例 (必须, 如 SPI1)
 *   ST7735_DMA_CHANNEL                DMA 通道 (ST7735_USE_DMA=1 时必须)
 *   ST7735_PANEL_A | _B | _C | _D     面板批次 (默认 A)
 *   ST7735_WIDTH / ST7735_HEIGHT      逻辑分辨率 (默认 160 x 80)
 *   ST7735_ROTATION                   ST7735::DEG_0 / DEG_90 / DEG_180 / DEG_270
 *   ST7735_USE_DMA                    0 | 1  (DMA 用于大块填充/图像)
 *   ST7735_DMA_TIMEOUT_MS             DMA 超时
 *   ST7735_RES_PORT/PIN, ST7735_CS_PORT/PIN,
 *   ST7735_DC_PORT/PIN,  ST7735_EN_PORT/PIN (必须)
 *
 * ============================================================================
 *  用法
 * ============================================================================
 *    ST7735_Init();
 *    ST7735_FillScreen(ST7735::RED);        // 面板自检: 应显示红色
 *    ST7735_WriteString(4, 4, "Hello", FULLFONT_STY1_7x10,
 *                       ST7735::WHITE, ST7735::BLACK);
 *    ST7735_SetRotation(ST7735::DEG_90);    // 运行时换方向
 */
#ifndef ST7735_HPP
#define ST7735_HPP

/* 先于工程/BSP 头 —— 它们与 fonts.h 都直接使用 uintN_t/intN_t
 * (原平台层 lf_platform.hpp 隐式提供 <cstdint>, 直绑改写后在此显式给出)。 */
#include <cstdint>
#include <cstddef>
#include <cstdarg>

#include "lf_config.h"      /* ★ 工程侧: 引脚 / SPI / DMA / 面板选择 (仿 PY32 的 gpio.hpp) */
#include "wk_system.h"      /* wk_delay_ms/µs — WorkBench BSP (仿 PY32 的 LL_mDelay) */
#include "font.h"           /* FontDef + 字体名册 + font_get_glyph* (PickSoul 生成) */

/*============================================================================
 * MADCTL 位定义 —— 必须在 PanelConfig 之前 (面板表用它们拼值)
 *============================================================================*/
namespace ST7735 {

constexpr uint8_t MADCTL_MY  = 0x80;
constexpr uint8_t MADCTL_MX  = 0x40;
constexpr uint8_t MADCTL_MV  = 0x20;
constexpr uint8_t MADCTL_ML  = 0x10;
constexpr uint8_t MADCTL_BGR = 0x08;
constexpr uint8_t MADCTL_MH  = 0x04;
constexpr uint8_t MADCTL_RGB = 0x00;

} // namespace ST7735

/* MADCTL 位定义 (面板表拼值用)。
 * 四套面板参数 (A/B/C/D) 已【全部】内置 st7735.cpp 的运行时表 kPanels[4]:
 *   - 上电默认批次由【工程侧】lf_config.h 定义且只定义一个 ST7735_PANEL_A/B/C/D
 *     决定 (映射为 ST7735_DEFAULT_PANEL_IDX);
 *   - 运行期可 ST7735_SelectPanel(0..3) 切换, 供点屏排障逐套对比 (无需重编译)。 */
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

namespace ST7735 {

/*============================================================================
 * 旋转档位
 *============================================================================*/
enum Rotation : uint8_t {
    DEG_0   = 0,
    DEG_90  = 1,
    DEG_180 = 2,
    DEG_270 = 3
};

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
 * 外设与引脚 —— 必须由工程 lf_config.h 提供 (仿 PY32: 由工程 gpio.hpp 提供)
 *--------------------------------------------------------------------------*/
#if !defined(ST7735_RES_PORT) || !defined(ST7735_RES_PIN) || \
    !defined(ST7735_CS_PORT)  || !defined(ST7735_CS_PIN)  || \
    !defined(ST7735_DC_PORT)  || !defined(ST7735_DC_PIN)  || \
    !defined(ST7735_SPI)
#error "lf_config.h 必须定义 ST7735_RES/CS/DC_PORT+PIN 与 ST7735_SPI (直绑 AT32 BSP)"
#endif

#if (ST7735_USE_DMA == 1) && !defined(ST7735_DMA_CHANNEL)
#error "ST7735_USE_DMA=1 时 lf_config.h 必须定义 ST7735_DMA_CHANNEL (如 DMA1_CHANNEL3)"
#endif

/*============================================================================
 * 屏幕参数
 *============================================================================*/
constexpr uint16_t WIDTH  = ST7735_WIDTH;
constexpr uint16_t HEIGHT = ST7735_HEIGHT;

/* 面板批次名 / 反色极性 —— 随运行时选择变化, 用 ST7735_GetPanelName() 等查询 (见下)。 */

/*============================================================================
 * 命令
 *============================================================================*/
constexpr uint8_t NOP     = 0x00;
constexpr uint8_t SWRESET = 0x01;
constexpr uint8_t RDDID   = 0x04;
constexpr uint8_t RDDST   = 0x09;
constexpr uint8_t SLPIN   = 0x10;
constexpr uint8_t SLPOUT  = 0x11;
constexpr uint8_t PTLON   = 0x12;
constexpr uint8_t NORON   = 0x13;
constexpr uint8_t INVOFF  = 0x20;
constexpr uint8_t INVON   = 0x21;
constexpr uint8_t GAMSET  = 0x26;
constexpr uint8_t DISPOFF = 0x28;
constexpr uint8_t DISPON  = 0x29;
constexpr uint8_t CASET   = 0x2A;
constexpr uint8_t RASET   = 0x2B;
constexpr uint8_t RAMWR   = 0x2C;
constexpr uint8_t RAMRD   = 0x2E;
constexpr uint8_t PTLAR   = 0x30;
constexpr uint8_t MADCTL  = 0x36;
constexpr uint8_t COLMOD  = 0x3A;
constexpr uint8_t FRMCTR1 = 0xB1;
constexpr uint8_t FRMCTR2 = 0xB2;
constexpr uint8_t FRMCTR3 = 0xB3;
constexpr uint8_t INVCTR  = 0xB4;
constexpr uint8_t DISSET5 = 0xB6;
constexpr uint8_t PWCTR1  = 0xC0;
constexpr uint8_t PWCTR2  = 0xC1;
constexpr uint8_t PWCTR3  = 0xC2;
constexpr uint8_t PWCTR4  = 0xC3;
constexpr uint8_t PWCTR5  = 0xC4;
constexpr uint8_t VMCTR1  = 0xC5;
constexpr uint8_t RDID1   = 0xDA;
constexpr uint8_t RDID2   = 0xDB;
constexpr uint8_t RDID3   = 0xDC;
constexpr uint8_t RDID4   = 0xDD;
constexpr uint8_t PWCTR6  = 0xFC;
constexpr uint8_t GMCTRP1 = 0xE0;
constexpr uint8_t GMCTRN1 = 0xE1;

/*============================================================================
 * RGB565 颜色
 *============================================================================*/
constexpr uint16_t BLACK   = 0x0000;
constexpr uint16_t BLUE    = 0x001F;
constexpr uint16_t RED     = 0xF800;
constexpr uint16_t GREEN   = 0x07E0;
constexpr uint16_t CYAN    = 0x07FF;
constexpr uint16_t MAGENTA = 0xF81F;
constexpr uint16_t YELLOW  = 0xFFE0;
constexpr uint16_t WHITE   = 0xFFFF;
constexpr uint16_t ORANGE  = 0xFC00;
constexpr uint16_t GRAY    = 0x8410;

[[nodiscard]] constexpr uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) noexcept
{
    return static_cast<uint16_t>(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | ((b & 0xF8u) >> 3));
}

} // namespace ST7735

/*============================================================================
 * Gamma 预设
 *============================================================================*/
enum class GammaDef : uint8_t {
    GAMMA_10 = 0x01,
    GAMMA_25 = 0x02,
    GAMMA_22 = 0x04,
    GAMMA_18 = 0x08
};

/*============================================================================
 * 公开 API
 *============================================================================*/
extern "C" {

/** 初始化: 复位 + 下发初始化命令表 + 应用面板参数。 */
void ST7735_Init();

/*---- 面板自检 (点屏排查用) ----*/
/**
 * @brief 依次刷红/绿/蓝/白，每次间隔 delayMs。
 * 用于区分「SPI 不通」「颜色序反」「反色反」三种故障。
 */
void ST7735_PanelSelfTest(uint16_t delayMs);

/*---- 基础绘制 ----*/
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

/*---- DMA 加速版本 (大块数据) ----*/
void ST7735_FillRectangle_DMA(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_FillScreen_DMA(uint16_t color);
void ST7735_DrawImage_DMA(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

/*---- 文本 ----*/
void ST7735_WriteChar(uint16_t x, uint16_t y, char ch,
                      const FontDef &font, uint16_t color, uint16_t bgcolor);
void ST7735_WriteString(uint16_t x, uint16_t y, const char *str,
                        const FontDef &font, uint16_t color, uint16_t bgcolor);
void ST7735_WriteStringUTF8(uint16_t x, uint16_t y, const char *str,
                            const FontDef &font, uint16_t color, uint16_t bgcolor);
void ST7735_Print(uint16_t x, uint16_t y, const FontDef &font,
                  uint16_t color, uint16_t bgcolor, const char *format, ...);

/*---- 显示控制 ----*/
void ST7735_InvertColors(bool invert);
void ST7735_SetGamma(GammaDef gamma);
void ST7735_DisplayOn();
void ST7735_DisplayOff();
void ST7735_SetBacklight(bool on);

/*---- 旋转 ----*/
void ST7735_SetRotation(uint8_t rotation);      /* ST7735::DEG_0 / 90 / 180 / 270 */
uint8_t ST7735_GetRotation();
uint16_t ST7735_GetWidth();
uint16_t ST7735_GetHeight();

/*---- 面板批次 — 运行时切换 (点屏排障: 无需重编译逐套对比 A/B/C/D) ----*/
void ST7735_SelectPanel(uint8_t idx);            /* 0=A 1=B 2=C 3=D; 随后须重调 ST7735_Init() */
uint8_t ST7735_GetPanelIndex();
const char* ST7735_GetPanelName();               /* "PanelA".."PanelD" */

/*---- 图形原语 ----*/
void ST7735_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7735_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color);
void ST7735_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color);
void ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7735_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

} // extern "C"

#endif /* ST7735_HPP */
