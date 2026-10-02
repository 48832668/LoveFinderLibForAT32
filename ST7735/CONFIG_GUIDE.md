# ST7735 — 配置引导 (AT32)

ST7735 LCD 驱动：SPI 轮询 + DMA 加速，含完整图形原语与文本渲染。
**面板批次可切换**（颜色序 / 反色极性 / RAM 偏移），**四档旋转**。

---

## 依赖

- 工程侧 `lf_config.h`（`ST7735_SPI` / `ST7735_DMA_CHANNEL` / CS·DC·RES 引脚宏）
- 时基 `lf_get_tick()`（DMA 超时）与 WorkBench `wk_delay_ms()`（初始化延时）
- FontLib（`fonts.h` + `fonts_subset.h`）
- **前置**：调用 `ST7735_Init()` 前工程必须先 `wk_spi1_init()`

---

## 1. 面板批次（点屏最关键的一项）

ST7735 只是控制器型号，**不同批次的模组在颜色序 / 反色 / RAM 偏移上并不一致**。
同一份驱动，参数不对就会出现「背光亮但全黑 / 全白 / 颜色反相」。

在项目 `lf_config.h` 中定义 **且只定义一个**：

```cpp
#define ST7735_PANEL_A      // 原批次  BGR, 偏移 0/24, 不反色
// #define ST7735_PANEL_B   // 新批次  RGB, 偏移 1/26, 需反色
// #define ST7735_PANEL_C   // 新批次水平镜像
// #define ST7735_PANEL_D   // A 的变体 BGR, 偏移 1/26, 需反色
```

未定义任何一个时默认 **Panel A**。

| 面板 | 颜色序 | 偏移 | 反色 | 适用 |
|------|--------|------|------|------|
| A | BGR | 0/24 | 否 | 原批次（491/907/891 当前默认） |
| B | RGB | 1/26 | **是** | 2025-2026 新批次 |
| C | BGR | 1/26 | **是** | 新批次水平镜像 |
| D | BGR | 1/26 | **是** | A 的电气特性但需反色 |

---

## 2. 点屏排查（重要）

**不要靠猜面板批次**——先用 `ST7735_PanelSelfTest()` 做颜色自检：

```cpp
ST7735_Init();
ST7735_PanelSelfTest(800);   // 依次刷 红 → 绿 → 蓝 → 白, 每个 800ms
```

根据肉眼看到的现象判断：

| 现象 | 结论 | 处理 |
|------|------|------|
| 依次红绿蓝白 | **完全正常** | 成功 |
| 红显示成青/蓝绿 | 面板需要反色 | 换 `ST7735_PANEL_B` 或 `_D` |
| 红显示成蓝 | R/B 通道颠倒 | 颜色序配反了（A↔B 互换） |
| 画面整体偏移 | RAM 偏移不对 | 换到偏移 1/26 的面板 |
| 画面上下/左右颠倒 | 旋转档位不对 | 换 `ST7735_ROTATION` |
| **始终全黑/全白** | **SPI 数据没到屏上** | 查焊接 / SCK / MOSI / CS / 时钟 |

> 「全黑/全白、毫无变化」几乎可以断定是**硬件或 SPI 通路**问题，而非面板参数——
> 因为红色在颜色序错和反色错的情况下**都仍然可见**。
> 优先排查：① 排线虚焊 ② CS/DC 引脚 ③ SPI 时钟 ④ 背光与屏供电是否独立。

**另一种快速验证**：全屏纯红。

```cpp
ST7735_Init();
ST7735_FillScreen(ST7735::RED);
```

---

## 3. 其余配置（项目 `lf_config.h`）

```cpp
/* ---- 分辨率 (逻辑尺寸, 通常不改) ---- */
#define ST7735_WIDTH            160
#define ST7735_HEIGHT           80

/* ---- 旋转 ---- */
#define ST7735_ROTATION         ST7735::DEG_0    /* DEG_0/90/180/270 */

/* ---- DMA ---- */
#define ST7735_USE_DMA          1                /* 0 = 全部走轮询 */
#define ST7735_DMA_TIMEOUT_MS   100

/* ---- 外设 (必须; 缺失时 st7735.hpp 直接 #error) ---- */
#define ST7735_SPI              SPI1
#define ST7735_DMA_CHANNEL      DMA1_CHANNEL3    /* USE_DMA=1 时必须 */

/* ---- 引脚 (必须; 宏值通常取自 at32f421_wk_config.h) ---- */
#define ST7735_RES_PORT         LCD_RESET_GPIO_PORT
#define ST7735_RES_PIN          LCD_RESET_PIN
#define ST7735_CS_PORT          LCD_CS_GPIO_PORT
#define ST7735_CS_PIN           LCD_CS_PIN
#define ST7735_DC_PORT          LCD_DC_GPIO_PORT
#define ST7735_DC_PIN           LCD_DC_PIN
/* 可选: 背光/供电使能 */
#define ST7735_EN_PORT          LCD_EN_GPIO_PORT
#define ST7735_EN_PIN           LCD_EN_PIN
```

> `ST7735_ROTATION` 的值在 `st7735.hpp` 的命名空间内定义，
> 所以宏值需写成 `ST7735::DEG_0` 这种形式。

---

## 4. 硬件配置要求

1. SPI：主机、8-bit、MSB first、软件 CS，**半双工仅发送或全双工均可**；
   时钟分频不小于 `SPI_MCLK_DIV_8`（≤15MHz，ST7735 写时序上限）
   - WorkBench (`.ATWP`) 原生配置是 `Simplex_Bidirectional_Master` +
     `SPI_TRANSMIT_HALF_DUPLEX_TX` —— 对只写屏、无 MISO 的 ST7735 是
     **正确模式**（半双工单线走 PA7/MOSI）。唯一必须覆盖的是时钟：
     WorkBench 默认 30MHz(DIV_4) 超标，需改 `DIV_8`(15MHz)。
2. （可选）`ST7735_USE_DMA=1` 时，SPI TX DMA 通道配置为内存→外设、
   **字节**宽度、**非循环模式**；`=0`（当前工程）则全部走 `spiWriteByte`
   轮询，无需 DMA
   - AT32F421：`DMA1_CHANNEL3` 固定映射为 `SPI1_TX`
3. **无需 DMA 中断**：完成判定轮询 `dtcnt == 0` + `lf_get_tick()` 超时
   （工程 NVIC 未开 DMA 中断，弱定义 handler 无碍；
   可选把 `lf_irq_lcd_dma()` 挂到 `DMA1_Channel3_2_IRQHandler` 置
   `g_lcd_dma_done`，但驱动不依赖它）
4. **无需注入句柄**：`ST7735_SPI` / `ST7735_DMA_CHANNEL` / 引脚宏全部来自
   工程 `lf_config.h`，`ST7735_Init()` 内部自配 CS/DC/RES 推挽输出

---

## 5. 使用

```cpp
#include "st7735.hpp"
#include "fonts_subset.h"

ST7735_Init();
ST7735_FillScreen(ST7735::BLACK);

/* 文本 (模式 B 字符级子集, 只把用到的字符编进固件) */
ST7735_WriteString(2, 2, "Hello", Font_Subset_7x10,
                   ST7735::WHITE, ST7735::BLACK);

/* 中文 (UTF-8) */
ST7735_WriteStringUTF8(20, 40, "\xE4\xBD\xA0\xE5\xA5\xBD",
                       Font_Subset_ZH_16x16, ST7735::YELLOW, ST7735::BLACK);

/* 格式化 */
ST7735_Print(2, 20, Font_Subset_7x10, ST7735::GREEN, ST7735::BLACK,
             "%u mA", current);

/* 图形 */
ST7735_DrawRect(0, 0, 160, 80, ST7735::CYAN);
ST7735_FillCircle(80, 40, 20, ST7735::RED);
ST7735_DrawLine(0, 0, 159, 79, ST7735::WHITE);

/* 运行时换方向 */
ST7735_SetRotation(ST7735::DEG_90);
```

---

## 6. API 一览

| 分类 | 函数 |
|------|------|
| 初始化 | `ST7735_Init()` |
| **面板自检** | `ST7735_PanelSelfTest(delayMs)` |
| 基础绘制 | `DrawPixel` `FillRectangle` `FillScreen` `DrawImage` |
| DMA 加速 | `FillRectangle_DMA` `FillScreen_DMA` `DrawImage_DMA` |
| 文本 | `WriteChar` `WriteString` `WriteStringUTF8` `Print` |
| 显示控制 | `InvertColors` `SetGamma` `DisplayOn` `DisplayOff` `SetBacklight` |
| 旋转 | `SetRotation` `GetRotation` `GetWidth` `GetHeight` |
| 图形原语 | `DrawHLine` `DrawVLine` `DrawRect` `DrawRoundRect` `FillRoundRect` `DrawLine` `DrawCircle` `FillCircle` `DrawTriangle` `FillTriangle` |

**颜色**：`BLACK / BLUE / RED / GREEN / CYAN / MAGENTA / YELLOW / WHITE / ORANGE / GRAY`
以及 `constexpr Color565(r,g,b)`（可作编译期常量）。

**面板批次（运行时可切）**：`ST7735_SelectPanel(0..3)` 切 A/B/C/D、
`ST7735_GetPanelIndex()`、`ST7735_GetPanelName()`（"PanelA".."PanelD"）。
四套参数内置 `st7735.cpp` 的 `kPanels[4]`；上电默认批次由
`lf_config.h` 的 `ST7735_PANEL_A/B/C/D` 决定（映射 `ST7735_DEFAULT_PANEL_IDX`）。
切换后须重调 `ST7735_Init()` 才生效。

---

## 7. 初始化时序（与参考工程一致）

```
select → reset(低5ms→高) → cmds1 → MADCTL → cmds2 → cmds3 → INVON/INVOFF → deselect
```

注意 **MADCTL 必须在 cmds1 之后、cmds2 之前**发送——
这是 `LoveFinder491_PowerOneT2` 与 `stm32-st7735` 的共同顺序，
cmds1 内部的 `INVOFF` 不会覆盖 MADCTL。

---

## 8. 实现说明

- **DMA 路径**：ST7735 是 16-bit 像素而 DMA 按 byte 传输，因此先用
  `static uint8_t g_rowBuf[ST7735_WIDTH * 2]` 行缓冲把 RGB565 展开成大端
  字节流，再分块交给 DMA。DMA 失败自动退回轮询，功能不丢。
  > 行缓冲是 `static`，**非线程安全**，且要求 `ST7735_WIDTH` 编译期已知。
- **轮询路径**：`spiWriteByte` 等 `TDBE` 置位后写数据寄存器，
  `spiWaitIdle` 等 `BF` 清零，切换 DC/CS 前确保移位完成——
  这是阻塞模式能稳定点亮的关键。
- `ST7735_USE_DMA=0` 时，`*_DMA` 函数直接退化为对应轮询版本。

---

## 9. 参考来源

| 资源 | 路径 |
|------|------|
| 面板体系 + 旋转表 | `C:\Debug\Self\LoveFinder491_PowerOneT2\LoveFinderLib\ST7735\` |
| 初始化命令表 | `C:\Debug\gitRepo\stm32-st7735\st7735\st7735.c` |
| Adafruit 原始实现 | `C:\Debug\gitRepo\Adafruit-ST7735-Library\` |
