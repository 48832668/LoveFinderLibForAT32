# ST7735 — 配置引导 (AT32)

ST7735 LCD 驱动：SPI 轮询 + DMA 加速，含完整图形原语与文本渲染。
**面板批次可切换**（颜色序 / 反色极性 / RAM 偏移），**四档旋转**。

> **★ 本驱动是【一个对象 = 一块屏】的实例化 API**（与 SW3538 / FUSB302 同一模式）。
> 旧的 60 个 `extern "C"` 全局函数（`ST7735_Init()` / `ST7735_WriteString()` …）
> **已全部删除**，API 只剩实例方法一套 —— 调用方必须先建实例。

---

## 依赖

- 工程侧 `lf_config.h`（`ST7735_SPI` / `ST7735_DMA_CHANNEL` / CS·DC·RES 引脚宏 ——
  全部作为 `init()` 的**缺省值**，不传形参就用它们）
- 时基 `lf_get_tick()`（DMA 超时）与 WorkBench `wk_delay_ms()`（初始化延时）
- FontLib（`fonts.h` + `fonts_subset.h`）
- **前置**：调用 `lcd.init()` 前工程必须先 `wk_spi1_init()`
- BSP 头 `at32f421_spi.h` / `at32f421_gpio.h` / `at32f421_dma.h` ——
  **由 `st7735.hpp` 自己 include**（不能指望 `at32f421_conf.h` 条件带出，
  见库 `AGENTS.md` §7.8）

---

## 1. 面板批次（点屏最关键的一项）

ST7735 只是控制器型号，**不同批次的模组在颜色序 / 反色 / RAM 偏移上并不一致**。
同一份驱动，参数不对就会出现「背光亮但全黑 / 全白 / 颜色反相」。

**两种指定方式**（都有效，二选一或组合）：

```cpp
/* A. 编译期缺省 (工程 lf_config.h), 定义且只定义一个 —— 决定 init() 的第 4 参缺省 */
#define ST7735_PANEL_D
// #define ST7735_PANEL_A    // 原批次  BGR, 偏移 0/24, 不反色
// #define ST7735_PANEL_B    // 新批次  RGB, 偏移 1/26, 需反色
// #define ST7735_PANEL_C    // 新批次水平镜像

/* B. 运行期指定 (不必重编译): init 第 4 参, 或 selectPanel(0..3) 后重调 init() */
lcd.init(nullptr, nullptr, LoveFinderLib::ST7735_PinSet{}, 3);   /* 3 = Panel D */
```

未定义任何一个时默认 **Panel A**。

| 面板 | 颜色序 | 偏移 | 反色 | idx | 适用 |
|------|--------|------|------|-----|------|
| A | BGR | 0/24 | 否 | 0 | 原批次（491/907/891 当前默认） |
| B | RGB | 1/26 | **是** | 1 | 2025-2026 新批次 |
| C | BGR | 1/26 | **是** | 2 | 新批次水平镜像 |
| D | BGR | 1/26 | **是** | 3 | A 的电气特性但需反色 |

---

## 2. 点屏排查（重要）

**不要靠猜面板批次**——先用 `panelSelfTest()` 做颜色自检：

```cpp
LoveFinderLib::ST7735 lcd;
lcd.init();
lcd.panelSelfTest(800);   // 依次刷 红 → 绿 → 蓝 → 白, 每个 800ms
```

根据肉眼看到的现象判断：

| 现象 | 结论 | 处理 |
|------|------|------|
| 依次红绿蓝白 | **完全正常** | 成功 |
| 红显示成青/蓝绿 | 面板需要反色 | 换 `ST7735_PANEL_B` 或 `_D` |
| 红显示成蓝 | R/B 通道颠倒 | 颜色序配反了（A↔B 互换） |
| 画面整体偏移 | RAM 偏移不对 | 换到偏移 1/26 的面板 |
| 画面上下/左右颠倒 | 旋转档位不对 | 换 `ST7735_ROTATION` 或 `setRotation()` |
| **始终全黑/全白** | **SPI 数据没到屏上** | 查焊接 / SCK / MOSI / CS / 时钟 |

> 「全黑/全白、毫无变化」几乎可以断定是**硬件或 SPI 通路**问题，而非面板参数——
> 因为红色在颜色序错和反色错的情况下**都仍然可见**。
> 优先排查：① 排线虚焊 ② CS/DC 引脚 ③ SPI 时钟 ④ 背光与屏供电是否独立。

**另一种快速验证**：全屏纯红。

```cpp
lcd.init();
lcd.fillScreen(LoveFinderLib::ST7735::RED);
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

> `ST7735_ROTATION` 的值必须写成 `ST7735::DEG_0` 这种形式。
> 常量（`DEG_0` / `RED` / `MADCTL` …）现在是**类的静态成员**，
> `st7735.hpp` 尾部有一条全局别名 `using ST7735 = LoveFinderLib::ST7735;`
> —— 所以 `ST7735::DEG_0` 这类旧写法**完全不用改**，且展开点在类内时
> 也能正确解析到类自身。

**这些宏现在只是 `init()` 的缺省值**：不想用宏时，全套引脚/SPI/DMA 都可以
通过形参传入（见 §5），宏仍然必须定义（缺失即 `#error`）。

---

## 4. 硬件配置要求

1. SPI：主机、8-bit、MSB first、软件 CS，**半双工仅发送或全双工均可**；
   时钟分频不小于 `SPI_MCLK_DIV_8`（≤15MHz，ST7735 写时序上限）
   - WorkBench (`.ATWP`) 原生配置是 `Simplex_Bidirectional_Master` +
     `SPI_TRANSMIT_HALF_DUPLEX_TX` —— 对只写屏、无 MISO 的 ST7735 是
     **正确模式**（半双工单线走 PA7/MOSI）。唯一必须覆盖的是时钟：
     WorkBench 默认 30MHz(DIV_4) 超标，需改 `DIV_8`(15MHz)。
2. （可选）`ST7735_USE_DMA=1` 时，SPI TX DMA 通道配置为内存→外设、
   **字节**宽度、**非循环模式**；`=0` 则全部走 `spiWriteByte` 轮询
   - AT32F421：`DMA1_CHANNEL3` 固定映射为 `SPI1_TX`
3. **无需 DMA 中断**：完成判定轮询 `dtcnt == 0` + `lf_get_tick()` 超时
   （工程 NVIC 未开 DMA 中断，弱定义 handler 无碍）
4. **总线绑定是可选的**：`init()` 不传参 = 本对象自建一条 `SpiBus` 并绑
   `lf_config.h` 的 `ST7735_SPI`；传 `&spi1` 则挂到外部共用总线
   （多设备场景，见库 `SPI/spi_bus.hpp`）
5. `init()` 内部自配 CS/DC/RES 推挽输出与初始电平（设备控制脚，非总线引脚）
6. **CS 走 `SpiCsLock`（RAII）**：每个绘制操作 `selectCs()` 拉低、析构等空 +
   拉高；CS 会自动 `addDevice()` 登记到所用总线

---

## 5. 使用

```cpp
#include "st7735.hpp"
#include "fonts_subset.h"

namespace app { inline LoveFinderLib::ST7735 g_lcd; }   /* 一屏一实例 */

/* ---- 初始化: 三种绑定力度 ---- */
g_lcd.init();                                  /* 全用 lf_config.h 的宏 (= 旧行为) */
g_lcd.init(&spi1);                             /* 挂到共用总线 (多设备场景) */
g_lcd.init(&spi1, DMA1_CHANNEL3, pins, 3);     /* 全套: 总线 + DMA + 引脚 + Panel D */
/*                           ↑ LoveFinderLib::ST7735_PinSet{} 可缺省 */

g_lcd.setBacklight(true);

/* ---- 绘制 ---- */
g_lcd.fillScreen(LoveFinderLib::ST7735::BLACK);

/* 文本 (模式 B 字符级子集, 只把用到的字符编进固件) */
g_lcd.writeString(2, 2, "Hello", Font_Subset_7x10,
                  LoveFinderLib::ST7735::WHITE, LoveFinderLib::ST7735::BLACK);

/* 中文 (UTF-8) */
g_lcd.writeStringUtf8(20, 40, "\xE4\xBD\xA0\xE5\xA5\xBD",
                      Font_Subset_ZH_16x16, ST7735::YELLOW, ST7735::BLACK);

/* 格式化 */
g_lcd.print(2, 20, Font_Subset_7x10, ST7735::GREEN, ST7735::BLACK,
            "%u mA", current);

/* 图形 */
g_lcd.drawRect(0, 0, 160, 80, ST7735::CYAN);
g_lcd.fillCircle(80, 40, 20, ST7735::RED);
g_lcd.drawLine(0, 0, 159, 79, ST7735::WHITE);

/* 运行时换方向 / 换面板 */
g_lcd.setRotation(ST7735::DEG_90);
g_lcd.selectPanel(1);   /* B → 随后 g_lcd.init() 才生效 */
```

---

## 6. API 一览

| 分类 | 成员函数 |
|------|----------|
| 初始化 | `init(bus, dma, pins, panelIdx)` · `isInitialized()` |
| **面板自检** | `panelSelfTest(delayMs)` |
| 基础绘制 | `drawPixel` `fillRectangle` `fillScreen` `drawImage` |
| DMA 加速 | `fillRectangleDma` `fillScreenDma` `drawImageDma` |
| 文本 | `writeChar` `writeString` `writeStringUtf8` `print` |
| 显示控制 | `invertColors` `setGamma` `displayOn` `displayOff` `setBacklight` |
| 旋转 | `setRotation` `getRotation` `getWidth` `getHeight` |
| 面板 | `selectPanel` `getPanelIndex` `getPanelName` |
| 图形原语 | `drawHLine` `drawVLine` `drawRect` `drawRoundRect` `fillRoundRect` `drawLine` `drawCircle` `fillCircle` `drawTriangle` `fillTriangle` |

**颜色**：`ST7735::BLACK / BLUE / RED / GREEN / CYAN / MAGENTA / YELLOW / WHITE /
ORANGE / GRAY`，以及 `constexpr ST7735::Color565(r,g,b)`（可作编译期常量）。
写法与旧版完全一致（全局别名保证）。

**面板批次（运行时可切）**：`selectPanel(0..3)` 切 A/B/C/D、`getPanelIndex()`、
`getPanelName()`（"PanelA".."PanelD"）。四套参数内置 `st7735.cpp` 的
`kPanels[4]`（只读，所有实例共享）；上电默认批次由 `lf_config.h` 的
`ST7735_PANEL_A/B/C/D` 决定（映射 `ST7735_DEFAULT_PANEL_IDX`）。
切换后须重调 `init()` 才生效。

**旧 API → 新 API 对照**（迁移时逐条替换）：

| 旧（已删除） | 新 |
|---|---|
| `ST7735_Init()` | `lcd.init()` |
| `ST7735_FillScreen(c)` | `lcd.fillScreen(c)` |
| `ST7735_FillRectangle_DMA(...)` | `lcd.fillRectangleDma(...)` |
| `ST7735_DrawImage_DMA(...)` | `lcd.drawImageDma(...)` |
| `ST7735_WriteString(...)` | `lcd.writeString(...)` |
| `ST7735_DrawPixel(...)` | `lcd.drawPixel(...)` |
| `ST7735_SetBacklight(b)` | `lcd.setBacklight(b)` |
| `ST7735_SelectPanel(n)` | `lcd.selectPanel(n)` |

---

## 7. 初始化时序（与参考工程一致）

```
select → reset(低120ms→高120ms) → cmds1 → MADCTL → cmds2 → cmds3 → INVON/INVOFF → deselect
```

- **MADCTL 必须在 cmds1 之后、cmds2 之前**发送——这是
  `LoveFinder491_PowerOneT2` 与 `stm32-st7735` 的共同顺序，
  cmds1 内部的 `INVOFF` 不会覆盖 MADCTL。
- 复位低/高电平各 **120ms**：早期版本用 5ms，复位不彻底 → 旧 GRAM 残留，
  切换配置时新旧文字重叠（现象像「清屏不生效」）。

---

## 8. 实现说明

- **状态全部是成员**：`m_rotation` / `m_width` / `m_height` / `m_xStart` /
  `m_yStart` / `m_panelIdx` / `m_initialized` / 硬件句柄与引脚。
  只读的面板表与初始化命令表放在匿名 namespace，**所有实例共享**。
  （旧版是文件级 `g_*` 全局 —— 两块屏会互相覆盖旋转/窗口状态。）
- **DMA 路径**：ST7735 是 16-bit 像素而 DMA 按 byte 传输，因此用
  `m_rowBuf[ST7735_WIDTH * 2]` 行缓冲把 RGB565 展开成大端字节流，
  再分块交给 DMA。DMA 失败自动退回轮询，功能不丢。
  > 行缓冲是**每实例一份**（160 宽 = 320 B RAM），同一实例非线程安全；
  > 要求 `ST7735_WIDTH` 编译期已知。
- **轮询路径**：`spiWriteByte` 等 `TDBE` 置位后写数据寄存器，
  `spiWaitIdle` 等 `BF` 清零，切换 DC/CS 前确保移位完成——
  这是阻塞模式能稳定点亮的关键。
- **大地址窗口（不切块）**：`fillRectangle` 一次开出完整目标窗口（整屏即 160x80）
  连续发数据，**不按 48x12 分块**。
  > **分批实测结论**：*旧批次*模组「单个地址窗口 >48x12 必花屏」——证据链
  > （1x1 正常 / 48x12 正常 / 160x80 必花、连续发与逐行发都花、降速无效）
  > 保留在 `st7735.cpp` 源码注释里，当时的对策是切 48x12 小块。
  > **2026-10-02 换的新批次模组**复测：整屏单窗口**完全正常**（开屏页 160x80
  > 背景走 `fillScreenDma`，上板验证通过）。
  > ⇒ 该限制属**模组批次相关**，不是驱动或 SPI 时序问题。若碰到旧批次屏复现
  > 花屏，从 `git` 取回 commit `25b3a95` 的分块实现即可。
  > 208 渲染管线仍按 48x12 推屏 —— 那是**脏块粒度**（`g_dirty` 位图 +
  > `g_tileBuf` 1152B + 气泡 mask），与窗口上限解耦。
- `ST7735_USE_DMA=0` 时，`*Dma` 函数直接退化为对应轮询版本。

---

## 9. 参考来源

| 资源 | 路径 |
|------|------|
| 面板体系 + 旋转表 | `C:\Debug\Self\LoveFinder491_PowerOneT2\LoveFinderLib\ST7735\` |
| 初始化命令表 | `C:\Debug\gitRepo\stm32-st7735\st7735\st7735.c` |
| Adafruit 原始实现 | `C:\Debug\gitRepo\Adafruit-ST7735-Library\` |
