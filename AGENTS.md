# AGENTS.md — LoveFinderLibForAT32 外设驱动库

> 面向 AI 编码智能体 / 后续维护者的库说明。
> **目标平台**: AT32F421 系列 (Cortex-M4F) · **标准**: 纯 C++17

---

## 1. 这是什么

一套**跨工程复用**的 AT32 外设驱动库。与 `LoveFinderLibForPY32_LL` /
`LoveFinderLibForSTM32_HAL` 同构 —— 每个 MCU 家族一份库，驱动直调该家族 BSP，
目录结构 / 配置清单机制在三份库间保持一致。

**核心原则**

1. **库本体不放在工程内**。工程通过 CMake `add_subdirectory` 引用本目录。
2. **无平台抽象层**。驱动头直接 `#include "lf_config.h"`（工程配置头，扮演
   PY32 库参考工程 `gpio.hpp` 的角色）并直调 AT32 BSP（`gpio_*` / `spi_*` /
   `i2c_*` / `dma_*` / `exint_*`）。这是 2026-09-24 彻底对齐 PY32 模式后的形态：
   `lf_platform*.{hpp,cpp}` 已删除，`I2cBus` / `LfPin` / `SpiDmaTx` /
   `AdcDmaChannel` 等抽象类型**不存在**。
3. **项目专属配置不进库**。引脚、外设实例、阻值、地址等全部由工程侧的
   `lf_config.h` 提供；库目录下每个模块给一份 `CONFIG_GUIDE.md` 作引导。
   库内**没有任何 conf 文件**。

---

## 2. 目录结构

```
LoveFinderLibForAT32\
├── CMakeLists.txt              # ★ 库唯一构建入口
├── AGENTS.md                   # ★ 本文件
├── PROJECT_SUMMARY.md          #   交付清单与验证结果
│
├── I2C\                        # 阻塞式硬件 I2C 主机 (i2c_master 自由函数)
├── BUTTON\                     # 按键: 单击 / 双击 / 长按 + 充能 (EXINT 上下沿, ISR 只置标志)
├── TIMER\                      # 可选的 TIM 毫秒时基 (按键脱离 SysTick 时才用)
├── SW3526\                     # SW3526S 快充控制器 (I2C)
├── SW2305\                     # SW2305 PD 协议芯片 (I2C, 5mΩ 检流)
├── SW3538\                     # SW3538 多口快充控制器 (I2C, 读为主 + ADC)
├── FUSB302\                    # FUSB302 Type-C/PD 控制器 (I2C, 一期: 通讯自检)
├── NTC_Temperature\            # NTC 测温 (ADC-DMA, B 值公式)
├── ST7735\                     # ST7735 LCD (SPI 轮询 + DMA)
└── FontLib\                    # 字库 (驱动无关, PickSoul 规范)
```

---

## 3. 分层架构（直绑模式，仿 PY32）

```
┌──────────────────────────────────────────────────────────┐
│ 应用层 (工程侧)  main.cpp / app.cpp / lf_irq.cpp          │
│   → 持有驱动对象, 处理业务逻辑                             │
│   → 提供 lf_config.h (配置) + lf_get_tick() (时基实现)      │
└───────────────────────────┬──────────────────────────────┘
                            │ #include "lf_config.h"
┌───────────────────────────▼──────────────────────────────┐
│ 驱动层 (库)  BUTTON · I2C · SW3526 · SW2305 · FUSB302 · NTC · ST7735│
│   直调 AT32 BSP: gpio_*/spi_*/i2c_*/dma_*/exint_*          │
│   延时 wk_delay_ms (WorkBench) · 时基 lf_get_tick (工程)    │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│ AT32 BSP  at32f421_*.c/h  (WorkBench 生成)                │
└──────────────────────────────────────────────────────────┘
```

### 工程侧契约（库依赖的全部工程符号）

| 符号 | 声明处 | 实现处 | 用途 |
|------|--------|--------|------|
| `lf_get_tick()` | 工程 `lf_config.h` | 工程 `lf_irq.cpp`（SysTick 递增） | I2C 超时 / 按键消抖（默认时基） / LCD DMA 超时（仿 PY32 `BSP_GetTick`） |
| `lf_irq_tick()` | — | 工程 `lf_irq.cpp` | `SysTick_Handler` 每 1ms 调用 |
| 引脚/实例宏 | 工程 `lf_config.h` | — | `ST7735_*` / `BUTTON_KEY_*` / `LF_I2C_BUS0` / `NTC_*` |
| `fonts_config.hpp` | 工程 `project/inc/` | — | 字符清单 X 宏（FontLib 经 `-I` 找到） |

---

## 4. 移植到新工程

### Step 1 — 工程侧建 `lf_config.h`

放在 `project/inc/`，列出所需宏（见各模块 `CONFIG_GUIDE.md`），
并**声明** `uint32_t lf_get_tick(void);`（`extern "C"`）。

### Step 2 — CMake 引用

```cmake
get_filename_component(LOVE_FINDER_LIB_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../LoveFinderLibForAT32" ABSOLUTE)

set(LF_PROJECT_INC_DIR  "${CMAKE_SOURCE_DIR}/project/inc")

add_subdirectory("${LOVE_FINDER_LIB_DIR}" "${CMAKE_BINARY_DIR}/lovefinder_lib")

target_link_libraries(your_target at32_workbench lovefinder_lib)
```

> 前提：宿主工程必须先定义 `at32_workbench` INTERFACE 目标
> （提供 AT32 BSP 头文件路径与 `USE_STDPERIPH_DRIVER` / `AT32F421G8U7` 宏）。
> 库会在缺失时报 `FATAL_ERROR` 并给出提示。
> 工程根 `CMakeLists.txt` 需在 `add_subdirectory` 前设置 `LF_PROJECT_INC_DIR`。

### Step 3 — 时基与中断

```c
/* at32f421_int.c */
void lf_irq_tick(void);
void SysTick_Handler(void) { lf_irq_tick(); }    /* 1ms → lf_get_tick */

/* 用 BUTTON 时, 对应 EXINT 分组中断里调: */
void EXINT1_0_IRQHandler(void) { /* exint 已由 onExti 清标志 */ key.onExti(); }
```

工程 `lf_irq.cpp` 实现 `lf_get_tick()` 读取一个由 `lf_irq_tick()` 递增的
`volatile uint32_t`（见 891 工程 `project/src/lf_irq.cpp` 范例）。

### Step 4 — 启动顺序约束

```cpp
wk_spi1_init();   /* ★ 必须在 ST7735_Init() 之前 (SPI 时钟未开时首字节死等 TDBE) */
...
ST7735_Init();
```

**没有 bind/init 注入步骤** —— 外设实例全部来自 `lf_config.h` 宏。

---

## 5. 各模块要点

### BUTTON — 按键

```cpp
LoveFinderLib::Button key;
key.init(BUTTON_KEY_PORT, BUTTON_KEY_PIN);   /* 直调 gpio_/scfg_/exint_/nvic_ */
/* EXINT 中: key.onExti();   主循环 5ms: switch(key.update()) {...} */
```

EXINT 配置（输入+上拉 → SCFG 线绑定 → 触发沿 → NVIC 分组中断）已内联在
`BUTTON.cpp`。状态机：`IDLE → DEBOUNCE → PRESSED → WAIT_CLICK → IDLE`。
长按触发 `LONG_PRESS` 一次，并开始 0~100 充能（`getChargePercent()`）。
**工程必须定义对应 EXINT 分组中断并在其中调 `onExti()`**（否则按键触发即进
弱定义死循环）。

**两段式中断模型（改这个驱动时必须守住的边界）**：

- `onExti()` **只置标志** —— 清挂起位 → 读一次电平辨认「按下沿 / 释放沿」
  → 记 `buttonNow()` 时间戳。不做消抖、不改状态机。
- `update()` 在主循环里做全部判定，标志没来时用电平轮询兜底。
- EXINT 默认 `EXINT_TRIGGER_BOTH_EDGE`（上下沿都触发，`bothEdges` 可关回单沿）。
  按下沿 → `m_pressPending`（唤醒状态机）；释放沿 → `m_releasePending`
  （起**释放消抖窗**：电平稳定 `debounceMs` 才提交 `RELEASE`，回弹则取消）。
- 时基走 `buttonNow()`：默认 `lf_get_tick()`；`BUTTON_TIMEBASE_TIMER=1` 时改读
  TIMER 模块的 `timerNow()`（起失败自动退回 `lf_get_tick()`）。
- 只有 `app_nav` 这类消费方**不受影响**：`RELEASE` 会比旧版晚约 `debounceMs`。

### TIMER — 可选的 TIM 毫秒时基

```cpp
LoveFinderLib::TIMER_Config tc;
tc.tmr = TMR6; tc.clkHz = 120000000; tc.periodMs = 10;
bool ok = LoveFinderLib::timerTickStart(tc);     /* 分频不成立 → false */
extern "C" void TMR6_GLOBAL_IRQHandler(void) { LoveFinderLib::timerTickIsr(); }
/* 工程自己 init 的 TIM: 中断里只调 timerTickAdd(periodMs), 标志自己清 */
uint32_t ms = LoveFinderLib::timerNow();
```

**默认不用**（`BUTTON_TIMEBASE_TIMER=0`）：工程本就有 SysTick 1ms 的
`lf_get_tick()`，分辨率更细、且 I2C/ST7735 超时也用它 —— 系统留一个时间轴更好。
只在「要脱离 SysTick / 复用自己的一路 TIM」时启用。

要点：
- 分频按 `{1MHz,100kHz,10kHz,1kHz}` 逐档选**能整除** `clkHz` 的，
  实际中断周期与 `periodMs` 严格相等、无累积误差；凑不出则返回 `false`。
- 只能占**空闲** TIM（会 `tmr_base_init()` 覆盖）；AT32F421 七个定时器
  `TMR1/3/6/14/15/16/17` 全部支持（`mapTmr()` 自动配 CRM 时钟 + NVIC 中断号），
  TMR6 首选、TMR14~17 备选，TMR3 常被 WorkBench 的 LED PWM 占用。
- **WorkBench 预生成这路 TIM 时**：WK 只配时钟+分频（参数见 `TIMER/CONFIG_GUIDE.md`），
  **不要在 WK 打开它的中断/NVIC** —— 否则 `at32f421_int.c` 会生成强符号 handler，
  与工程 `lf_irq.cpp` 的 `BUTTON_TIM_IRQ` 重复定义，链接报错。
- handler 必须是**强符号**且名字与实例配套（`BUTTON_TIM_IRQ` 宏），
  否则第一次溢出就进弱定义死循环。
- **自带** `at32f421_tmr.h`/`_crm.h`/`_misc.h` 三个 BSP 头 —— 不能只靠
  `at32f421_conf.h`，它是按 `TMR_MODULE_ENABLED` 等宏**条件包含**的，
  不用定时器的工程（907）根本没有 `tmr_type`（2026-10-02 实测踩到）。
  同理 BUTTON.hpp/.cpp 也自带 `at32f421_gpio.h`/`_exint.h`/`_scfg.h`/`_misc.h`。

### I2C — 阻塞式硬件 I2C 主机

```cpp
using namespace LoveFinderLib;
i2cMemWrite(I2C1, 0x3C, reg, &v, 1, 100);
std::optional<uint8_t> v = i2cReadReg8(I2C1, 0x3C, reg, 100);
```

自由函数收 `i2c_type*`，轮询标志 + `lf_get_tick()` 超时；`memRead` 用
repeated START（规范寄存器读时序）。**软件 Bitbang 后端已随平台层删除**
（无工程使用；将来需要时在 `I2C/` 下新增实现）。
`i2cMasterRecover()` 只做外设软复位，GPIO 级 9 脉冲恢复由应用层自行实现。

### SW3526 — 快充控制器

**不含 I2C 实现**，init 时传入 I2C 外设实例：

```cpp
LoveFinderLib::SW3526 ch1;   ch1.init(LF_I2C_BUS0, 0x3C, "CH1");  /* I2C1 */
```

地址用 **7-bit** 形式（SW3526S 为 `0x3C`）。电流测量按检流电阻缩放：
`setSenseResistor(mΩ)`。

### SW2305 — PD 协议芯片

依据 **SW2305_REG V1.2**。两个关键点：

- **写保护**：写 `0x14` / `0xA0~0xBF` 前须 `unlockForConfig()`；
  写 `0x16` 前须 `unlockForForceCtrl()`。都是 `0x20→0x40→0x80` 三写序列。
- **ADC**：写 `0x3B` 选通道 → 数据锁存到 `0x3C/0x3D`。
  电流 `3.125mA/bit` 是 **10mΩ 基准**，5mΩ 需 ×2
  （`setSenseResistor(5)` 已自动处理）。

### SW3538 — 多口快充控制器（读为主）

依据 **SW3538_寄存器列表 Release_RG108_3_v1.2**，地址与 SW3526S 同为 `0x3C`
（**目前只有 208 用它**；907/POT6 用的是 SW2305 + SW3526S + FUSB302，不涉及）。

- **通路使能**：`init(bus, addr, name, ports)` 第 4 参声明这颗芯片接了哪些口 ——
  `e_SW3538_Port::PATH1`(Type-C) / `PATH2`(Type-A) / `BOTH`(缺省)。
  没接的口 `readMeasurement()` 不读电流、状态位恒 false（否则 UI 会误报
  "通路2 有设备"）。**一条 I2C 总线上只能挂一颗**（现仅 0x3C 一个地址），
  多颗必须分总线；厂家将来放出新地址只改 `addr7`，实例化方式不变。
- **VBUS ≠ VOUT**（屏显要的是 VBUS）：`vout_mv` = BUCK 输出（ADC 通道 11，
  空载约 5.1V）；`vbus_mv` = 经 NMOS 之后口上的电压，按 PD 规范没接 SINK 时
  为 0V。**手册没有独立 VBUS ADC 通道** → 驱动用【通路导通状态】门控 VOUT
  得到（导通 ≈ VOUT、关断 = 0）。判据在 `SW3538.cpp` 的 VBUS 段
  （`conducting`），上板若恒 5.1V / 恒 0V 就换成 `path1Online` 或
  `buckOn && path1On`。功率仍按 `vout × iout`，不受该判据影响。
- **写寄存器**要先走 `Reg0x10 = 0x20→0x40→0x80` 解锁序列（`writeReg()`
  内部自动带）；写 `Reg0x16~0x19` 额外要 `Reg0x15` 序列 → `writeRegForce()`。
- **VIN ADC 默认关闭**，上电须 `enableAdcSampling()`（`Reg0x18 = 0xE0`），
  否则输入电压永远读 0。**通路2电流在 `Reg0x18` 的哪个 bit 无手册依据**
  （0xE0 只覆盖 VOUT/VIN/通路1电流），要接 Type-A 口时先查手册再扩这一步。
- **ADC 锁存式**：写 `Reg0x40` 选通道 → 读 `Reg0x41/0x42`
  （`value = (0x42[6:0]<<8)|0x41`）。通道 1=通路1电流 2.5mA/bit@5mΩ、
  2=通路2电流、5=输出电压 6mV、6=输入电压 10mV、7=NTC、
  11=单位转换输出电压 1mV（**没有 VBUS 通道**）。
- `readMeasurement()` 一次拿齐：快充状态(0x09)+系统状态(0x0A)+
  设备在线(0x0D)+三次 ADC，再算出 `vbus_mv`。

```cpp
using LoveFinderLib::e_SW3538_Port;
LoveFinderLib::SW3538 ch1;
ch1.init(LF_I2C_BUS0, 0x3C, "IIC1", e_SW3538_Port::PATH1);   /* POT1: 只接 Type-C */
if (ch1.probe()) { ch1.enableAdcSampling(); ch1.readMeasurement(m); }
/* 屏显电压用 m.vbus_mv；m.vout_mv 留作功率口径与调试 */
```

### FUSB302 — Type-C/PD 控制器（一期：通讯自检）

**不含 I2C 实现**，init 时传入 I2C 外设实例（7-bit 地址固定 `0x22`）：

```cpp
LoveFinderLib::FUSB302 ct;   ct.init(LF_I2C_BUS1);   /* I2C2 */
if (ct.probe()) { ... }      /* 读 Device ID (0x01): 应答 且 != 0x00/0xFF */
```

寄存器枚举 `e_FUSB302_Reg` 已按手册全量就位（DEVICE_ID…FIFOs），CC / PD
功能在 `readReg/writeReg` 上扩展。注意 0x44/0x45 是 **8-bit** 地址形式，
库里一律填 7-bit。

### NTC_Temperature — 测温

四宏配置，改 `lf_config.h` 即可适配任意 NTC 电路：

```cpp
#define NTC_POSITION        NTC_LOW_SIDE   // 或 NTC_HIGH_SIDE
#define NTC_RESISTANCE      100000.0f      // NTC 25℃ 阻值
#define NTC_SERIES_RESISTOR 100000.0f      // 串联贴片电阻
#define NTC_B_VALUE         3950.0f        // B 值
```

```cpp
ntc.init();          // 缺省 NTC_ADC_INSTANCE(ADC1) / NTC_DMA_CHANNEL(DMA1_CH1)
ntc.start();         // 驱动静态缓冲 m_buf[] + 软触发 ADC
float t = ntc.getTemp();      // ℃, 无效返回 NTC_INVALID_TEMP_C
```

滤波：`NONE` / `MEAN` / `MEDIAN`（默认中值，抗脉冲）。
静态换算函数 `adcToCelsius()` / `resistanceToCelsius()` / `celsiusToResistance()`
为纯函数，逻辑可核验。

### ST7735 — LCD

尺寸/偏移/旋转/颜色顺序全部参数化，面板表在 `st7735.hpp` 内
（工程用 `ST7735_PANEL_A/B/C/D` 选择）。硬件访问直调：

- SPI 字节：`spi_i2s_flag_get(SPI_I2S_TDBE_FLAG)` → `spi_i2s_data_transmit`，
  翻 CS 前等 `SPI_I2S_BF_FLAG` 清零
- GPIO：`gpio_bits_set/reset(ST7735_CS/DC/RES_PORT, …)`
- DMA：轮询 `ST7735_DMA_CHANNEL->dtcnt == 0` + `lf_get_tick()` 超时
  （**工程 NVIC 未开 DMA 中断**，与 PY32 一样纯轮询；超时关通道退回轮询发送）
- 延时：`wk_delay_ms`

```cpp
ST7735_Init();
ST7735_FillScreen(ST7735::BLACK);
ST7735_WriteString(2, 2, "CH1 20.0V", Font_Subset_7x10,
                   ST7735::WHITE, ST7735::BLACK);
```

DMA 版本（`*_DMA`）使用行缓冲分块搬运 —— ST7735 是 16-bit 像素而 DMA 按
byte 传输，需先展开成大端字节流。DMA 失败自动退回轮询。

### FontLib — 字库

**驱动无关**，遵循 `LoveFinder830-PickSoul` 规范。双模式：

| 模式 | 文件 | 说明 |
|------|------|------|
| A 完整字库 | `fonts.c` | `data != nullptr`，固定索引 `(ch-32)*height` |
| B 字符级子集 | `fonts_subset.cpp` | `data == nullptr`，走 `font_get_glyph()` 查表 |

字模格式：每字符 = `height` 个 `uint16_t`，**bit15 = 最左列**。

编译哪些**字体**由 `USE_FONT_*` 开关控制；编译哪些**字符**由**工程侧**
`fonts_config.hpp` 的 `FONT_SUBSET_*_CHARS` X 宏清单决定（库内不带该文件，
`#include "fonts_config.hpp"` 经 include path 解析到工程副本）：

```c
/* 工程侧: 只要数字的工程就只列数字 */
#define FONT_SUBSET_7X10_CHARS(CP) CP(32, space) CP(48, 0) CP(49, 1) /* ... */
```

库的 `fonts_subset.cpp` 持有全部字形（每字符一个 `static` 数组），`switch`
只展开清单里的项，未被引用的数组由 `--gc-sections` 丢弃、不占 Flash。
所以「只要数字」和「要大量中文」的工程共用同一份库、**互不干扰**
（891 实测：清单只留数字时 FLASH 25 268 B → 23 492 B，map 中未列字形地址为
`0x00000000`，见 `FontLib/README.md`）。

> ⚠ 清单宏的形参名不要用 `X`（与字形名大写 `X` 冲突），本库统一用 `CP`。

---

## 6. 修改约定

| 要改什么 | 改哪里 |
|----------|--------|
| 某个驱动的行为 | 对应模块目录下的 `.hpp` / `.cpp` |
| BSP 调用差异 / 新 MCU 家族 | 本家族库内驱动源码（每家族一份库：AT32/PY32/STM32） |
| 引脚 / 外设实例 / 阻值 / 地址 | **工程侧** `lf_config.h`（不要改库） |
| 编译哪些字符（子集字库） | **工程侧** `fonts_config.hpp` 的 `FONT_SUBSET_*_CHARS` 清单 |
| 新增模块 | 建目录 + 写 `.hpp/.cpp` + 在 `CMakeLists.txt` 的 `LF_LIB_SOURCES` 注册 |

### 代码规范

- 命名空间 `LoveFinderLib`（`lf_get_tick()` 等工程符号在全局命名空间，
  `extern "C"` 链接）
- 文件扩展名 `.hpp` / `.cpp`（C 数据文件用 `.c`）
- 用 `std::optional` 表达可能失败、`std::uint8_t` 等定宽类型
- 不用 RTTI / 异常（编译选项已关）；不用 `std::function` 除非必要
- 所有可能失败的 I2C 操作返回 `bool` 或 `std::optional`
- **注释里不要写 `*/` 形式的通配连写**（如 `LL_SPI_*/X` 会提前终止注释，
  曾造成级联编译错误；写 `LL_SPI_ / X` 或拆词）

---

## 7. 已知约束

1. **`lf_config.h` 必须可见**。库经 `-I ${LF_PROJECT_INC_DIR}` 找到它；
   工程没提供时各驱动头会 `#error`（ST7735 检查引脚/外设宏）或找不到符号。
2. **驱动直绑后不能脱离工程头做 host 侧单测** —— 需要自测时给
   `-fsyntax-only` 传工程 `inc` 路径（或把纯算法函数保持为静态纯函数）。
3. **ST7735_Init() 前必须 wk_spi1_init()** —— SPI 时钟未开时 TDBE 永不置位，
   首字节发送死等。
4. **DMA 无中断依赖** —— `wk_dma.c` 未开 DMA NVIC，`DMA1_Channel3_2_IRQHandler`
   为弱定义；驱动完成判定用 `dtcnt` 轮询 + `lf_get_tick()` 超时。
   `g_lcd_dma_done` / `lf_irq_lcd_dma` 仅作预留挂点。
5. **`i2cMasterRecover()` 只做外设软复位**，不做 GPIO 级 9 脉冲恢复。
   需要时由应用层自行实现（依赖具体引脚）。
6. **ST7735 的 `*_DMA` 使用 static 行缓冲**（`ST7735_WIDTH * 2` 字节），
   非线程安全，且要求 `ST7735_WIDTH` 与 `ST7735_HEIGHT` 编译期已知。
7. **WorkBench 重新生成的风险** —— `.ATWP` 里 SPI 仍配 HALF_DUPLEX/30MHz，
   工程 `main.cpp` 在 `wk_spi1_init()` 后以 C++ 断言 FULL_DUPLEX + DIV_8 对冲。
8. **`at32f421_conf.h` 是按 `*_MODULE_ENABLED` 宏条件包含外设头的** ——
   某工程没用某外设，该 `at32f421_*.h` 就不会被包含。**模块要用哪些 BSP 头，
   自己在 `.hpp/.cpp` 里列明**（TIMER 用 tmr/crm/misc，BUTTON 用 gpio/exint/scfg/misc），
   不要指望 conf.h 顺带带出来，否则换个工程就编不过。

---

## 8. 构建验证

```powershell
$tc  = "C:\Users\ZWDX_LKJ\AppData\Local\at32-tools"
& "$tc\ninja\V1.11.1\ninja.exe" -C <工程>\build
# 零错误零警告; 尺寸看链接输出 / arm-none-eabi-size <elf>
```

当前引用本库的工程：`LoveFinder891-SC8102_Charger`、`LoveFinder907-PowerOneT6_MuiltiCharger`
（POT6）、`LoveFinder208-PowerOneT1_Dual140W_Charger`。
改库后三个工程都要编一遍（2026-10-02 改 BUTTON/TIMER 即按此验证）。

---

*最后更新: 2026-10-02*（新增 TIMER 模块 + BUTTON 双沿/纯标志中断/可选时基；
SW3538 加通路使能 init 形参与 VBUS 推导）
