# LoveFinderLibForAT32 — AT32 外设驱动库

**目标平台**: AT32F421 系列 (Cortex-M4F)
**标准**: 纯 C++17（不做向下兼容）
**协议**: MIT

> 📖 **详细使用说明见 [`AGENTS.md`](AGENTS.md)** —— 架构、移植步骤、各模块要点、修改约定。
> 📋 交付清单与验证结果见 [`PROJECT_SUMMARY.md`](PROJECT_SUMMARY.md)。

---

## 概述

一套**跨工程复用**的 AT32 外设驱动库。与 `LoveFinderLibForPY32_LL` /
`LoveFinderLibForSTM32_HAL` 同构：**每个 MCU 家族一份库**，驱动直调该家族 BSP，
API 与目录结构在三份库之间保持一致。

**三条核心原则**

1. **库本体不放在工程内** —— 工程通过 CMake `add_subdirectory` 引用本目录
2. **驱动直绑 AT32 BSP，无平台抽象层** —— 驱动头直接 `#include "lf_config.h"`
   （工程侧配置头，扮演 PY32 库参考工程 `gpio.hpp` 的角色），调用
   `gpio_*` / `spi_*` / `i2c_*` / `dma_*` 等 BSP 函数；时基由工程提供
   `lf_get_tick()`，延时用 WorkBench 的 `wk_delay_ms()`（仿 PY32 的 `BSP_GetTick` / `LL_mDelay`）
3. **项目专属配置不进库** —— 引脚、外设实例、阻值、地址全部由工程侧 `lf_config.h` 提供，
   库目录下**没有任何 conf 文件**

---

## 目录结构

```
LoveFinderLibForAT32/
├── CMakeLists.txt              # ★ 库唯一构建入口
├── AGENTS.md                   # ★ 使用说明
├── PROJECT_SUMMARY.md          #   交付清单与验证结果
│
├── I2C/                        # 阻塞式硬件 I2C 主机 (i2c_master, 自由函数)
├── BUTTON/                     # 按键：单击 / 双击 / 长按 + 充能
├── SW3526/                     # SW3526S 快充控制器 (I2C)
├── SW2305/                     # SW2305 PD 协议芯片 (I2C, 5mΩ 检流)
├── FUSB302/                    # FUSB302 Type-C/PD 控制器 (I2C, 一期: 通讯自检)
├── NTC_Temperature/            # NTC 测温 (ADC-DMA, B 值公式)
├── ST7735/                     # ST7735 LCD (SPI 轮询 + DMA)
└── FontLib/                    # 字库 (驱动无关, PickSoul 规范)
```

每个模块目录下都有 **`CONFIG_GUIDE.md`**，列出它读取的全部宏定义与用法示例。

---

## 快速开始

### 1. 工程侧建 `lf_config.h`

放在工程的 `project/inc/`（这是**唯一**的项目侧配置点）：

```cpp
#ifndef LF_CONFIG_H
#define LF_CONFIG_H

#include <stdint.h>              /* lf_get_tick 声明用 */
#include "at32f421_wk_config.h"  /* AT32 BSP + WorkBench 引脚宏 */

/* ---- ST7735 (直绑外设与引脚, 缺一不可) ---- */
#define ST7735_WIDTH        160
#define ST7735_HEIGHT       80
#define ST7735_PANEL_A
#define ST7735_USE_DMA      1
#define ST7735_DMA_TIMEOUT_MS 100
#define ST7735_SPI          SPI1
#define ST7735_DMA_CHANNEL  DMA1_CHANNEL3
#define ST7735_RES_PORT     LCD1_RESET_GPIO_PORT   /* + _PIN */
#define ST7735_CS_PORT      LCD1_CS_GPIO_PORT      /* + _PIN */
#define ST7735_DC_PORT      LCD_DC_GPIO_PORT       /* + _PIN */

/* ---- I2C 实例 (SW3526/SW2305/FUSB302 init 时传入) ---- */
#define LF_I2C_BUS0         I2C1

/* ---- NTC ---- */
#define NTC_POSITION        NTC_LOW_SIDE
#define NTC_RESISTANCE      100000.0f
#define NTC_SERIES_RESISTOR 100000.0f
#define NTC_B_VALUE         3950.0f
#define NTC_VREF_MV         3300.0f

/* ---- 工程时基 (库驱动超时/消抖用, 仿 PY32 的 BSP_GetTick) ---- */
#ifdef __cplusplus
extern "C" {
#endif
uint32_t lf_get_tick(void);
#ifdef __cplusplus
}
#endif
#endif /* LF_CONFIG_H */
```

### 2. 工程侧实现时基 (`lf_irq.cpp`)

```cpp
static volatile uint32_t g_lfTickMs = 0;
uint32_t lf_get_tick(void) { return g_lfTickMs; }

extern "C" void lf_irq_tick(void) { ++g_lfTickMs; }   /* SysTick 1ms 调用 */
```

### 3. CMake 引用

```cmake
# 推荐: 库与本工程同级, 用相对路径 —— 整体搬迁后无需改动
get_filename_component(LOVE_FINDER_LIB_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../LoveFinderLibForAT32" ABSOLUTE)

set(LF_PROJECT_INC_DIR  "${CMAKE_SOURCE_DIR}/project/inc")

add_subdirectory("${LOVE_FINDER_LIB_DIR}" "${CMAKE_BINARY_DIR}/lovefinder_lib")

target_link_libraries(your_target at32_workbench lovefinder_lib)
```

> 前提：宿主工程必须先定义 `at32_workbench` INTERFACE 目标
> （提供 AT32 BSP 头文件路径与 `USE_STDPERIPH_DRIVER` / `AT32F421G8U7` 宏）。

### 4. 挂接 SysTick 与中断

```c
/* at32f421_int.c */
void lf_irq_tick(void);          /* SysTick 1ms → 递增 lf_get_tick 时基 */
void lf_irq_lcd_dma(void);       /* 可选: LCD DMA 完成 (驱动当前轮询, 不依赖) */

void SysTick_Handler(void)            { lf_irq_tick(); }
void EXINT1_0_IRQHandler(void)        { key.onExti(); }   /* 按键 EXINT */
```

> **无需任何 bind/init 注入** —— 外设实例全部来自 `lf_config.h` 的宏。

---

## 直绑接口一览

| 用途 | 机制 | 工程侧提供 |
|------|------|-----------|
| SPI (LCD) | `ST7735_SPI` + `ST7735_DMA_CHANNEL` 直收发 | 引脚宏 + `wk_spi1_init()` 先行 |
| I2C (快充) | `i2c_*` 自由函数收 `i2c_type*` | `LF_I2C_BUS0` 等实例宏 |
| GPIO/EXINT (按键) | `gpio_*` / `exint_*` / `scfg_*` 直调 | `BUTTON_KEY_PORT/PIN`（`init(port, pin)` 也可显式传） |
| ADC-DMA (NTC) | `adc_*` / `dma_*` 直调 + 驱动静态缓冲 | `NTC_ADC_INSTANCE` / `NTC_DMA_CHANNEL`（缺省 ADC1/DMA1_CHANNEL1） |
| 时基 | `lf_get_tick()` | `lf_config.h` 声明 + `lf_irq.cpp` 实现 |
| 毫秒延时 | `wk_delay_ms()` (WorkBench BSP) | 无（BSP 自带） |

---

## 模块速查

```cpp
/* ---- 按键 ---- */
Button key;
key.init(BUTTON_KEY_PORT, BUTTON_KEY_PIN);
/* EXINT 中: key.onExti();   主循环 5ms: switch(key.update()) {...} */

/* ---- SW3526S ---- */
SW3526 ch1;  ch1.init(LF_I2C_BUS0, 0x3C, "CH1");   // 实例来自 lf_config.h
uint16_t mv = ch1.getVout();

/* ---- SW2305 ---- */
SW2305 ch3;  ch3.init(LF_I2C_BUS0, 0x28, "CH3");
ch3.setSenseResistor(5);            // 5mΩ
uint16_t ich = ch3.getIch();        // 已按检流电阻缩放

/* ---- FUSB302 (一期: 通讯自检) ---- */
FUSB302 ct;  ct.init(LF_I2C_BUS1);  // 缺省 7-bit 地址 0x22
bool ok = ct.probe();               // 读 Device ID (0x01) 应答即在线

/* ---- NTC 测温 (缺省即 ADC1 + DMA1_CH1) ---- */
NTC_Temperature ntc;
ntc.init();
ntc.start();
float t = ntc.getTemp();            // ℃

/* ---- LCD (前提: wk_spi1_init() 已完成) ---- */
ST7735_Init();
ST7735_WriteString(2, 2, "CH1 20.0V", Font_Subset_7x10,
                   ST7735::WHITE, ST7735::BLACK);
```

---

## 字库 (FontLib) — 每工程独立的字符清单

编译哪些**字符**由**工程侧** `project/inc/fonts_config.hpp` 的 X 宏清单决定：

```c
#define FONT_SUBSET_7X10_CHARS(CP) CP(48, 0) CP(49, 1) /* ... 只要数字就只列数字 */
```

库持有全部字形，`switch` 只展开清单项，未引用的数组被 `--gc-sections` 丢弃、
不占 Flash → 多个工程共用同一份库、**字符集互不干扰**（已实测，见 `FontLib/README.md`）。

---

## 移植到其他 MCU

本系列按 **每个 MCU 家族一份库** 组织（`ForAT32` / `ForPY32_LL` / `ForSTM32_HAL`，
目录与 API 结构一致）。移植即把驱动里的 BSP 调用换成目标家族的等价物
（`gpio_*` → `LL_GPIO_*` → `HAL_GPIO_*`），配置头与清单机制原样沿用。
时基函数名是各库的移植锚点：本库 `lf_get_tick()` ↔ PY32 `BSP_GetTick()`。

---

## 构建验证

```powershell
# 完整工程构建 (库经 add_subdirectory 进工程, lf_config.h 由工程 inc 目录提供)
& "C:\Users\ZWDX_LKJ\AppData\Local\at32-tools\ninja\V1.11.1\ninja.exe" -C build
```

> 单文件 `-fsyntax-only` 检查需自带工程 `lf_config.h` 路径
> （驱动直绑后不再能脱离工程头做 host 侧自测）。
