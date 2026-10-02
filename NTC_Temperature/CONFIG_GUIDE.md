# NTC_Temperature — 配置引导 (AT32)

NTC 热敏电阻测温库：ADC-DMA 连续采样 + 中值/均值滤波 + B 值公式换算。

## 依赖

- `lf_config.h`（工程侧配置头：电路参数 + ADC/DMA 实例宏）
- 一个配置为**循环模式**的 ADC-DMA 通道（直调 `adc_*` / `dma_*`，
  缓冲由驱动静态持有；实例宏 `NTC_ADC_INSTANCE` / `NTC_DMA_CHANNEL`，
  缺省 `ADC1` / `DMA1_CHANNEL1`）

## 支持两种电路拓扑

由 `NTC_POSITION` 选择：

```
NTC_LOW_SIDE  (NTC 在低侧, 最常见)
    3V3 ──[R_series]──┬── ADC
                      └──[NTC]── GND
    Vadc = Vref · Rntc / (R_series + Rntc)
    → Rntc = R_series · Vadc / (Vref − Vadc)

NTC_HIGH_SIDE (NTC 在高侧)
    3V3 ──[NTC]──┬── ADC
                 └──[R_series]── GND
    Vadc = Vref · R_series / (R_series + Rntc)
    → Rntc = R_series · (Vref − Vadc) / Vadc
```

## 在项目 `lf_config.h` 中配置

```cpp
/* ---- 1. NTC 在高侧还是低侧 ---- */
#define NTC_POSITION            NTC_LOW_SIDE    /* 或 NTC_HIGH_SIDE */

/* ---- 2. NTC 自身参数 ---- */
#define NTC_RESISTANCE          100000.0f       /* 25℃ 标称阻值 (Ω) */
#define NTC_B_VALUE             3950.0f         /* B 值 (K) */
#define NTC_R0_TEMPERATURE_C    25.0f           /* R0 对应温度 (℃) */

/* ---- 3. 串联贴片电阻 (位置由 NTC_POSITION 决定是上拉还是下拉) ---- */
#define NTC_SERIES_RESISTOR     100000.0f       /* (Ω) */

/* ---- 4. 参考电压与 ADC ---- */
#define NTC_VREF_MV             3300.0f         /* 上拉电源 (mV) */
#define NTC_ADC_RESOLUTION      4095.0f         /* 12-bit 满量程 */

/* ---- 5. 滤波与缓冲 ---- */
#define NTC_DMA_BUFFER_SIZE     16              /* DMA 环形缓冲深度 = 滤波窗口 */
#define NTC_INVALID_TEMP_C      (-1000.0f)      /* 无效值哨兵 */
```

> 本项目实例：3V3 —[100K 贴片]— ADC(PA4) —[100K NTC B=3950]— GND
> → `NTC_LOW_SIDE` / `NTC_RESISTANCE=100000` / `NTC_SERIES_RESISTOR=100000` / `NTC_B_VALUE=3950`

## 硬件配置要求

1. ADC 通道对应引脚设为 `GPIO_MODE_ANALOG`（WorkBench 已生成）
2. DMA 通道配置为**循环模式**（`loop_mode_enable = TRUE`），外设→内存、半字宽度
3. 选择实例（缺省即 `ADC1` + `DMA1_CHANNEL1`，可在 `lf_config.h` 覆盖）：

```cpp
/* ---- 可选: 覆盖缺省实例 ---- */
#define NTC_ADC_INSTANCE    ADC1
#define NTC_DMA_CHANNEL     DMA1_CHANNEL1
```

> 无需注入句柄 —— 采样缓冲是驱动内静态 `m_buf[NTC_DMA_BUFFER_SIZE]`，
> `start()` 时装载 DMA 并软触发 ADC。

## 使用

```cpp
#include "NTC_Temperature.hpp"
using namespace LoveFinderLib;

NTC_Temperature ntc;

void setup() {
    ntc.init();                         /* 缺省绑定 ADC1 + DMA1_CH1 */
    ntc.setFilter(e_NTC_Filter::MEDIAN);/* 默认即中值 */
    ntc.start();                        /* 启动连续采样 */
}

void loop() {
    float t = ntc.getTemp();            /* ℃ */
    if (t <= NTC_INVALID_TEMP_C + 1.0f) {
        /* 无有效数据: 断线 / 短路 / DMA 未启动 */
    }
}
```

## 滤波模式

| 模式 | 说明 |
|------|------|
| `e_NTC_Filter::NONE` | 直接取最新一次采样（最快，噪声最大） |
| `e_NTC_Filter::MEAN` | 算术平均（平滑，但会被脉冲拉偏） |
| `e_NTC_Filter::MEDIAN` | **默认**，中值滤波，抗脉冲干扰 |

窗口大小 = `NTC_DMA_BUFFER_SIZE`（内部上限 32 点）。
`readRawADC()` 会先做一次快照再排序，避免 DMA 后台写入造成数据撕裂。

## 静态换算接口（可脱离实例使用）

```cpp
uint16_t adc = 2048;
float t  = NTC_Temperature::adcToCelsius(adc);            // ADC → ℃
float r  = NTC_Temperature::celsiusToResistance(25.0f);   // ℃ → Ω
float t2 = NTC_Temperature::resistanceToCelsius(r);       // Ω → ℃
```

这三个函数是纯函数，**可在 host 侧（PC）直接单元测试**，不依赖硬件。

## 精度（本工程参数实测）

| 项 | 值 |
|----|-----|
| 全量程往返误差（−40 ~ +125 ℃） | ≤ 0.061 ℃ |
| 分辨率（25 ℃ 附近） | 0.022 ℃/LSB |
| 分辨率（−40 ℃ / 125 ℃） | 0.138 / 0.297 ℃/LSB |

> 因上下拉阻值相同（100K/100K），25 ℃ 时 ADC 恰在半量程（~2048），
> 该点 dV/dT 最大、分辨率最好；两端变差属正常物理特性。

## 边界行为

| ADC 原始值 | 结果 |
|-----------|------|
| `0` | 无效（短路到地） |
| `1` | 有效，约 +528 ℃（极端读数，非错误） |
| `4094` | 有效，约 −90 ℃ |
| `4095` | 无效（断线到 Vref） |
