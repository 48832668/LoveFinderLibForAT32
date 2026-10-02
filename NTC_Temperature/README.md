# NTC_Temperature — NTC 热敏电阻测温库 (C++17)

## 概述

ADC-DMA 连续采样 + 中值/均值滤波 + B 值公式换算。
支持 NTC 在**低侧**或**高侧**两种电路拓扑。

> 完整配置说明见 [`CONFIG_GUIDE.md`](CONFIG_GUIDE.md)。

## 温度计算公式

使用 B 值公式（Steinhart-Hart 的简化形式）：

### NTC 低侧（NTC 在地侧，上拉到 Vref）

```
Vadc  = Vref · Rntc / (R_series + Rntc)
Rntc  = R_series · Vadc / (Vref − Vadc)
1/T   = 1/T0 + (1/B) · ln(Rntc / R0)
T(°C) = 1/T − 273.15
```

### NTC 高侧（NTC 在 Vref 侧，下拉到 GND）

```
Vadc  = Vref · R_series / (R_series + Rntc)
Rntc  = R_series · (Vref − Vadc) / Vadc
```

## 文件结构

| 文件 | 说明 |
|------|------|
| `NTC_Temperature.hpp` | 类定义 + 静态换算函数（全部内联，直绑 ADC/DMA 实例） |
| `NTC_Temperature.cpp` | 占位实现（预留查表法扩展） |
| `CONFIG_GUIDE.md` | 配置引导 |
| `README.md` | 本文档 |

## 依赖

- `lf_config.h` — 工程侧配置头（电路参数 + `NTC_ADC_INSTANCE` / `NTC_DMA_CHANNEL`）

## 四宏配置

改 `lf_config.h` 即可适配任意 NTC 电路：

```cpp
#define NTC_POSITION            NTC_LOW_SIDE   /* 1. 高侧还是低侧 */
#define NTC_RESISTANCE          100000.0f      /* 2. NTC 25℃ 阻值 */
#define NTC_SERIES_RESISTOR     100000.0f      /* 3. 串联贴片电阻 */
#define NTC_B_VALUE             3950.0f        /* 4. B 值 */
```

## 使用

```cpp
NTC_Temperature ntc;
ntc.init();                 /* 缺省绑定 ADC1 + DMA1_CH1 */
ntc.start();
float t = ntc.getTemp();      /* ℃, 无效返回 NTC_INVALID_TEMP_C */
```

## 静态换算接口

纯函数，可在 host 侧直接单元测试：

```cpp
float t = NTC_Temperature::adcToCelsius(2048);            // ADC → ℃
float r = NTC_Temperature::celsiusToResistance(25.0f);    // ℃ → Ω
float c = NTC_Temperature::resistanceToCelsius(r);        // Ω → ℃
```

## 实测精度（本工程 100K/100K/B3950 参数）

| 项 | 值 |
|----|-----|
| 全量程往返误差（−40 ~ +125 ℃） | ≤ 0.061 ℃ |
| 分辨率（25 ℃ 附近） | 0.022 ℃/LSB |
| 分辨率（−40 ℃ / 125 ℃） | 0.138 / 0.297 ℃/LSB |
