# SW2305 — 配置引导 (AT32)

SW2305 USB PD 协议芯片驱动。寄存器映射依据 **SW2305_REG V1.2**（珠海智融科技）。

## 依赖

- `lf_config.h`（工程侧配置头）+ `i2c_master.hpp`（库内阻塞式硬件 I2C 驱动）
- 一条硬件 I2C 总线的实例宏（如 `LF_I2C_BUS0 = I2C1`）

## 在项目 `lf_config.h` 中配置

```cpp
/* ---- I2C 地址 (7-bit 形式) ---- */
#define SW2305_I2C_ADDR_7BIT        0x28    /* 8-bit 写地址 = 0x50 */
#define SW2305_I2C_TIMEOUT_MS       100

/* ---- 检流电阻 ---- */
#define SW2305_SENSE_MILLIOHM       5       /* ★ 本项目 5mΩ */
```

> **地址约定**：库内部统一使用 **7-bit** 地址。若你的宏习惯写 8-bit 形式
> （如 `0x50`），传入时右移一位：`SW2305_I2C_ADDR >> 1`。

## 检流电阻与电流标定（重要）

数据手册中 `adc_ich` 的 **3.125 mA/bit 是 10mΩ 基准**。
实际电流按检流电阻等比缩放：

```
实际电流 = adc_ich × 3.125mA × (10mΩ / Rsense_mΩ)
```

- `Rsense = 10mΩ` → 比例 1.0
- `Rsense = 5mΩ`  → 比例 2.0（**本项目**，同样电流下 ADC 读数翻倍）

只要调用 `setSenseResistor(5)`，`getIch()` 会自动完成缩放。

## 写保护（重要）

SW2305 的部分寄存器受写保护，**必须先解锁**：

| 目标寄存器 | 解锁寄存器 | 解锁序列 | 库接口 |
|-----------|-----------|---------|--------|
| `0x14`, `0xA0~0xBF` | `0x12` | `0x20 → 0x40 → 0x80` | `unlockForConfig()` |
| `0x16` | `0x15` | `0x20 → 0x40 → 0x80` | `unlockForForceCtrl()` |

未解锁就写，操作会被芯片静默忽略。

## 使用

```cpp
#include "SW2305.hpp"
using namespace LoveFinderLib;

SW2305 ch3;

void setup() {
    ch3.init(LF_I2C_BUS0, 0x28, "CH3");   /* 实例来自 lf_config.h (I2C1) */
    ch3.setSenseResistor(5);               /* 5mΩ */
    ch3.setTimeout(100);

    if (!ch3.probe()) {
        /* 芯片无响应 */
    }
}

void loop() {
    uint16_t vin  = ch3.getVin();      /* mV */
    uint16_t vbus = ch3.getVbus();     /* mV */
    uint16_t ich  = ch3.getIch();      /* mA, 已按 5mΩ 缩放 */
    float    td   = ch3.getTdiet();    /* ℃ (Die 温度) */

    if (ch3.isFastCharging()) {
        const char *p = SW2305::getProtocolName(ch3.getFastProtocol());
    }
}
```

## ADC 读取机制

ADC 是 12-bit，但不能直接读 `0x3C/0x3D`——必须先写 `0x3B` 选择通道，
芯片会把对应数据**锁存**到 `0x3C`（高 8 位）/ `0x3D`（低 4 位），
以保证高低字节对应。库的 `getVin()` 等函数已封装该流程：

```cpp
/* 手动方式 */
ch3.selectADCChannel(e_SW2305_ADC_Channel::VBUS);
uint16_t raw = ch3.readADC12();     /* 12-bit 原始值 */
```

| 通道 | 枚举 | 分辨率 |
|------|------|--------|
| Vin | `VIN` | 7.5 mV/bit |
| Vbus | `VBUS` | 7.5 mV/bit |
| Ich | `ICH` | 3.125 mA/bit @10mΩ |
| Die 温度 | `TDIET` | `Tdiet = (adc − 1848) / 6.72` ℃ |

## 常用配置接口

所有 `0xA0~0xBF` 的配置写入前需先 `unlockForConfig()`：

```cpp
ch3.unlockForConfig();

ch3.setWireComp(e_SW2305_WireComp::OHM_50);        /* 线补阻抗 (0xA4) */
ch3.setOtpThreshold(e_SW2305_OtpThreshold::C125);  /* 过温门限 (0xAB) */
ch3.setOnlineCurrent(e_SW2305_OnlineCurrent::MA_200);/* 在线门限 (0xAC) */
ch3.setMaxPowerByRegister(65);                     /* 最大功率 65W (0xAF) */

/* PD / 快充协议使能 (位语义: 0=使能, 1=不使能) */
ch3.setPDEnable(true);
ch3.setPDVoltageEnable(true, true, true, true);    /* 9/12/15/20V */
ch3.setPPSEnable(true, true, false, false);        /* PPS0/PPS1 */
ch3.setQC2Enable(true);
ch3.setQC3Enable(true);
ch3.setSCPEnable(true, true);                      /* 高压/低压 SCP */
ch3.setFCPEnable(true);
ch3.setAFCEnable(true);
```

强制控制（`0x16`）需先 `unlockForForceCtrl()`：

```cpp
ch3.unlockForForceCtrl();
ch3.forcePathOn();
ch3.forceDacControl(true);
ch3.clearForceCtrl();       /* 清除全部强制位 */
```

## 状态读取

```cpp
/* 快充指示 (0x06) */
e_SW2305_FastProt   prot = ch3.getFastProtocol();
e_SW2305_PD_Version pdv  = ch3.getPDVersion();
bool fast  = ch3.isFastCharging();   /* [7] 处于快充协议 */
bool vfast = ch3.isVoltageFast();    /* [6] 处于快充电压 */

/* 系统状态 */
auto s0 = ch3.getSysStatus0();   /* 通路管/线补/CC环路/拉光耦 */
auto s1 = ch3.getSysStatus1();   /* Vin过压欠压/过流/过温 */
auto s2 = ch3.getSysStatus2();   /* CC/DP/DM过压/低电/过流保护 */
bool online = ch3.isOnline();    /* 0x0D [7] */

/* 回读设定值 */
uint16_t dac  = ch3.getTargetVoltage();  /* mV = dac_vol × 10 */
uint16_t icc  = ch3.getCCLimit();        /* mA = 1000 + ctrl × 50 */
```

## 协议名称

```cpp
const char *name = SW2305::getProtocolName(e_SW2305_FastProt::PD_PPS);
/* "PPS " — 固定 4 字符宽, 便于等宽字体对齐 */
```

## 已知约束

1. **未定义的寄存器/位不可改写**。库只操作数据手册明确定义的位。
2. `getFastProtocol()` 对 reserved 值会重读一次，仍无效则返回 `NONE`。
3. `getTdiet()` 在 ADC 无有效数据时返回 0.0f（非 `NTC_INVALID_TEMP_C`）。
4. 解锁序列内含 `wk_delay_ms(1)` 间隔，**不要在高频中断中调用**。
