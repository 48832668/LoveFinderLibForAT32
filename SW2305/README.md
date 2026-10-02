# SW2305 — 快充协议芯片库 (C++17)

## 概述

SW2305 USB PD 协议芯片驱动。寄存器映射依据 **SW2305_REG V1.2**（珠海智融科技）。

> 完整配置说明见 [`CONFIG_GUIDE.md`](CONFIG_GUIDE.md)。

## 文件结构

| 文件 | 说明 |
|------|------|
| `SW2305.hpp` | 寄存器映射、枚举、状态结构、`SW2305` 类 |
| `SW2305.cpp` | 实现（I2C 通讯、ADC 测量、状态读取、配置写入） |
| `CONFIG_GUIDE.md` | 配置引导 |
| `README.md` | 本文档 |

## 依赖

- `lf_config.h`（工程侧配置头）+ `i2c_master.hpp`（库内阻塞式硬件 I2C 驱动）

## 三个关键点

### 1. 写保护需先解锁

| 目标寄存器 | 解锁寄存器 | 序列 | 库接口 |
|-----------|-----------|------|--------|
| `0x14`, `0xA0~0xBF` | `0x12` | `0x20→0x40→0x80` | `unlockForConfig()` |
| `0x16` | `0x15` | `0x20→0x40→0x80` | `unlockForForceCtrl()` |

未解锁就写，操作会被芯片静默忽略。

### 2. ADC 需先选通道

写 `0x3B` 选择通道后，数据才锁存到 `0x3C`（高 8 位）/ `0x3D`（低 4 位）。
库的 `getVin()` / `getVbus()` / `getIch()` / `getTdiet()` 已封装该流程。

| 通道 | 枚举 | 分辨率 |
|------|------|--------|
| Vin | `VIN` | 7.5 mV/bit |
| Vbus | `VBUS` | 7.5 mV/bit |
| Ich | `ICH` | 3.125 mA/bit @10mΩ |
| Die 温度 | `TDIET` | `Tdiet = (adc − 1848) / 6.72` ℃ |

### 3. 电流需按检流电阻缩放

手册的 **3.125 mA/bit 是 10mΩ 基准**：

```
实际电流 = adc_ich × 3.125mA × (10mΩ / Rsense_mΩ)
```

本项目 5mΩ → 比例 ×2。`setSenseResistor(5)` 会自动处理。

## 使用

```cpp
#include "SW2305.hpp"
using namespace LoveFinderLib;

SW2305 ch3;
ch3.init(LF_I2C_BUS0, 0x28, "CH3");   /* I2C1, 7-bit 0x28 */
ch3.setSenseResistor(5);               /* 5mΩ */

if (ch3.probe()) {
    uint16_t vbus = ch3.getVbus();     /* mV */
    uint16_t ich  = ch3.getIch();      /* mA (已缩放) */
    float    td   = ch3.getTdiet();    /* ℃ */
    bool     fast = ch3.isFastCharging();
}
```

## 寄存器总览

| 地址 | 名称 | R/W | 说明 |
|------|------|-----|------|
| `0x01` | `CHIP_VER` | R | 芯片版本（[1:0]） |
| `0x03`~`0x04` | `DAC_VOL_H/L` | R | 设置电压 `dac_vol × 10mV` |
| `0x05` | `CTRL_ICC` | R | 设置限流 `1000mA + val × 50mA` |
| `0x06` | `FAST_STATUS` | R | 快充协议 / PD 版本指示 |
| `0x07` | `SYS_STATUS0` | R | 通路管 / 线补 / CC 环路 |
| `0x0B` | `SYS_STATUS1` | R | Vin 过压欠压 / 过流 / 过温 |
| `0x0C` | `SYS_STATUS2` | R | CC / DP / DM 过压 / 低电 / OCP |
| `0x0D` | `SYS_STATUS3` | R | 在线指示（Bit7） |
| `0x12` | `UNLOCK_0` | W | 解锁 `0x14` / `0xA0~0xBF` |
| `0x14` | `CONN_CTRL` | R/W | 线补控制 / CC un-driving |
| `0x15` | `UNLOCK_1` | W | 解锁 `0x16` |
| `0x16` | `FORCE_CTRL` | R/W | 强制控制使能 |
| `0x30`~`0x36` | `ADC_*` | R | Vin/Vbus/Ich/Tdiet 高 8bit |
| `0x3B`~`0x3D` | `ADC_CFG/DATA` | R/W | 12-bit 通道选择 + 数据锁存 |
| `0xA1`~`0xBF` | OTP 配置 | R/W | 需先解锁 |
| `0xB6`~`0xBF` | VID/XID/PID/SVID | R/W | 需先解锁 |

## 注意事项

- **未定义的寄存器或位不可改写**，库只操作数据手册明确定义的位
- 解锁序列内含 `wk_delay_ms(1)` 间隔，**不要在高频中断中调用**
- 使能类配置位的语义为 **0 = 使能、1 = 不使能**；库接口统一用
  「`true` = 使能」语义，内部做反转
- `getFastProtocol()` 遇到 reserved 值会重读一次，仍无效则返回 `NONE`
- `getTdiet()` 无有效数据时返回 `0.0f`
