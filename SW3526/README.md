# SW3526 — AT32 快充控制器库 (C++17)

## 概述

SW3526S USB 快充控制器驱动。**驱动本身不含 I2C 实现** —— 收发走库内
`I2C/i2c_master` 阻塞驱动，驱动只持有一个 `i2c_type*` 外设实例。

> 完整配置说明见 [`CONFIG_GUIDE.md`](CONFIG_GUIDE.md)。

## 文件结构

| 文件 | 说明 |
|------|------|
| `SW3526.hpp` | 寄存器映射、枚举、`SW3526` 与 `SW3526_Manager` 类 |
| `SW3526.cpp` | 实现（I2C 通讯、ADC 测量、状态读取、协议配置） |
| `CONFIG_GUIDE.md` | 配置引导 |
| `README.md` | 本文档 |

## 依赖

- `lf_config.h`（工程侧配置头）+ `i2c_master.hpp`（库内阻塞式硬件 I2C 驱动）

## 使用

```cpp
#include "SW3526.hpp"
using namespace LoveFinderLib;

/* 通道 1: TYPE-C, 硬件 I2C1 */
SW3526 ch1;
ch1.init(LF_I2C_BUS0, 0x3C, "CH1");   /* 7-bit 地址 0x3C; 实例来自 lf_config.h */
ch1.setSenseResistor(10);              /* 检流电阻 10mΩ */

/* 读取 */
uint16_t vin  = ch1.getVin();      /* mV */
uint16_t vout = ch1.getVout();     /* mV */
uint16_t iout = ch1.getIout();     /* mA (已按检流电阻缩放) */

/* 协议配置 */
ch1.setFastMode(e_SW3526_FastMode::FAST_ON);
ch1.setPortType(e_SW3526_PortType::TYPE_C);
```

## 地址约定

库内部统一使用 **7-bit** 地址：

| 芯片 | 7-bit | 8-bit 写地址 |
|------|-------|-------------|
| SW3526S | `0x3C` | `0x78` |

若宏写成 8-bit 形式，传入时右移一位：`SW3526_I2C_ADDR_7BIT >> 1`。

## 注意事项

- 写受保护寄存器前必须先 `unlockRegisters()`（`setFastMode()` 内部已自动调用）
- 使能类寄存器的位语义为 **0 = 使能、1 = 关闭**；库接口统一用
  「`true` = 使能」语义，内部做反转，调用方无需关心
- `setCCLimit()` 有效范围 **1000 ~ 4100 mA**，步进 50mA
- I2C 收发为阻塞轮询 + `lf_get_tick()` 超时；`unlockRegisters()` 内含
  `wk_delay_ms(1)` 间隔

## 本项目通道映射

| 通道 | 芯片 | 接口 | 7-bit 地址 | 实例宏 |
|------|------|------|-----------|---------|
| CH1 | SW3526S (TYPE-C) | I2C1 (硬件) | `0x3C` | `LF_I2C_BUS0` |
| CH3 | SW2305 + SC8703 | I2C1 (硬件) | `0x28` | `LF_I2C_BUS0` |

> 软件 Bitbang 后端已随平台抽象层删除（无工程使用）。
