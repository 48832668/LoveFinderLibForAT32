# SW3538 配置指南

SW3538 多口快充控制器驱动（I2C，读为主）。寄存器依据
`SW3538_寄存器列表 Release_RG108_3_v1.2`。

本模块**只读工程侧 `lf_config.h`**，库内没有 conf 文件。

## 工程侧必须提供

| 宏 | 说明 | 本库缺省 | 示例 |
|---|---|---|---|
| `SW3538_I2C_ADDR_7BIT` | 7-bit 从机地址 | `0x3C`（与 SW3526S 相同） | `#define SW3538_I2C_ADDR_7BIT 0x3C` |
| `SW3538_I2C_TIMEOUT_MS` | 单次 I2C 事务超时 | `100` | `#define SW3538_I2C_TIMEOUT_MS 50` |
| `SW3538_SENSE_MILLIOHM` | 检流电阻（电流换算基准） | `5`（手册 2.5mA/bit 的基准值） | `#define SW3538_SENSE_MILLIOHM 5` |
| `lf_get_tick()` | 时基（库内 I2C 用） | —— | 工程 `lf_irq.cpp` 实现 |

I2C 外设实例**不是宏**，由调用方 `init(bus, addr, name)` 传入：
`LF_I2C_BUS0`（I2C1）、`LF_I2C_BUS1`（I2C2）等。

## 用法

```cpp
#include "SW3538.hpp"

using LoveFinderLib::e_SW3538_Port;

LoveFinderLib::SW3538 ch1, ch2;
// ★ 第 4 个形参 = 这颗芯片实际接了哪些口 (1=Type-C / 2=Type-A)。
//   POT1 每颗只接通路1; 不声明的话驱动会去读没接的口的电流和状态位。
ch1.init(LF_I2C_BUS0, SW3538_I2C_ADDR_7BIT, "IIC1", e_SW3538_Port::PATH1);
ch2.init(LF_I2C_BUS1, SW3538_I2C_ADDR_7BIT, "IIC2", e_SW3538_Port::PATH1);
// 两个口都接: e_SW3538_Port::BOTH (缺省值, 不写第 4 参 = BOTH)

if (ch1.probe()) {                 // 读 Reg0x00 成功 = 在线
    ch1.enableAdcSampling();       // ★ 必须做一次, 否则 VIN 读不到 (默认关闭)
    LoveFinderLib::SW3538_Measurement m;
    if (ch1.readMeasurement(m)) {
        // m.vbus_mv   ← 屏幕要显示的 (口上电压, 空载 = 0V)
        // m.vout_mv   ← BUCK 输出 (空载约 5.1V), 功率口径仍用它
        // m.vin_mv / m.iout_ma / m.power_mw / m.fast / m.path1Online
    }
}
```

## 关键点

1. **写寄存器要解锁**：`writeReg()` 内部自带 `Reg0x10 = 0x20 → 0x40 → 0x80`
   序列，调用方不用管；写 `Reg0x16~0x19` 用 `writeRegForce()`（额外走
   `Reg0x15` 强制操作序列）。
2. **VIN ADC 默认关闭**：上电后调一次 `enableAdcSampling()`
   （= `Reg0x18 = 0xE0`，打开 VIN / VOUT / 通路1电流三路采样）。
3. **ADC 是锁存式**：每次 `readAdc()` 先写 `Reg0x40` 选通道，再读
   `Reg0x41/0x42`，一次约 5 个 I2C 事务（含解锁序列）。
4. **检流电阻必须与板子一致**：手册电流换算是 `2.5mA/bit @5mΩ`，
   板上不是 5mΩ 时 `setSenseResistor(mΩ)` 或在 `lf_config.h` 里定义
   `SW3538_SENSE_MILLIOHM`，否则电流/功率全错。
5. **通路使能（`init` 第 4 参）**：`e_SW3538_Port::PATH1`（Type-C 口）/
   `PATH2`（Type-A 口）/ `BOTH`（缺省，不写第 4 参就是它）；可用 `|` 组合。
   没接的口驱动**不读电流、状态位恒 false**，否则 UI 会误报"通路2 有设备"
   这类根本不存在的状态。一条 I2C 总线上只能挂一颗 SW3538（现只有 7-bit
   `0x3C` 一个地址），多颗必须分总线；厂家将来若放出更多地址，只改 `init`
   的 `addr7` —— 每颗本来就是独立实例。
6. **VBUS ≠ VOUT，屏显要的是 VBUS**：
   `vout_mv` = BUCK 输出（ADC 通道 11，空载约 5.1V）；
   `vbus_mv` = 经 SW3538 控制的 NMOS 之后、口上的电压。按 PD 规范没接 SINK
   时不许 VBUS 上电 → 空载 VBUS = 0V 而 VOUT 仍 5.1V。
   手册 ADC 通道表里**没有独立的 VBUS 通道**，驱动用【通路导通状态】门控
   VOUT 得到（导通 ≈ VOUT、关断 = 0）。
   ★ 上板若发现恒 5.1V 或恒 0.00V，说明 `Reg0x0A[0]` 的语义与预期不符 ——
   把 `SW3538.cpp` VBUS 段的 `conducting` 换成 `path1Online`（有设备接入）
   或 `(buckOn && path1On)` 即可，只动那一处。
   功率仍按 `vout × iout`，不受该判据影响。
7. **本板接法（LoveFinder208）**：两颗 SW3538 各挂一路硬件 I2C
   （I2C1→IIC1、I2C2→IIC2），每颗 `init(..., e_SW3538_Port::PATH1)`
   只接 Type-C 口；UI 两列 C1/C2 各显示一颗的通路1（电压 = `vbus_mv`）。
