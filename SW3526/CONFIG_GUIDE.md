# SW3526 — 配置引导 (AT32)

SW3526S USB PD / 快充控制器驱动。寄存器映射依据 **SW3526S_REG V1.0.0**。

## 依赖

- `lf_config.h`（工程侧配置头）+ `i2c_master.hpp`（库内阻塞式硬件 I2C 驱动）
- 一条硬件 I2C 总线的实例宏（如 `LF_I2C_BUS0 = I2C1`）

## 核心设计：外设实例注入

**驱动本身不含任何 I2C 实现**。收发走库内 `I2C/i2c_master` 自由函数，
驱动只持有一个 `i2c_type*` 外设实例，`init()` 时传入：

```cpp
SW3526 ch1;   ch1.init(LF_I2C_BUS0, 0x3C, "CH1");  /* I2C1 (lf_config.h 实例宏) */
```

> 软件 Bitbang 后端已随平台抽象层删除（无工程使用）。
> 多条总线时传对应外设实例（`I2C1` / `I2C2` / …）即可，一份代码驱动任意条。

## 在项目 `lf_config.h` 中配置

```cpp
/* ---- 通道实例与地址 ---- */
#define LF_I2C_BUS0             I2C1        /* 硬件 I2C 实例 (at32f421 BSP) */
#define SW3526_I2C_ADDR_7BIT    0x3C        /* 7-bit 地址 */
#define SW3526_I2C_TIMEOUT_MS   100         /* ms */
```

> **地址约定**：库内部统一使用 **7-bit** 地址（SW3526S 为 `0x3C`）。
> 若宏写成 8-bit 形式（`0x78`），传入时右移一位：`SW3526_I2C_ADDR_7BIT >> 1`。

## I2C 说明

- 收发为**阻塞轮询**（`i2c_master`），每步等标志 + `lf_get_tick()` 超时
- 寄存器读用 **repeated START** 规范时序（不发中间 STOP）
- `i2cMasterRecover()` 只做外设软复位；GPIO 级总线恢复由应用层实现

## 使用

```cpp
#include "SW3526.hpp"
using namespace LoveFinderLib;

SW3526 ch1;

void setup() {
    ch1.init(LF_I2C_BUS0, 0x3C, "CH1");
    ch1.setSenseResistor(10);          /* 检流电阻 10mΩ */
    ch1.setTimeout(100);

    if (!ch1.probe()) {
        /* 芯片无响应 */
    }
}

void loop() {
    uint16_t vin  = ch1.getVin();      /* mV */
    uint16_t vout = ch1.getVout();     /* mV */
    uint16_t iout = ch1.getIout();     /* mA, 已按检流电阻缩放 */
    uint16_t pwr  = ch1.getAllMeasurements().power_mw;

    if (ch1.isFastCharging()) {
        const char *p = SW3526::getProtocolName(ch1.getFastStatus());
    }
}
```

## 检流电阻与电流标定

ADC 分辨率按 **10mΩ 基准**定义，实际电流按检流电阻等比缩放：

```
实际电流 = 原始值 × (10mΩ / Rsense_mΩ)
```

`setSenseResistor(10)` 即为基准（比例 1.0）。若换成 5mΩ，比例变 2.0。

## ADC 读取机制

ADC 为 12-bit：高 8 位在 `VIN_L`/`VOUT_L`/`IOUT_L`，低 4 位在 `ADC_LSB[3:0]`。
读取前需通过 `ADC_SENSE` 选择通道（库已封装）：

| 通道 | 分辨率（10mΩ 基准） |
|------|-------------------|
| Vin | 10 mV/bit |
| Vout | 6 mV/bit |
| Iout | 2.5 mA/bit |

## 快充协议控制

部分寄存器受写保护，`setFastMode()` 内部会自动调用 `unlockRegisters()`
（向 `0x12` 写 `0x20 → 0x40 → 0x80`）。

```cpp
ch1.setFastMode(e_SW3526_FastMode::FAST_ON);   /* 全协议 */
ch1.setFastMode(e_SW3526_FastMode::ONLY_PD);   /* 仅 PD   */
ch1.setFastMode(e_SW3526_FastMode::NO_FAST);   /* 关闭快充 */
```

协议单项使能（位语义：**0 = 使能，1 = 关闭**，库接口已反转）：

```cpp
ch1.setPDEnable(true);
ch1.setQC2Enable(true);
ch1.setQC3Enable(true);
ch1.setSCPEnable(true);
ch1.setFCPEnable(true);
ch1.setAFCEnable(true);
ch1.setSFCPEnable(true);
ch1.setPEEnable(true);
ch1.setFastChargeEnable(true);     /* 快充总开关 */
ch1.setDPDMEnable(true);           /* BC1.2 / DPDM */

/* PD 电压档位 */
ch1.setPDVoltageEnable(true, true, true, true);   /* 9/12/15/20V */
ch1.setPPSEnable(true, true);                     /* PPS0/PPS1 */
```

其他配置：

```cpp
ch1.setPortType(e_SW3526_PortType::TYPE_C);       /* 或 TYPE_A */
ch1.setNoLoadVoltage(e_SW3526_NoLoadVolt::V5_12);
ch1.setWireCompEnable(true);
ch1.setWireCompSize(e_SW3526_WireComp::MV_120_PER_A);
ch1.setMaxPower(65);                              /* W */
ch1.setSCPMaxCurrent(e_SW3526_SCP_MaxCurrent::A3_5);
ch1.setNonPdMaxVoltage(e_SW3526_NonPdMaxVolt::V12);
```

## 状态与故障

```cpp
auto sys = ch1.getSysStatus();
bool open = sys.port_open;      /* 通路管打开 */
bool buck = sys.buck_on;        /* Buck 工作 */

auto f = ch1.getFaultStatus();
if (f.input_ov || f.input_uv || f.output_uv || f.die_ot_shdn || f.output_sc) {
    /* 故障处理 */
}

uint8_t cc = ch1.getCCConnection();   /* bit0=CC1, bit1=CC2 */
uint8_t w  = ch1.getWattConfig();
```

## 多实例管理

```cpp
SW3526_Manager mgr;
mgr.addDevice(&ch1);
mgr.addDevice(&ch2);

mgr.forEach([](SW3526 *dev, size_t idx) {
    dev->getAllMeasurements();
});

SW3526 *found = mgr.findByName("CH2");
```

## 协议名称

```cpp
const char *name = SW3526::getProtocolName(e_SW3526_FastProt::PD_PPS);
/* "PPS " — 固定 4 字符宽, 便于等宽字体对齐 */
```

## 已知约束

1. `unlockRegisters()` 内含 `wk_delay_ms(1)` 间隔，不要在高频中断中调用。
2. `setCCLimit()` 有效范围 **1000 ~ 4100 mA**，步进 50mA，超范围返回 `false`。
3. 位语义易混淆：SW3526S 的使能类寄存器多为 **0=使能、1=关闭**。
   库接口统一用「`true` = 使能」的语义，内部做反转，调用方无需关心。
