# FUSB302 配置引导

FUSB302 / PDSink 共读 **6 个**工程侧宏，全部定义在 `lf_config.h`
（库内有 `#ifndef` 缺省，不定义也能跑）：

| 宏 | 缺省 | 说明 |
|----|------|------|
| `FUSB302_I2C_ADDR_7BIT` | `0x22` | 7-bit 从机地址（本芯片固定；ADDR 衍生型号 0x23/0x24/0x25） |
| `FUSB302_I2C_TIMEOUT_MS` | `50` | 单次寄存器事务超时（ms），走 `lf_get_tick()` |
| `PD_SINK_T_WAIT_CAP_MS` | `10000U` | 等 Source Capabilities 超时（先换极性重试，再发 Get_Source_Cap，三次耗尽才 FAULT） |
| `PD_SINK_T_SENDER_RESP_MS` | `2700U` | Request → Accept 超时（重发 Request ×3 后 FAULT） |
| `PD_SINK_T_PS_RDY_MS` | `5000U` | Accept → PS_RDY 超时 |
| `PD_SINK_T_PPS_REQ_MS` | `5000U` | PPS 合同周期续签间隔（tPPSRequest，掉签会掉电压） |

## 实例宏（I2C 外设）

`init()` 第一个参数是 `i2c_type*`，通常传工程 `lf_config.h` 里定义的总线宏：

```c
#define LF_I2C_BUS1   I2C2     /* 本板 FUSB302 挂硬件 I2C2 (PA0=SCL / PA1=SDA) */
```

```cpp
FUSB302 ct;
ct.init(LF_I2C_BUS1);          // 缺省地址 / 名字
if (ct.probe()) { ... }        // Device ID (0x01) 应答即在线
```

## 引脚与总线前提

- 硬件 I2C2 的引脚复用由 WorkBench 生成的 `wk_i2c2_init()` 完成
  （PA0 = SCL / PA1 = SDA，`wk_i2c.c` 里写死）。
- 建议上电先把两脚 ODR 置 1 再切复用（开漏阶段 ODR=0 有把总线钉低的风险）——
  见工程 `wk_gpio.c` 的 user code 块。
- 冷启动 / 从机把线拉死时，应用层先做 GPIO 级 9 脉冲 + STOP 解总线，
  再 `wk_i2c2_init()` 交还外设（参照工程 `recoverHwBusByBitBang`）。
