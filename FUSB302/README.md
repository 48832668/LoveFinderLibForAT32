# FUSB302 — USB Type-C / PD 控制器库 (C++17)

## 概述

onsemi FUSB302（Type-C / PD 接口控制器）驱动。寄存器映射依据
**FUSB302B 数据手册 Register Definitions**。

当前为 **第二阶段：SINK 模式 PD 受电驱动 (`PDSink`)** —— 在第一阶段
「通讯自检」（读 Device ID 验证 I2C 在线）之上，补齐 CC 检测、Source
Capabilities 解析、RDO 构造与完整受电状态机；`FUSB302` 类提供寄存器
读写与批量 FIFO 收发口。

> 完整配置说明见 [`CONFIG_GUIDE.md`](CONFIG_GUIDE.md)。

## 文件结构

| 文件 | 说明 |
|------|------|
| `FUSB302.hpp` | 寄存器枚举 `e_FUSB302_Reg`、`FUSB302` 类（寄存器读写 + 批量 `readBytes/writeBytes`） |
| `FUSB302.cpp` | 实现（阻塞式硬件 I2C 读写 + probe；批量写自动按 16B 分片） |
| `PDSink.hpp` | SINK 侧 PD 状态机（`PDSink` 类、超时常量、位域速查注释） |
| `PDSink.cpp` | CC 扫描 / 收 caps / RDO 构造 / Request→Accept→PS_RDY / PPS 周期续签 |
| `CONFIG_GUIDE.md` | 配置引导 |
| `README.md` | 本文档 |

## 依赖

- `lf_config.h`（工程侧配置头）+ `i2c_master.hpp`（库内阻塞式硬件 I2C 驱动）
- **只支持硬件 I2C**（`init` 传入 `i2c_type*`）。若某工程没有空余硬件 I2C，
  可仿 SW3526 补一个 `initBitbang`。

## 关键点

### 1. 7-bit 地址固定 0x22

`0b0100010`（写 0x44 / 读 0x45 是 **8-bit** 形式，别按 8-bit 往库里填）。
无地址引脚；ADDR 选择的衍生型号为 0x23 / 0x24 / 0x25（8-bit 0x46/0x48/0x4A），
需要时 `init(bus, addr7)` 覆盖。

### 2. probe = 读 Device ID (寄存器 0x01)

```cpp
bool ok = ct.probe();              // 应答 且 != 0x00 / 0xFF
auto id = ct.getDeviceId();        // optional<uint8_t>
```

Device ID 位域（0x01，只读）：

| 位 | 名称 | 说明 |
|----|------|------|
| 7:4 | Version ID | A=0x8 / B=0x9 / C=0xA（由 Trim 决定） |
| 3:2 | Product ID | 00 = FUSB302BMPX/UCX，其余为 B01/B10/B11（仅 MLP） |
| 1:0 | Revision ID | revA=00 … revD=11 |

- 读得回 = 地址被 ACK（有器件应答）；`0x00 / 0xFF` 再滤掉总线被拉死却
  凑巧读回全 0 / 全 1 的假成功。FUSB302B 实读多为 `0x9x`。

### 3. 寄存器读时序由 i2c_master 负责

`memRead = START + addr(W) + reg + repeated START + addr(R) + data + STOP`
（不发中间 STOP），本驱动不自己拼时序。

## 使用

```cpp
#include "FUSB302.hpp"
using namespace LoveFinderLib;

FUSB302 ct;
ct.init(LF_I2C_BUS1, FUSB302_I2C_ADDR_7BIT, "FUSB302");   // LF_I2C_BUS1 = I2C2
ct.setTimeout(FUSB302_I2C_TIMEOUT_MS);

if (ct.probe()) {
    auto id = ct.getDeviceId();
    /* ... */
}
```

寄存器访问（后续功能的基座）：

```cpp
ct.writeReg(e_FUSB302_Reg::POWER, 0x07);          // PD 必开的内部模块电源
auto v = ct.readReg(e_FUSB302_Reg::STATUS0);      // optional<uint8_t>
```

## PDSink — SINK 模式受电状态机

定位（907-PowerOneT6 硬件架构）：FUSB302 的 CC 直连 SW3526S（A1 输出
通道）的 CC。平时 USBA 口由 SW3526S 自跑协议快充；进入**可调电源模式**
时用 `PDSink` 走 CC/PD 向 SW3526S 申请 PPS，把 USBA 口变成电压可调输出
（USBA 转 DC，带 12V 路由器 / 20V 显示器等）。

```cpp
#include "PDSink.hpp"
using namespace LoveFinderLib;

FUSB302 ct;  ct.init(LF_I2C_BUS1, FUSB302_I2C_ADDR_7BIT, "FUSB302");
PDSink  pd;  pd.init(ct);
if (pd.begin() && ct.probe()) {
    pd.setTarget(12000, 1000);   // 目标 12V / 1A (mV, mA)
    pd.start();                  // 进 DETECT, CC1/CC2 挂 Rd 下拉
}
/* 主循环 */
pd.tick(lf_get_tick());
if (pd.isReady()) { use(pd.contractMv(), pd.contractMa()); }
```

状态机（`tick` 驱动，非阻塞，轮询式不用 INT 脚）：

| 状态 | 行为 |
|------|------|
| `DETECT` | 交替 MEAS_CC1/CC2 读 `STATUS0.BC_LVL`，连续两轮稳定且有 Rp → attach（极性取大者，平手先 CC1） |
| `WAIT_CAP` | 等 Source Capabilities（10s；超时：双 CC 都有 Rp 先换极性重试 → 再发 Get_Source_Cap → `FAULT(NOSRCS)`） |
| `CAPS` | 已收到 Source Caps；`setAutoRequest(false)` 时停在这里等上层 `requestNow()`（给用户看 PDO 列表用），否则直接进 SEND_REQ |
| `SEND_REQ` | 按 `setTarget` 选 PDO（优先 PPS APDO 中覆盖目标且范围最小的，退路固定 PDO 精确同压）构造 RDO 压 TX FIFO |
| `WAIT_ACCEPT` | 等 Accept（2.7s 超时重发 Request ×3 → `FAULT(NO_RESPONSE)`；Reject → 同故障；Wait → 重置宽限） |
| `WAIT_PS_RDY` | 等 PS_RDY（5s → `FAULT(NO_RESPONSE)`）→ 合同落账进 `READY` |
| `READY` | PPS 每 5s 周期重发同一 RDO 续签；`setTarget` 改目标 → 自动重协商；BC_LVL 连续两轮归零 → 拔出回 `DETECT` |
| `FAULT` | `fault()`/`faultName()` 给原因，`clearFault()` 或 `start()` 重走 |

其它行为：收到 Hard Reset（`INTERRUPTA` 闩锁）→ 清合同回 `WAIT_CAP`；
收到 SoftReset → 回包后回 `WAIT_CAP`；RX 收到扩展报文 → 整条 FIFO 冲掉。

**中断 / INT 脚**：`begin()` 会配好中断掩码（Mask1 全屏蔽；Maska 解
HARDRST/TXSENT/HARDSENT/RETRYFAIL；Maskb 解 GCRCSENT）并把
`CONTROL0.INT_MASK` 清 0 打开 INT 脚（907 板：PA5 / EXINT5 下降沿，外部 10K 上拉）。
`tick()` 每次都把 `INTERRUPT/INTERRUPTA/INTERRUPTB` 读掉（读即清），所以 INT 脚
不会被钉在低电平；上层可以 EXINT 到了立刻调一次 `tick()`，也可以只靠周期 tick。

**手动申请（可选）**：`setAutoRequest(false)` → 收到 caps 后停在 `CAPS`，
上层调 `requestNow()` 才发 Request（907 就是用它先给用户看 PDO 列表）。

超时常量可用 `lf_config.h` 覆盖：`PD_SINK_T_WAIT_CAP_MS`（10s）、
`PD_SINK_T_SENDER_RESP_MS`（2.7s）、`PD_SINK_T_PS_RDY_MS`（5s）、
`PD_SINK_T_PPS_REQ_MS`（5s）。
