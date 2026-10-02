# LoveFinderLibForAT32 — 交付总结

**目标平台**: AT32F421 系列 (Cortex-M4F)
**标准**: 纯 C++17
**更新日期**: 2026-09-24

> 完整使用说明见 `AGENTS.md`；各模块配置见对应目录的 `CONFIG_GUIDE.md`。

---

## 1. 交付清单

| 模块 | 目录 | 文件 | 说明 |
|------|------|------|------|
| **I2C 主机** | `I2C/` | `i2c_master.hpp/.cpp` | 阻塞式硬件 I2C（自由函数，收 `i2c_type*`，repeated-START 寄存器读） |
| **BUTTON** | `BUTTON/` | `.hpp/.cpp` | 按键：单击 / 双击 / 长按 + 充能（EXINT **上下沿**触发，中断只置标志、判定全在 `update()`；释放也消抖；时基可切 TIMER） |
| **TIMER** | `TIMER/` | `.hpp/.cpp` | 可选的 TIM 毫秒时基（每 N ms 中断一次 `+N`）。默认不用；`BUTTON_TIMEBASE_TIMER=1` 才被引用，分频按整除档位选取故无累积误差 |
| **SW3526** | `SW3526/` | `.hpp/.cpp` | SW3526S 快充控制器（`init(i2c_type*, addr7, name)`） |
| **SW2305** | `SW2305/` | `.hpp/.cpp` | SW2305 PD 协议芯片（V1.2 寄存器，5mΩ 检流） |
| **NTC_Temperature** | `NTC_Temperature/` | `.hpp/.cpp` | NTC 测温（ADC-DMA + 驱动静态缓冲，B 值公式，四宏配置） |
| **ST7735** | `ST7735/` | `.hpp/.cpp` | LCD 驱动（SPI 直发 + dtcnt 轮询 DMA，完整图形原语，面板表内联） |
| **FontLib** | `FontLib/` | 9 个文件 | 驱动无关字库（PickSoul 规范，双模式 + 工程清单隔离） |

> 平台抽象层 `lf_platform*.{hpp,cpp}` 已于 2026-09-24 **删除**（对齐 PY32 模式）。

---

## 2. 架构（直绑模式）

```
应用层 (工程侧)
    → 提供 lf_config.h (全部配置) + lf_get_tick() 时基实现 + fonts_config.hpp 清单
    ↓ #include "lf_config.h"
驱动层 (库)
    BUTTON · I2C · SW3526 · SW2305 · NTC_Temperature · ST7735 · FontLib
    直调 AT32 BSP: gpio_* / spi_* / i2c_* / dma_* / exint_* ; wk_delay_ms / lf_get_tick
    ↓
AT32 BSP (at32f421_*.c, WorkBench 生成)
```

**核心原则**

1. 库本体不放在工程内，工程通过 CMake `add_subdirectory` 引用
2. **无平台抽象层** —— 驱动直调 BSP；每 MCU 家族一份库（AT32 / PY32_LL / STM32_HAL 同构）
3. 项目专属配置（引脚/实例/阻值/地址）由工程侧 `lf_config.h` 提供，库内零 conf 文件

---

## 3. 工程侧契约

| 工程提供 | 声明 / 实现 | 库用途 |
|----------|-------------|--------|
| `lf_get_tick()` | `lf_config.h` 声明 / `lf_irq.cpp` 实现（SysTick 递增） | I2C 超时、按键消抖、LCD DMA 超时（仿 `BSP_GetTick`） |
| 引脚与实例宏 | `lf_config.h`（`ST7735_*` / `BUTTON_KEY_*` / `LF_I2C_BUS0` / `NTC_*`） | 驱动直绑的外设与引脚 |
| `fonts_config.hpp` | `project/inc/`（X 宏清单） | FontLib 只编译清单内字符 |
| 中断挂接 | `at32f421_int.c` → `lf_irq_tick()`；用按键时 EXINT 中调 `key.onExti()` | 时基、按键事件 |

### 891 工程实例映射

| 宏 / 实例 | 对应硬件 | 说明 |
|-----------|----------|------|
| `LF_I2C_BUS0` | I2C1 (PB6/PB7) | SC8102 状态读取 / SW3526·SW2305 |
| `NTC_ADC_INSTANCE` + `NTC_DMA_CHANNEL` | ADC1 IN0 + DMA1_CH1（缺省值） | NTC 测温 |
| `ST7735_SPI` + `ST7735_DMA_CHANNEL` | SPI1 + DMA1_CH3 | ST7735 |
| `BUTTON_KEY_PORT/PIN` | MCU1_KEY (WorkBench 宏) | 按键 |

---

## 4. 验证状态

**编译验证**（arm-none-eabi-g++ 10.3.1 / CMake / Ninja，2026-09-24 直绑重构后）

```
text 17352 + data 28 = FLASH 17380 B / 64 KB (26.52%)
data 28 + bss 2108 = RAM   2136 B / 16 KB (13.04%)
=== 零错误, 零警告 ===   （重构前含平台层: FLASH 25268 B / 38.56%）
```

**字库字符级隔离**（同一份库, 不同工程清单，891 实测）

| 项 | 结果 |
|----|------|
| 清单只留数字 | FLASH 25268 → 23492 B；map 中未列字形地址 `0x00000000`（被 gc-sections 丢弃） |
| 恢复全量清单 | 回到 25268 B（当时基线） |
| 缺失字形 | 工程清单声明库中不存在的字符 → **编译错误**（防静默缺字） |

**算法验证**（NTC 测温，host 侧数值核验）

| 项 | 结果 |
|----|------|
| 全量程往返误差（−40 ~ +125 ℃） | ≤ 0.061 ℃ |
| 分辨率（25 ℃ 附近） | 0.022 ℃/LSB |
| 边界（ADC=0 / 4095） | 正确判为无效 |
| 单调性 | 通过 |

**数据完整性**（FontLib）

| 项 | 结果 |
|----|------|
| ASCII 子集 65/65 数组 vs Mode A | 逐字节一致 |
| `FULLFONT_STY1_7x10` / `11x18` / `16x26` 尺寸 | 950 / 1710 / 2470 = 95×10/18/26 ✓ |
| C/C++ 链接（`fonts.c` ↔ `fonts_subset.cpp`） | 符号正确解析 |
| `USE_FONT_*` 开关门控 | 全部置 0 时零字体符号 |

---

## 5. 关键设计决策

| 决策 | 理由 |
|------|------|
| **删除平台抽象层，驱动直绑 BSP** | 对齐 `LoveFinderLibForPY32_LL` 模式：少一层翻译与绑定注入，配置集中在工程 `lf_config.h`，-O0 下还省约 7.9 KB Flash |
| **I2C 收发保留为库内 `I2C/` 模块** | SW3526/SW2305 共用一套状态机；仅保留硬件后端（Bitbang 无工程使用，已删） |
| **LCD DMA 纯轮询 dtcnt + tick 超时** | 工程 NVIC 未开 DMA 中断、弱定义 handler；与 PY32 同款做法，无 IRQ 依赖 |
| **NTC 位置宏在 `#if` 之前定义** | 原实现把 `NTC_LOW_SIDE` 定义在比较之后，导致 `#if` 恒真 |
| **SW2305 双级解锁** | 文档要求 `0x12` 解锁 `0x14`/`0xA0~0xBF`，`0x15` 解锁 `0x16` |
| **SW2305 电流 ×2 缩放** | 手册 3.125mA/bit 是 10mΩ 基准，5mΩ 需等比缩放 |
| **`font_get_glyph` 内部兼容模式 A** | 渲染端直接调用该函数，不内部分发会导致 Mode A 渲染失效 |
| **ST7735 DMA 用行缓冲** | 16-bit 像素 vs byte DMA 宽度不匹配，需展开为大端字节流 |
| **SPI 时序在 C++ 侧二次断言** | WorkBench 重新生成会把 `wk_spi.c` 还原成 HALF_DUPLEX/30MHz |

---

## 6. 待办

- [ ] 硬件实测（LCD 上电自检红绿蓝白 → HelloWorld、I2C 通讯、NTC 采样、按键时序）
- [ ] 屏幕颜色/偏移不对时依次试 `ST7735_PANEL_B/C/D`
- [ ] 若需 GPIO 级 I2C 总线恢复（9 脉冲），在应用层补充

---

*最后更新: 2026-09-24*
