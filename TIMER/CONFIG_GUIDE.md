# TIMER —— 可选的 TIM 毫秒时基 (AT32)

一个由**一路 TIM 周期中断**驱动的毫秒计数器：每 10ms 进一次中断，
中断里让计时变量 `+10`；主循环读 `timerNow()`。中断里只做「清标志 + 自增」，
全部业务判定留在主循环 —— 与 BUTTON 的两段式模型配套。

> **它是可选的，默认用不到。** 工程本就有 `lf_get_tick()`（SysTick 1ms 中断），
> 分辨率比 10ms 的 TIM 更细，且库里 I2C / ST7735 的超时、按键消抖全都用它。
> 只有在「想脱离 SysTick」或「要复用自己的一路 TIM 当时间轴」时才启用。

## 依赖

- `lf_config.h`（工程配置头；TIMER 自己另带 `at32f421_tmr.h` / `at32f421_crm.h` /
  `at32f421_misc.h` 三个 BSP 头，**不依赖 `at32f421_conf.h` 的模块开关** ——
  不用定时器的工程没开 `TMR_MODULE_ENABLED`，靠 conf.h 带不出来）
- 空闲的一路 TIM：**TMR6**（基本定时器，无通道无引脚，首选）/ **TMR14~17**（备选，
  同样可做纯溢出时基）/ TMR3（常被 WorkBench 的 LED PWM 占用）/ TMR1（高级定时器，
  两套中断，一般不用）

## 工程侧 `lf_config.h` 配置

```c
/* 按键时基: 0 = lf_get_tick() (SysTick 1ms, 默认) / 1 = 用一路 TIM */
#define BUTTON_TIMEBASE_TIMER   1
#define BUTTON_TIM_INSTANCE     TMR6     /* ★ 与下面的中断名两行一起改 */
#define BUTTON_TIM_CLK_HZ       120000000 /* 该 TIM 的计数时钟 (APB 分频 1 → sysclk) */
#define BUTTON_TIM_PERIOD_MS    10        /* 中断周期: 每次中断计时 +10ms */
#define BUTTON_TIM_IRQ          TMR6_GLOBAL_IRQHandler
```

| 宏 | 含义 | 不对会怎样 |
|----|------|-----------|
| `BUTTON_TIM_INSTANCE` | 占哪一路 TIM | 与中断名不配套 → 开了 A 的 NVIC 却只有 B 的 handler → 卡死 |
| `BUTTON_TIM_CLK_HZ` | 该 TIM 的计数时钟 | 偏了按比例快/慢；不合法 → `timerTickStart()` 返回 false |
| `BUTTON_TIM_PERIOD_MS` | 中断周期 | 时间轴粒度（建议 5~20ms） |
| `BUTTON_TIM_IRQ` | handler **函数名** | `lf_irq.cpp` 用它命名，所以改 TIM 时跟着改就不会漏 |

**各 TIM 对应的中断号 / handler 名（AT32F421，七个定时器全列）**

| TIM | 中断号 | handler 名（写成强符号，覆盖启动文件的弱定义） |
|-----|--------|------|
| TMR6 | `TMR6_GLOBAL_IRQn` | `TMR6_GLOBAL_IRQHandler` |
| TMR14 | `TMR14_GLOBAL_IRQn` | `TMR14_GLOBAL_IRQHandler` |
| TMR15 | `TMR15_GLOBAL_IRQn` | `TMR15_GLOBAL_IRQHandler` |
| TMR16 | `TMR16_GLOBAL_IRQn` | `TMR16_GLOBAL_IRQHandler` |
| TMR17 | `TMR17_GLOBAL_IRQn` | `TMR17_GLOBAL_IRQHandler` |
| TMR3 | `TMR3_GLOBAL_IRQn` | `TMR3_GLOBAL_IRQHandler` |
| TMR1 | `TMR1_BRK_OVF_TRG_HALL_IRQn` | `TMR1_BRK_OVF_TRG_HALL_IRQHandler` |

> 这七个实例 `timerTickStart()` 都能起（`mapTmr()` 会自动开对应外设时钟 + 配对
> 的 NVIC 中断号）；传别的指针（或传错实例）直接返回 `false`。

## 用法一：用库初始化一路 TIM

```cpp
/* 启动时 (例: app_core 的 init) */
LoveFinderLib::TIMER_Config timCfg;
timCfg.tmr      = BUTTON_TIM_INSTANCE;   /* TMR6 */
timCfg.clkHz     = BUTTON_TIM_CLK_HZ;     /* 120000000 */
timCfg.periodMs = BUTTON_TIM_PERIOD_MS;   /* 10 */
if (!LoveFinderLib::timerTickStart(timCfg)) {
    /* 分频不成立: CLK_HZ 不是 1/100/10/1k Hz 的整数倍, 或 16-bit 装不下。
     * 这时不要启用 —— 退回 SysTick 即可 (BUTTON_TIMEBASE_TIMER 0)。 */
}
```

```cpp
/* 中断: lf_irq.cpp 里 (函数名取 lf_config.h 的宏, 改 TIM 时不用改这里) */
#if BUTTON_TIMEBASE_TIMER
void BUTTON_TIM_IRQ(void)
{
    LoveFinderLib::timerTickIsr();   /* 清溢出标志 + 计时变量 += BUTTON_TIM_PERIOD_MS */
}
#endif
```

## 用法二：复用工程自己已有的 TIM

自己 `init` TIM、自己在中断里清标志，中断里加一句：

```cpp
LoveFinderLib::timerTickAdd(10);     // 每次中断自增多少 ms (自己算)
```

不要与 `timerTickStart` 混用，也不要调 `timerTickIsr()` ——
它清的是**库 init 的那路 TIM** 的标志（清错会中断风暴）。

## 接口

```cpp
bool        timerTickStart(const TIMER_Config&)  // 开外设时钟+分频+CNT 归零+溢出中断+NVIC
void        timerTickStop()                      // 关中断/计数/NVIC
void        timerTickIsr()                       // 库那路 TIM 的中断里调
void        timerTickAdd(uint32_t ms)            // 工程自己的 TIM 中断里调
uint32_t    timerNow()                           // 当前毫秒 (32-bit 读原子)
uint16_t    timerTickPeriodMs()                  // 已配的周期
bool        timerTickRunning()                   // 起来了没有
```

## 分频规则（保证时间轴不快不慢）

计数频率按 `{1MHz, 100kHz, 10kHz, 1kHz}` 逐档尝试，取第一个同时满足
「能整除 `clkHz`」「预分频 16-bit 内」「`periodMs` 内计数次数 16-bit 内」的档位
→ 实际中断周期与 `periodMs` **严格相等**，无累积误差。
四档都不成立则 `timerTickStart()` 返回 `false`（例如 `clkHz` 按错）。

例：`clkHz = 120MHz`、`periodMs = 10` → 计数频率 1MHz（预分频 119）、
周期 10000 次计数 = 10.000ms。

## 在 AT32 WorkBench 里预生成这路 TIM（可选）

`timerTickStart()` 本身就开外设时钟 + 分频 + 溢出中断 + NVIC，**完全可以不生成**。
若你更想让 WorkBench 出 `wk_tmr6_init()`，参数必须与上面的分频规则算出的值**完全一致**：

| WK 字段 | 值（120MHz 计数时钟 / 10ms） | 生成的代码 |
|---------|------------------------------|-----------|
| Mode / 模式 | Time Base（时间基），Activated = TRUE | `wk_tmr6_init()` |
| `DividerValue`（预分频） | **119** | `tmr_base_init(TMR3…, pr, 119)` 的第 3 参 |
| `Period`（周期） | **9999** | `tmr_base_init` 的第 2 参（`tmr_pr`） |

`120MHz ÷ (119+1) = 1MHz` → `1MHz ÷ (9999+1) = 100Hz` = **严格 10ms**，
与 `pickDivisors()` 给 `clkHz=120000000 / periodMs=10` 算出的 psc/pr 一模一样。
改 `BUTTON_TIM_PERIOD_MS` 时 WK 侧同步改：1ms→`119/999`，5ms→`119/4999`，20ms→`119/19999`。

> **红线**：WK 里**不要勾 TMR6 的溢出中断、也不要在 NVIC 打开 `TMR6_GLOBAL_IRQHandler`**。
> 一旦打开，WorkBench 会在 `at32f421_int.c` 生成一个**强符号** handler，
> 与工程 `lf_irq.cpp` 里 `BUTTON_TIM_IRQ` 的强符号同名 → **链接期重复定义报错**。
> 中断与 NVIC 由 `timerTickStart()` 负责，WK 只出时钟 + 分频即可。

## 注意

1. **只能占用空闲 TIM** —— `timerTickStart()` 会 `tmr_base_init()` 覆盖它。
   208/POT6 工程的 TMR3 已被 WorkBench LED PWM (`wk_tmr3_init`) 配好，别碰。
2. **handler 必须存在**，否则第一次溢出就进启动文件的弱定义 `Default_Handler`
   死循环（与 BUTTON 的 EXINT 同一坑）。
3. **`BUTTON_TIMEBASE_TIMER` 只影响 BUTTON 的时基**；`lf_get_tick()` /
   SysTick 照常工作，库里 I2C / ST7735 超时、UI 的 `navTick` 都还用它 ——
   系统里会同时存在两个时间轴，不要把两者的读数混着算。
4. 起失败时 BUTTON 会**自动退回** `lf_get_tick()`（不会因此哑掉）。
