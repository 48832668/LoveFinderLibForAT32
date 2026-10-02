# BUTTON — 配置引导 (AT32)

按键库：单击 / 双击 / 长按 + 长按充能。基于 EXINT 中断 + 主循环状态机。

**两段式模型（关键）**：中断 `onExti()` **只置标志**（清挂起位 → 读一次电平
辨认「按下沿 / 释放沿」→ 记下边沿时刻），一个判定都不做；消抖、单击/双击/长按
判定、充能全部在主循环 `update()` 里跑。EXINT 默认**上下沿都触发**。

## 依赖

- `lf_config.h`（工程侧配置头：引脚宏 + 时基声明）
- 引脚必须支持 EXINT 外部中断
- 时基默认 `lf_get_tick()`（SysTick 1ms）；想改用一路 TIM 见下方「时基」

## 在项目 `lf_config.h` 中配置

```cpp
/* ---- 引脚 ---- */
#define BUTTON_KEY_PORT             KEY_GPIO_PORT   /* 来自 at32f421_wk_config.h */
#define BUTTON_KEY_PIN              KEY_PIN

/* ---- 时序阈值 (ms), 均有缺省值, 不写也可 ---- */
#define BUTTON_DEBOUNCE_MS_DEFAULT      20
#define BUTTON_LONG_PRESS_MS_DEFAULT    1000
#define BUTTON_DOUBLE_CLICK_MS_DEFAULT  300
#define BUTTON_CHARGE_FULL_MS_DEFAULT   2000    /* 长按充能 0→100 的时间 */
#define BUTTON_DECAY_MS_DEFAULT         3000    /* 松开后充能衰减时间 */
#define BUTTON_ACTIVE_LOW_DEFAULT       1       /* 1 = 按下接地 (需上拉) */
#define BUTTON_BOTH_EDGES_DEFAULT       1       /* 1 = 上下沿都触发 (默认) */

/* ---- 时基 (可选, 默认 SysTick) ---- */
#define BUTTON_TIMEBASE_TIMER           0       /* 0=lf_get_tick() / 1=一路 TIM */
```

> 引脚宏不是必须的：`init()` 直接收 `(gpio_type *port, uint16_t pin)`，可以直接写
> `key.init(GPIOB, GPIO_PINS_1)`。上面两个宏只是方便集中管理。

## 使用

```cpp
#include "BUTTON.hpp"
using namespace LoveFinderLib;

Button key;

void setup() {
    key.init(BUTTON_KEY_PORT, BUTTON_KEY_PIN);
}

/* 主循环, 建议 5ms 调用一次 */
void loop() {
    switch (key.update()) {
        case e_BUTTON_Event::CLICK:        /* 单击 */ break;
        case e_BUTTON_Event::DOUBLE_CLICK: /* 双击 */ break;
        case e_BUTTON_Event::LONG_PRESS:   /* 长按 (触发一次) */ break;
        case e_BUTTON_Event::PRESS_DOWN:   /* 按下 */ break;
        case e_BUTTON_Event::RELEASE:      /* 释放 */ break;
        default: break;
    }
}

/* EXINT 中断服务函数中 */
void EXINT1_0_IRQHandler(void) {
    if (exint_interrupt_flag_get(EXINT_LINE_1) != RESET) {
        key.onExti();
    }
}
```

## 中断挂接

AT32F421 的 EXINT 是**分组中断**：

| 引脚范围 | 中断号 |
|----------|--------|
| 0 ~ 1 | `EXINT1_0_IRQn` |
| 2 ~ 3 | `EXINT3_2_IRQn` |
| 4 ~ 15 | `EXINT15_4_IRQn` |

`init()` 会自动完成：GPIO 输入+上拉 → SCFG 线绑定 → EXINT 触发沿 → NVIC 使能
（EXINT 配置代码内联在 `BUTTON.cpp`，直调 `gpio_*/scfg_*/exint_*/nvic_*` BSP）。
`onExti()` 内部已调用 `exint_flag_clear()` 清标志，**中断里不要再手动清**。

### 中断模型：中断只碰标志

```
EXINT 中断 → onExti():   ① 清 EXINT 挂起标志
                         ② 读一次电平 → 按下沿 (电平=按住) 还是释放沿 (电平=松开)
                         ③ 记边沿时刻 buttonNow()
                         —— 三步全是 volatile 标志/时间戳, 不做判定
主循环     → update():   消抖窗、状态机、充能; 标志没来时用电平轮询兜底
```

`bothEdges = true`（默认）时两条沿都触发 EXINT：
- **按下沿** → `m_pressPending`，唤醒 `IDLE/WAIT_CLICK` 的状态机
- **释放沿** → `m_releasePending`，用来起**释放消抖窗**并拿到精确释放时刻

`bothEdges = false`（老行为）只触发 `activeLow` 对应的那条沿，
释放靠 `update()` 轮询电平兜底，功能不变、只是时序精度略低。

### 释放也消抖

进入释放后要电平**连续稳定 `debounceMs`** 才提交 `RELEASE`；窗内若电平又按回去
（回弹）就取消，继续算按住。因此 `RELEASE` / `CLICK` 会比旧版晚约 `debounceMs`
（20ms），换来「按一下被判成两下」的毛刺消失。`PRESS_DOWN` / `LONG_PRESS` 时序不变。

### 时基（可选走一路 TIM）

默认 `lf_get_tick()`（SysTick 1ms 中断，分辨率最细）。想让按键脱离 SysTick：

```cpp
#define BUTTON_TIMEBASE_TIMER  1
#define BUTTON_TIM_INSTANCE    TMR6
#define BUTTON_TIM_CLK_HZ      120000000
#define BUTTON_TIM_PERIOD_MS   10
#define BUTTON_TIM_IRQ         TMR6_GLOBAL_IRQHandler
```

再按 `../TIMER/CONFIG_GUIDE.md` 接 `timerTickStart()` 与该 TIM 的中断。
两条路都经 `buttonNow()` 读取，消抖窗口与边沿时间戳自动跟着换；
`timerTickStart` 起失败时会自动退回 `lf_get_tick()`。

## 长按充能

长按触发 `LONG_PRESS` 后开始计时，`getChargePercent()` 从 0 线性涨到 100
（`chargeFullMs`），每满 100 计一次 `longPressCount` 并回绕。松开后按
`decayMs` 衰减回 0。可用于「长按调亮度 / 连续递增设定值」这类交互。

```cpp
uint8_t pct  = key.getChargePercent();   // 0~100
bool    full = key.isChargeFull();
```

## 事件消费的三种风格

```cpp
/* 风格 1: 返回值 (推荐) */
switch (key.update()) { ... }

/* 风格 2: 标志位 (适合多处消费者) */
uint8_t flags = key.getAndClearFlags();
if (flags & static_cast<uint8_t>(BUTTON_Flag::CLICK)) { ... }

/* 风格 3: 累计计数 */
uint32_t n = key.getClickCount();
```

## 边沿时刻（诊断用）

```cpp
uint32_t pressMs = key.getPressDurationMs();   // 最近一次完整按压时长 (释放-按下)
uint32_t edgeMs  = key.getLastEdgeTick();      // 中断侧最近一次边沿的时刻
bool     wasDown = key.wasPressedAtLastEdge(); // 那次边沿后的电平 (true=按下)
```

时长由中断记的两端边沿相减得出，与主循环被阻塞多久无关
（`timerTickAdd`/`buttonNow` 同源）。

## 已知约束

- 引脚配置（上拉/触发沿）由 `activeLow` 决定：`activeLow=true` → 上拉。
  若硬件是「按下接 VCC」，须把 `activeLow` 设为 `false`（内部下拉）。
  `bothEdges=true` 时上升/下降沿都进中断，pull 仍按 `activeLow` 选。
- `update()` 的调用周期应远小于 `debounceMs`，建议 1~10ms。周期过长会漏判双击，
  也拉长释放消抖窗（窗按「调用时刻」检查）。
- 释放消抖让 `RELEASE`/`CLICK` 比旧版晚约 `debounceMs`（见上），
  `POT6`/`891`/`208` 的消费方式（只吃 CLICK/DOUBLE_CLICK/LONG_PRESS）不受影响。
- **时基选 TIM 时**：`BUTTON_TIMEBASE_TIMER=1` 只换按键的时基，
  `lf_get_tick()` / SysTick 照常给 I2C、ST7735 超时用 —— 系统里会有两个时间轴，
  不要把两边的读数混着算。
