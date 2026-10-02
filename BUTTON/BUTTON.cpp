/**
 * @file    BUTTON.cpp
 * @brief   按键驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * 状态机 (全部在主循环 update() 里推进; 中断只置 flag):
 *
 *   IDLE ──(按下沿 flag)──> DEBOUNCE ──(消抖后仍按下)──> PRESSED
 *                              │                            │
 *                              └──(已松开/抖动)──> IDLE     ├─(释放沿或电平低, 稳定 debounceMs)
 *                                                           │        └─(释放消抖窗)──> RELEASE
 *                                                           ├─(短按释放)──> WAIT_CLICK
 *                                                           │                   │
 *                                                           │                   ├─(超时)──> CLICK ──> IDLE
 *                                                           │                   └─(又按下)──> DEBOUNCE ──> DOUBLE_CLICK
 *                                                           └─(持续按住达阈值)──> LONG_PRESS (触发一次)
 *
 * 时基: buttonNow() —— 默认工程 SysTick (lf_get_tick), 可切一路 TIM (TIMER 模块)。
 */
#include "BUTTON.hpp"

/* BSP 头 (EXINT/SCFG/NVIC): 不能只赌 at32f421_conf.h 的模块开关, 全部列明 */
#include "at32f421_exint.h"
#include "at32f421_scfg.h"
#include "at32f421_misc.h"

#if BUTTON_TIMEBASE_TIMER
#include "TIMER.hpp"      /* BUTTON_TIMEBASE_TIMER=1 时, 按键时基改用 TIMER 模块 */
#endif

namespace LoveFinderLib {

/*============================================================================
 * 时基 —— onExti() 的边沿时刻与 update() 的消抖窗口都读这里
 *============================================================================*/
std::uint32_t buttonNow() noexcept
{
#if BUTTON_TIMEBASE_TIMER
    /* 工程选了一路 TIM 当时基; 若 timerTickStart 没起来 (CLK_HZ 不合法、
     * 分频凑不出精确周期), 自动退回 SysTick, 保证按键不会因此哑掉。 */
    if (timerTickRunning()) {
        return timerNow();
    }
#endif
    return lf_get_tick();   /* 默认: 工程 SysTick 1ms */
}

/*============================================================================
 * EXINT 配置 — 直调 AT32 BSP (原平台层 lf_gpio_config_exti 的实现内联于此)
 *
 * AT32F421 的 EXINT 是分组中断:
 *   EXINT1_0_IRQn  — 引脚 0~1
 *   EXINT3_2_IRQn  — 引脚 2~3
 *   EXINT15_4_IRQn — 引脚 4~15
 * 配置链: GPIO 输入 + 上下拉 → SCFG 线绑定 → EXINT 触发沿 → NVIC 使能。
 *============================================================================*/
namespace {

/** 引脚掩码 → 引脚序号 (0..15)。掩码必须是单一 bit。 */
std::uint8_t pinIndex(std::uint16_t pinMask) noexcept
{
    std::uint8_t n = 0;
    std::uint16_t m = pinMask;
    while ((m & 1u) == 0u && n < 16u) {
        m >>= 1u;
        ++n;
    }
    return n;
}

/** 引脚序号 → SCFG 端口源。AT32F421 只有 GPIOA/B/C/F。 */
scfg_port_source_type portSourceOf(gpio_type *port) noexcept
{
    if (port == GPIOA) return SCFG_PORT_SOURCE_GPIOA;
    if (port == GPIOB) return SCFG_PORT_SOURCE_GPIOB;
    if (port == GPIOC) return SCFG_PORT_SOURCE_GPIOC;
    if (port == GPIOF) return SCFG_PORT_SOURCE_GPIOF;
    return SCFG_PORT_SOURCE_GPIOA;
}

/** 引脚序号 → NVIC 中断号 (AT32 分组中断)。 */
IRQn_Type irqOfPin(std::uint8_t n) noexcept
{
    if (n <= 1u)  return EXINT1_0_IRQn;
    if (n <= 3u)  return EXINT3_2_IRQn;
    return EXINT15_4_IRQn;
}

} // namespace

void Button::configureExti() noexcept
{
    if (m_port == nullptr) return;

    /* activeLow (按下接地) → 下降沿触发 */
    const bool fallingEdge = m_config.activeLow;
    const std::uint8_t n = pinIndex(m_pin);

    /* 1. 配置引脚为输入 + 上拉 (按键按下接地) */
    gpio_init_type cfg;
    gpio_default_para_init(&cfg);
    cfg.gpio_pins           = m_pin;
    cfg.gpio_mode           = GPIO_MODE_INPUT;
    cfg.gpio_pull           = fallingEdge ? GPIO_PULL_UP : GPIO_PULL_DOWN;
    cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(m_port, &cfg);

    /* 2. 绑定 EXINT 线到该端口 */
    scfg_exint_line_config(portSourceOf(m_port),
                           static_cast<scfg_pins_source_type>(n));

    /* 3. 配置 EXINT 触发沿
     *    bothEdges (默认) = 上下沿都触发: 中断能区分按下/释放, 边沿时刻由中断给出
     *    false            = 只触发 activeLow 对应的那条沿 (老行为, 释放靠轮询兜底) */
    exint_init_type exintCfg;
    exint_default_para_init(&exintCfg);
    exintCfg.line_enable   = TRUE;
    exintCfg.line_mode     = EXINT_LINE_INTERRUPT;
    exintCfg.line_select   = static_cast<std::uint32_t>(EXINT_LINE_0) << n;
    if (m_config.bothEdges) {
        exintCfg.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
    } else {
        exintCfg.line_polarity = fallingEdge ? EXINT_TRIGGER_FALLING_EDGE
                                             : EXINT_TRIGGER_RISING_EDGE;
    }
    exint_init(&exintCfg);

    /* 4. 使能 NVIC */
    nvic_irq_enable(irqOfPin(n), 0, 0);
}

/*============================================================================
 * EXINT 中断回调 —— ★ 中断里只碰 flag, 不做任何状态判定
 *
 * 三步: 清挂起标志 → 读一次电平辨认沿的类型 → 记下边沿时刻。
 * 谁来消费这些 flag 是主循环 update() 的事 (消抖窗、状态机都在那里)。
 *
 * 读电平必须在清标志之后: 若与清标志之间又来一个沿, 会再进一次中断补上,
 * 不会丢沿; 电平取的是「当前」值, 与最后一次沿一致。
 *============================================================================*/
void Button::onExti() noexcept
{
    if (m_port != nullptr) {
        exint_flag_clear(static_cast<std::uint32_t>(EXINT_LINE_0) << pinIndex(m_pin));
    }

    const bool pressed = isPressed();     /* 边沿后的电平 → 判定沿类型 */

    m_isrTick    = buttonNow();
    m_isrPressed = pressed;

    if (pressed) {
        m_pressPending = true;            /* 按下沿: 唤醒状态机 */
        m_pressTick    = m_isrTick;       /* 按压时长的起点 */
    } else {
        m_releasePending = true;          /* 释放沿: 起释放消抖窗 */
    }
}

/*============================================================================
 * 事件推送
 *============================================================================*/
void Button::pushEvent(e_BUTTON_Event e) noexcept
{
    m_lastEvent = e;
    m_hasEvent  = true;
    m_flags     = static_cast<std::uint8_t>(m_flags | flagOf(e));
}

e_BUTTON_Event Button::getEvent() noexcept
{
    const e_BUTTON_Event e = m_lastEvent;
    m_lastEvent = e_BUTTON_Event::NONE;
    m_hasEvent  = false;
    return e;
}

/*============================================================================
 * 主循环状态机
 *============================================================================*/
e_BUTTON_Event Button::update() noexcept
{
    const std::uint32_t now = buttonNow();   /* 与 onExti() 记的边沿时刻同源 */
    e_BUTTON_Event event = e_BUTTON_Event::NONE;

    switch (m_state) {
    /*------------------------------------------------------------------
     * IDLE: 等待中断置位
     *----------------------------------------------------------------*/
    case e_BUTTON_State::IDLE:
        if (m_pressPending) {
            m_pressPending = false;
            m_state        = e_BUTTON_State::DEBOUNCE;
            m_debounceTick = now;
        }
        break;

    /*------------------------------------------------------------------
     * DEBOUNCE: 消抖窗口结束后确认电平
     *----------------------------------------------------------------*/
    case e_BUTTON_State::DEBOUNCE:
        if ((now - m_debounceTick) >= m_config.debounceMs) {
            m_pressPending = false;

            if (isPressed()) {
                /* 确认按下 (此刻电平是按下, 之前残留的释放沿一律作废) */
                m_state          = e_BUTTON_State::PRESSED;
                m_releasePending = false;
                m_pressTime      = now;
                m_longPressFired = false;
                m_longPressActive = false;
                m_chargeTick     = now;

                /* 若处于「等待第二次点击」, 说明这是双击的第二下 */
                if (m_waitingSecondClick) {
                    m_waitingSecondClick = false;
                    ++m_counter.doubleClickCount;
                    event = e_BUTTON_Event::DOUBLE_CLICK;
                    pushEvent(event);
                    m_state = e_BUTTON_State::PRESSED;   /* 继续跟踪长按/释放 */
                } else {
                    event = e_BUTTON_Event::PRESS_DOWN;
                    pushEvent(event);
                }
            } else {
                /* 抖动, 回到空闲 */
                m_waitingSecondClick = false;
                m_state = e_BUTTON_State::IDLE;
            }
        }
        break;

    /*------------------------------------------------------------------
     * PRESSED: 跟踪长按与释放
     *
     * 释放来源两个, 都要过一遍消抖窗 (m_releaseWait):
     *   - 中断给的释放沿 (bothEdges=1, 精确时刻 = m_isrTick)
     *   - 电平轮询兜底   (单沿模式 / 中断丢失时, 时刻取本次 update 的 now)
     * 窗内若电平又按回去 → 取消, 继续算按住; 稳定 debounceMs 才提交释放。
     *----------------------------------------------------------------*/
    case e_BUTTON_State::PRESSED: {
        bool releaseEdge = false;
        if (m_releasePending) {
            m_releasePending = false;
            releaseEdge      = true;
        }

        if (m_releaseWait) {
            if (isPressed()) {
                m_releaseWait = false;                    /* 抖动回弹 → 继续按住 */
            } else if ((now - m_debounceTick) >= m_config.debounceMs) {
                event = commitRelease(now);               /* 稳定释放 → 提交 */
            }
            break;
        }

        if (releaseEdge || !isPressed()) {
            m_releaseWait  = true;                        /* 起释放消抖窗 */
            m_releaseTick  = releaseEdge ? m_isrTick : now;
            m_debounceTick = now;
            if (isPressed()) m_releaseWait = false;       /* 沿到时已回弹 → 不算释放 */
            break;
        }

        /* 仍按住: 检查长按阈值 */
        if (!m_longPressFired && (now - m_pressTime) >= m_config.longPressMs) {
            m_longPressFired  = true;
            m_longPressActive = true;
            m_chargeTick      = now;
            ++m_counter.longPressCount;
            event = e_BUTTON_Event::LONG_PRESS;
            pushEvent(event);
        }
        break;
    }

    /*------------------------------------------------------------------
     * WAIT_CLICK: 双击判定窗口
     *----------------------------------------------------------------*/
    case e_BUTTON_State::WAIT_CLICK:
        if (m_pressPending) {
            /* 窗口内又按下 → 可能是双击, 走消抖 */
            m_pressPending       = false;
            m_waitingSecondClick = true;
            m_state              = e_BUTTON_State::DEBOUNCE;
            m_debounceTick       = now;
        } else if ((now - m_debounceTick) >= m_config.doubleClickMs) {
            /* 窗口超时 → 判定为单击 */
            ++m_counter.clickCount;
            event = e_BUTTON_Event::CLICK;
            pushEvent(event);
            m_state = e_BUTTON_State::IDLE;
        } else if (isPressed()) {
            /* 边沿中断可能丢失, 用轮询兜底 */
            m_waitingSecondClick = true;
            m_state              = e_BUTTON_State::DEBOUNCE;
            m_debounceTick       = now;
        }
        break;
    }

    updateCharge(now);
    return event;
}

/*============================================================================
 * 释放提交 —— PRESSED 的释放消抖窗稳定通过后调用
 *
 * 与旧版「已释放」分支等价, 另外把按压时长结算掉。
 *============================================================================*/
e_BUTTON_Event Button::commitRelease(std::uint32_t now) noexcept
{
    m_pressDuration = (m_releaseTick >= m_pressTick)
                    ? (m_releaseTick - m_pressTick)
                    : 0u;

    m_longPressActive = false;
    m_decayTick       = now;
    m_pressPending    = false;
    m_releaseWait     = false;

    if (m_longPressFired) {
        m_state = e_BUTTON_State::IDLE;          /* 长按后释放: 不再产生单击 */
    } else {
        m_state        = e_BUTTON_State::WAIT_CLICK;   /* 短按: 等第二次点击 */
        m_debounceTick = now;
    }

    const e_BUTTON_Event e = e_BUTTON_Event::RELEASE;
    pushEvent(e);
    return e;
}

/*============================================================================
 * 长按充能
 *
 * 长按期间: 按 chargeFullMs 线性 0 → 100
 * 释放后:   按 decayMs    线性衰减回 0
 *============================================================================*/
void Button::updateCharge(std::uint32_t now) noexcept
{
    if (m_longPressActive) {
        const std::uint32_t dt = now - m_chargeTick;
        if (dt == 0u) return;

        const std::uint32_t add =
            static_cast<std::uint32_t>((static_cast<std::uint64_t>(dt) * 100u) /
                                       m_config.chargeFullMs);
        if (add == 0u) return;

        m_chargeTick = now;
        const std::uint32_t level = static_cast<std::uint32_t>(m_chargePercent) + add;

        if (level >= 100u) {
            /* 每满一圈计一次长按 */
            const std::uint32_t fills = level / 100u;
            m_counter.longPressCount += fills;
            m_chargePercent = static_cast<std::uint16_t>(level % 100u);
        } else {
            m_chargePercent = static_cast<std::uint16_t>(level);
        }
    } else if (!isPressed() && m_chargePercent > 0u) {
        const std::uint32_t dt = now - m_decayTick;
        if (dt == 0u) return;

        const std::uint32_t sub =
            static_cast<std::uint32_t>((static_cast<std::uint64_t>(dt) * 100u) /
                                       m_config.decayMs);
        if (sub == 0u) return;

        m_decayTick = now;
        m_chargePercent = (sub >= m_chargePercent)
                        ? 0u
                        : static_cast<std::uint16_t>(m_chargePercent - sub);
    }
}

} // namespace LoveFinderLib
