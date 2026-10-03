/**
 * @file    BUTTON.hpp
 * @brief   按键驱动 (EXINT 中断 + 状态机) — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  功能
 * ============================================================================
 *  - 单击 / 双击 / 长按 (按住即触发 + 持续充能进度)
 *  - 消抖 (按下沿 + 释放沿都要过消抖窗)
 *  - 上下沿都触发 EXINT (默认, 可配回单沿)
 *  - 按下/释放边沿事件 + 按压时长
 *  - 事件计数与标志位 (两种消费风格)
 *
 * ============================================================================
 *  两段式模型 —— ★ 中断里只碰 flag, 判定集中在状态机一步里
 * ============================================================================
 *  EXINT 中断 → onExti()  :  ① 清 EXINT 挂起标志
 *                            ② 读一次电平, 认出这是「按下沿」还是「释放沿」
 *                            ③ 记下边沿时刻
 *                            —— 仅此三步, 全是 volatile flag / 时间戳。
 *  状态机一步 → update() / tickIsr()  :
 *                            消抖、单击/双击/长按判定、充能衰减,
 *                            全部基于上面那些 flag + 电平轮询兜底。
 *                            ★ 两者是【同一个状态机】, 只是调用位置不同:
 *                              · 主循环模式: 主循环里调 update() (返回事件)
 *                              · 中断模式  : 周期性中断里调 tickIsr() ——
 *                                判定完全不依赖主循环, 主循环长阻塞也不丢按键
 *  事件读取   → getEvent() / getAndClearFlags() / getCounter()
 *
 *  断言不变式:
 *    ① 除了 pushEvent(), 状态机只在 update() / tickIsr() 里推进;
 *    ② onExti() 与状态机之间用【快照】交接 —— 读标志 + 清标志 + 取边沿时刻
 *       整段放进临界区, 所以两侧放主循环还是中断都安全, 不会丢沿;
 *    ③ 事件读取 (getEvent / getAndClearFlags) 也带临界区, 可与中断并存。
 *
 * ============================================================================
 *  时基 (★ 可选 SysTick / 一路 TIM)
 * ============================================================================
 *  默认走工程侧 lf_get_tick() (SysTick 1ms 中断, 分辨率更细)。
 *  想脱离 SysTick → 在 lf_config.h 里
 *      #define BUTTON_TIMEBASE_TIMER   1
 *      #define BUTTON_TIM_INSTANCE     TMR6
 *      #define BUTTON_TIM_CLK_HZ       120000000
 *      #define BUTTON_TIM_PERIOD_MS    10
 *  再配合 TIMER 模块 (timerTickStart + 该 TIM 的中断里调 timerTickIsr)。
 *  两条路都经 buttonNow() 读取 → 消抖/时间戳自动跟着换时基。
 *
 * ============================================================================
 *  使用步骤
 * ============================================================================
 *  1. 创建对象并 init(引脚, 配置) —— 内部会配置 GPIO + EXINT + NVIC
 *  2. 在 EXINT 中断中调用 button.onExti()   (只置 flag)
 *  3. 主循环每 1~10ms 调用 button.update()  (全部判定在这)
 *  4. 读取事件: update() 返回值 / getAndClearFlags() / getCounter()
 *
 * ============================================================================
 *  典型用法
 * ============================================================================
 *    LoveFinderLib::Button key;
 *    key.init({BUTTON_KEY_PORT, BUTTON_KEY_PIN});
 *    ...
 *    switch (key.update()) {
 *      case e_BUTTON_Event::CLICK:        ... break;
 *      case e_BUTTON_Event::DOUBLE_CLICK: ... break;
 *      case e_BUTTON_Event::LONG_PRESS:   ... break;
 *      default: break;
 *    }
 */
#ifndef LOVE_FINDER_LIB_BUTTON_HPP
#define LOVE_FINDER_LIB_BUTTON_HPP

#include "lf_config.h"      /* ★ 工程侧: BUTTON_KEY_* 引脚缺省 + lf_get_tick() 时基 */

/* ★ 自带所需 BSP 头 (不能只赌 at32f421_conf.h 的模块开关: conf.h 按
 *   GPIO_MODULE_ENABLED 等宏条件包含, 少开一个模块本文件就编不过)。 */
#include "at32f421_gpio.h"   /* gpio_type / gpio_input_data_bit_read (isPressed 内联用) */

#include <cstdint>

namespace LoveFinderLib {

/*============================================================================
 * 编译期默认阈值 (可在 lf_config.h 中覆盖)
 *============================================================================*/
#ifndef BUTTON_LONG_PRESS_MS_DEFAULT
#define BUTTON_LONG_PRESS_MS_DEFAULT    1000
#endif
#ifndef BUTTON_DOUBLE_CLICK_MS_DEFAULT
#define BUTTON_DOUBLE_CLICK_MS_DEFAULT  300
#endif
#ifndef BUTTON_DEBOUNCE_MS_DEFAULT
#define BUTTON_DEBOUNCE_MS_DEFAULT      20
#endif
#ifndef BUTTON_CHARGE_FULL_MS_DEFAULT
#define BUTTON_CHARGE_FULL_MS_DEFAULT   2000
#endif
#ifndef BUTTON_DECAY_MS_DEFAULT
#define BUTTON_DECAY_MS_DEFAULT         3000
#endif
#ifndef BUTTON_ACTIVE_LOW_DEFAULT
#define BUTTON_ACTIVE_LOW_DEFAULT       1
#endif
/** 上下沿都触发 EXINT (1, 默认) / 只触发 activeLow 对应的那一条沿 (0)。 */
#ifndef BUTTON_BOTH_EDGES_DEFAULT
#define BUTTON_BOTH_EDGES_DEFAULT       1
#endif
/**
 * 按键时基: 0 = lf_get_tick() (SysTick 1ms, 默认)
 *           1 = TIMER 模块的一路 TIM 中断 (需配 BUTTON_TIM_* 宏, 见文件头)
 */
#ifndef BUTTON_TIMEBASE_TIMER
#define BUTTON_TIMEBASE_TIMER           0
#endif

/*============================================================================
 * 时基
 *============================================================================*/
/**
 * @brief 按键时基读数 —— onExti() 记的边沿时刻与 update() 的消抖窗口同源。
 *
 * BUTTON_TIMEBASE_TIMER=0 → lf_get_tick();  =1 → LoveFinderLib::timerNow()。
 */
std::uint32_t buttonNow() noexcept;

/*============================================================================
 * 事件与状态
 *============================================================================*/
enum class e_BUTTON_Event : std::uint8_t {
    NONE         = 0,
    CLICK        = 1,   ///< 单击
    DOUBLE_CLICK = 2,   ///< 双击
    LONG_PRESS   = 3,   ///< 长按 (达到阈值时触发一次)
    PRESS_DOWN   = 4,   ///< 按下 (消抖后)
    RELEASE      = 5    ///< 释放
};

/** 事件标志位 (与 getAndClearFlags() 配合使用)。 */
enum class BUTTON_Flag : std::uint8_t {
    PRESS_DOWN   = 0x01,
    CLICK        = 0x02,
    DOUBLE_CLICK = 0x04,
    LONG_PRESS   = 0x08,
    RELEASE      = 0x10
};

[[nodiscard]] inline constexpr std::uint8_t flagOf(e_BUTTON_Event e) noexcept
{
    return e == e_BUTTON_Event::PRESS_DOWN   ? static_cast<std::uint8_t>(BUTTON_Flag::PRESS_DOWN)
         : e == e_BUTTON_Event::CLICK        ? static_cast<std::uint8_t>(BUTTON_Flag::CLICK)
         : e == e_BUTTON_Event::DOUBLE_CLICK ? static_cast<std::uint8_t>(BUTTON_Flag::DOUBLE_CLICK)
         : e == e_BUTTON_Event::LONG_PRESS   ? static_cast<std::uint8_t>(BUTTON_Flag::LONG_PRESS)
         : e == e_BUTTON_Event::RELEASE      ? static_cast<std::uint8_t>(BUTTON_Flag::RELEASE)
         : 0u;
}

enum class e_BUTTON_State : std::uint8_t {
    IDLE       = 0,   ///< 空闲
    DEBOUNCE   = 1,   ///< 消抖中
    PRESSED    = 2,   ///< 已按下
    WAIT_CLICK = 3    ///< 等待第二次点击 (判定双击)
};

/*============================================================================
 * 配置与计数
 *============================================================================*/
struct BUTTON_Config {
    std::uint16_t debounceMs    = BUTTON_DEBOUNCE_MS_DEFAULT;
    std::uint16_t longPressMs   = BUTTON_LONG_PRESS_MS_DEFAULT;
    std::uint16_t doubleClickMs = BUTTON_DOUBLE_CLICK_MS_DEFAULT;
    std::uint16_t chargeFullMs  = BUTTON_CHARGE_FULL_MS_DEFAULT;
    std::uint16_t decayMs       = BUTTON_DECAY_MS_DEFAULT;
    bool          activeLow     = (BUTTON_ACTIVE_LOW_DEFAULT != 0);
    /** EXINT 上下沿都触发 (默认) = 中断能区分按下/释放; false = 只触发一条沿。 */
    bool          bothEdges     = (BUTTON_BOTH_EDGES_DEFAULT != 0);

    static BUTTON_Config getDefault() noexcept { return {}; }
};

struct BUTTON_Counter {
    std::uint32_t clickCount       = 0;
    std::uint32_t doubleClickCount = 0;
    std::uint32_t longPressCount   = 0;
};

/** onExti() 置的待处理沿 + 对应边沿时刻 —— 由 takeIsrEdges() 原子取走。
 *  (三样一起读、一起清, 所以不会出现「标志拿到了但时刻被下一次边沿盖掉」) */
struct BUTTON_IsrEdges {
    bool          press   = false;   ///< 有按下沿待处理
    bool          release = false;   ///< 有释放沿待处理
    std::uint32_t tick    = 0;       ///< 最后一次边沿的时刻 (与上面两个标志同批)
};

/*============================================================================
 * Button
 *============================================================================*/
class Button {
public:
    Button() = default;

    /**
     * @param port   按键 GPIO 端口 (AT32 gpio_type*, 如 GPIOF)
     * @param pin    引脚掩码 (GPIO_PINS_x)
     * @param config 时序配置
     */
    Button(gpio_type *port, std::uint16_t pin,
           const BUTTON_Config &config = BUTTON_Config::getDefault()) noexcept
    {
        init(port, pin, config);
    }

    /**
     * @brief 完整初始化: 自己配 GPIO + SCFG + EXINT + NVIC, 然后绑定引脚。
     * @param port   按键 GPIO 端口
     * @param pin    引脚掩码
     * @param config 时序配置
     *
     * 这是【库内配置硬件】的唯一模式, 也是本库"初始化归 ATWK"原则的例外 ——
     * 理由: 按键要可靠判定按下与释放必须用【双沿】EXINT, 而 ATWK 的 EXINT
     * 配置只给单边沿, 库至少要改触发沿。详见 attach() 与 AGENTS.md §9。
     */
    void init(gpio_type *port, std::uint16_t pin, const BUTTON_Config &config) noexcept
    {
        m_port   = port;
        m_pin    = pin;
        m_config = config;
        reset();
        configureExti();
    }

    void init(gpio_type *port, std::uint16_t pin) noexcept
    {
        init(port, pin, BUTTON_Config::getDefault());
    }

    /**
     * @brief 轻量绑定: 硬件已由 ATWK 配好, 本类【不写任何硬件寄存器】。
     *
     * 与 init() 的唯一差别: 不调用 configureExti()。绑定后用法完全一样
     * (主循环 update() / 中断里 onExti())。
     *
     * ⚠ ATWK 侧必须把该引脚配成:
     *    - GPIO : 输入模式 (activeLow 时上拉, 否则下拉)
     *    - EXINT: 使能 + 【上下沿都触发】+ NVIC 使能
     *   若 ATWK 只配了单边沿, 按键仍能用 (释放靠 update() 轮询兜底),
     *   但中断辨不出释放沿, 长按/双击的时刻精度会下降。
     *   config 在此模式下只影响软件语义 (消抖 / 长按 / 双击时长), 不碰硬件。
     */
    void attach(gpio_type *port, std::uint16_t pin,
                const BUTTON_Config &config = BUTTON_Config::getDefault()) noexcept
    {
        m_port   = port;
        m_pin    = pin;
        m_config = config;
        reset();
    }


    void reset() noexcept
    {
        m_state              = e_BUTTON_State::IDLE;
        m_pressPending       = false;
        m_releasePending     = false;
        m_pressTick          = 0;
        m_isrTick            = 0;
        m_debounceTick       = 0;
        m_pressTime          = 0;
        m_releaseTick        = 0;
        m_pressDuration      = 0;
        m_releaseWait        = false;
        m_waitingSecondClick = false;
        m_longPressFired     = false;
        m_longPressActive    = false;
        m_chargePercent      = 0;
        m_chargeTick         = 0;
        m_decayTick          = 0;
        m_counter            = {};
        m_lastEvent          = e_BUTTON_Event::NONE;
        m_hasEvent           = false;
        m_flags              = 0;
    }

    /*---------- 状态机 (二选一: 主循环 或 周期性中断) ----------*/
    /** 推进一次状态机, 返回本次产生的事件。
     *  【主循环模式】在主循环里定期调用 (建议 1~10ms)。
     *  中断里也能调, 但中断模式请用语义更明确的 tickIsr()。 */
    e_BUTTON_Event update() noexcept;

    /** 推进一次状态机, 不返回事件 —— 【中断模式】专用。
     *
     *  在周期性中断里调用 (建议 1~10ms, 例如 1ms 的 SysTick)。
     *  判定从此完全脱离主循环: 主循环哪怕阻塞几百 ms, 消抖 / 双击窗口 /
     *  长按计时照样按真实时间推进。
     *
     *  ★ 事件不再靠返回值拿 (中断里没法把返回值交给别人)。请在主循环用
     *    getEvent() / getAndClearFlags() / getCounter() 读取, 都带临界区。
     *  ★ 别和 update() 混用同一实例: 两者是同一个状态机, 两个地方一起驱动
     *    会让时间判定乱掉。中断模式就只在中断里调 tickIsr()。 */
    void tickIsr() noexcept { (void)update(); }

    [[nodiscard]] bool hasEvent() const noexcept { return m_hasEvent; }

    /** 取走最近一次事件并清除 hasEvent。 */
    e_BUTTON_Event getEvent() noexcept;

    /*---------- 事件计数 ----------*/
    [[nodiscard]] const BUTTON_Counter &getCounter() const noexcept { return m_counter; }
    void resetCounter() noexcept { m_counter = {}; }
    [[nodiscard]] std::uint32_t getClickCount() const noexcept { return m_counter.clickCount; }
    [[nodiscard]] std::uint32_t getDoubleClickCount() const noexcept { return m_counter.doubleClickCount; }
    [[nodiscard]] std::uint32_t getLongPressCount() const noexcept { return m_counter.longPressCount; }

    /*---------- 事件标志位 ----------*/
    void setFlag(e_BUTTON_Event e) noexcept
    {
        m_flags = static_cast<std::uint8_t>(m_flags | flagOf(e));
    }
    [[nodiscard]] std::uint8_t peekFlags() const noexcept { return m_flags; }
    [[nodiscard]] bool testFlag(BUTTON_Flag f) const noexcept
    {
        return (m_flags & static_cast<std::uint8_t>(f)) != 0u;
    }
    /** 取走并清除全部事件标志位 (临界区内读+清, 可与中断并存)。 */
    std::uint8_t getAndClearFlags() noexcept;
    void clearFlags() noexcept { m_flags = 0; }

    /*---------- 物理状态 ----------*/
    /** 实时读取引脚电平 (不消抖)。 */
    [[nodiscard]] bool isPressed() const noexcept
    {
        const bool level = (m_port != nullptr) &&
                           (gpio_input_data_bit_read(m_port, m_pin) != RESET);
        return m_config.activeLow ? !level : level;
    }

    [[nodiscard]] e_BUTTON_State getState() const noexcept { return m_state; }
    [[nodiscard]] gpio_type *getPort() const noexcept { return m_port; }
    [[nodiscard]] std::uint16_t getPin() const noexcept { return m_pin; }

    /*---------- 运行时阈值 ----------*/
    void setLongPressMs(std::uint16_t ms) noexcept { m_config.longPressMs = ms; }
    [[nodiscard]] std::uint16_t getLongPressMs() const noexcept { return m_config.longPressMs; }
    void setDoubleClickMs(std::uint16_t ms) noexcept { m_config.doubleClickMs = ms; }
    [[nodiscard]] std::uint16_t getDoubleClickMs() const noexcept { return m_config.doubleClickMs; }

    /*---------- EXINT ----------*/
    /** 配置 GPIO + EXINT + NVIC (init 会自动调用)。 */
    void configureExti() noexcept;

    /** 在 EXINT 中断服务函数中调用。 */
    void onExti() noexcept;

    /*---------- 充能 (长按期间 0~100 递增, 松开后衰减) ----------*/
    [[nodiscard]] std::uint8_t getChargePercent() const noexcept
    {
        return static_cast<std::uint8_t>(m_chargePercent);
    }
    /** 长按充能是否已满 (达到 100)。 */
    [[nodiscard]] bool isChargeFull() const noexcept { return m_chargePercent >= 100u; }

    /*---------- 边沿时刻 (bothEdges=1 时释放沿由中断给出) ----------*/
    /**
     * @brief 最近一次【完整按压】的时长 (ms) = 释放时刻 - 按下时刻。
     *
     * 两条时基同源, 由中断记的边沿时刻算出, 与主循环被阻塞多久无关。
     * 还没跑完一次按压 (未释放) 时为 0。
     */
    [[nodiscard]] std::uint32_t getPressDurationMs() const noexcept { return m_pressDuration; }

    /** 中断侧最近一次边沿的时刻 (buttonNow() 同源), 供诊断/自检用。 */
    [[nodiscard]] std::uint32_t getLastEdgeTick() const noexcept { return m_isrTick; }

    /** 最近一次边沿后的电平 (true = 按下), 与 onExti() 当次采样一致。 */
    [[nodiscard]] bool wasPressedAtLastEdge() const noexcept { return m_isrPressed; }

private:
    gpio_type      *m_port = nullptr;
    std::uint16_t   m_pin  = 0;
    BUTTON_Config  m_config{};
    e_BUTTON_State m_state = e_BUTTON_State::IDLE;

    /*---- 中断侧 (只在 onExti() 里写, 主循环只读; 全 volatile) ----*/
    volatile bool          m_pressPending   = false;  ///< 按下沿待处理 (唤醒状态机)
    volatile bool          m_releasePending = false;  ///< 释放沿待处理 (起释放消抖窗)
    volatile std::uint32_t m_pressTick      = 0;      ///< 按下沿时刻
    volatile std::uint32_t m_isrTick        = 0;      ///< 最近一次边沿时刻
    volatile bool          m_isrPressed     = false;  ///< 最近一次边沿后的电平

    /*---- 主循环侧 ----*/
    std::uint32_t m_debounceTick       = 0;
    std::uint32_t m_pressTime          = 0;
    std::uint32_t m_releaseTick        = 0;    ///< 释放消抖窗起点记下的释放时刻
    std::uint32_t m_pressDuration      = 0;    ///< 最近一次按压时长
    bool          m_releaseWait        = false;///< 释放消抖窗开中
    bool          m_waitingSecondClick = false;
    bool          m_longPressFired     = false;
    bool          m_longPressActive    = false;

    std::uint16_t m_chargePercent = 0;
    std::uint32_t m_chargeTick    = 0;
    std::uint32_t m_decayTick     = 0;

    BUTTON_Counter m_counter{};

    volatile e_BUTTON_Event m_lastEvent = e_BUTTON_Event::NONE;
    volatile bool           m_hasEvent  = false;
    volatile std::uint8_t   m_flags     = 0;

    void pushEvent(e_BUTTON_Event e) noexcept;
    /** 原子取走 onExti() 置的待处理沿 + 边沿时刻 (临界区内读清)。
     *  状态机每一步开头调一次, 之后整步都用这份快照。 */
    BUTTON_IsrEdges takeIsrEdges() noexcept;
    void updateCharge(std::uint32_t now) noexcept;
    /** 释放消抖窗结束后提交释放 (RELEASE 事件 + 状态迁移 + 按压时长)。 */
    e_BUTTON_Event commitRelease(std::uint32_t now) noexcept;
};

} // namespace LoveFinderLib

#endif /* LOVE_FINDER_LIB_BUTTON_HPP */
