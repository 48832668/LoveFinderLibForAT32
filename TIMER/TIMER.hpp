/**
 * @file    TIMER.hpp
 * @brief   可选的 TIM 毫秒时基 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  这是什么
 * ============================================================================
 *  一个由【一路 TIM 周期中断】驱动的毫秒计数器。例如每 10ms 进一次中断,
 *  中断里让计时变量 +10 (timerTickIsr)。用途:
 *
 *    - 让某个模块 (典型是 BUTTON 按键) 脱离 SysTick / lf_get_tick(),
 *      自成一套时间轴;
 *    - 或复用工程已经开好的一路 TIM, 把它的中断当时间轴。
 *
 *  ★ 本模块是【可选】的, 且默认不启用:
 *    - 不调 timerTickStart() / timerTickAdd() 就不会被引用,
 *      --gc-sections 下不占 Flash;
 *    - 工程本就有 lf_get_tick() (SysTick 1ms 中断), 分辨率比 10ms 的 TIM 更细,
 *      其余驱动 (I2C 超时 / ST7735 DMA 超时 / 按键消抖) 也全部用它。
 *      只有在「不想依赖 SysTick」或「要把时间轴交给自己的一路 TIM」时才启用。
 *
 *  中断模型 (与 BUTTON 配套的两段式):
 *    - 中断里只做: 清中断标志 + 计时变量自增;
 *    - 主循环里做全部业务判定, 读 timerNow() 取时间。
 *
 * ============================================================================
 *  用法一 —— 用库初始化一路 TIM (工程还没开这个 TIM 时)
 * ============================================================================
 *  lf_config.h:
 *      #define BUTTON_TIMEBASE_TIMER   1
 *      #define BUTTON_TIM_INSTANCE     TMR6       // 空闲 TIM (TMR3 已被 LED PWM 占用)
 *      #define BUTTON_TIM_CLK_HZ       120000000  // 该 TIM 的计数时钟 (APB 分频 1 → sysclk)
 *      #define BUTTON_TIM_PERIOD_MS    10
 *
 *  启动时:
 *      LoveFinderLib::TIMER_Config tc;
 *      tc.tmr      = BUTTON_TIM_INSTANCE;
 *      tc.clkHz     = BUTTON_TIM_CLK_HZ;
 *      tc.periodMs = BUTTON_TIM_PERIOD_MS;
 *      LoveFinderLib::timerTickStart(tc);   // 返回 false = 分频不成立, 此时别启用
 *
 *  中断 (工程侧定义强符号, 覆盖启动文件里的弱定义):
 *      extern "C" void TMR6_GLOBAL_IRQHandler(void) { LoveFinderLib::timerTickIsr(); }
 *
 * ============================================================================
 *  用法二 —— 复用工程自己已有的一路 TIM
 * ============================================================================
 *  自己 init TIM、自己在中断里清标志, 中断里加一句即可:
 *      LoveFinderLib::timerTickAdd(10);      // 每次中断自增多少 ms
 *  不要与 timerTickStart 混用, 也不要调 timerTickIsr() ——
 *  它清的是【库自己 init 的那路 TIM】的溢出标志 (标志清错会中断风暴)。
 *
 * ============================================================================
 *  分频规则 (保证时基精确、无累积误差)
 * ============================================================================
 *  计数频率按 {1MHz, 100kHz, 10kHz, 1kHz} 逐档尝试, 取第一个同时满足
 *  「能整除 clkHz」「预分频 16-bit 内」「periodMs 内的计数次数 16-bit 内」
 *  的档位 → 实际中断周期与 periodMs 严格相等。
 *  clkHz 不是 1MHz 整数倍时自动落到下一档; 四档都不成立则 timerTickStart
 *  返回 false (此时不要启用, 改回 SysTick 时基)。
 *
 *  各 TIM 对应的中断号 (强符号函数名, AT32F421 共七个定时器):
 *      TMR1  → TMR1_BRK_OVF_TRG_HALL_IRQHandler  (高级定时器, 一般不用)
 *      TMR3  → TMR3_GLOBAL_IRQHandler            (常被 WorkBench 用作 PWM)
 *      TMR6  → TMR6_GLOBAL_IRQHandler            (基本定时器, 首选)
 *      TMR14 → TMR14_GLOBAL_IRQHandler           (备选)
 *      TMR15 → TMR15_GLOBAL_IRQHandler           (备选, 被 ADC 触发源引用)
 *      TMR16 → TMR16_GLOBAL_IRQHandler           (备选, 红外/DMAMUX remap)
 *      TMR17 → TMR17_GLOBAL_IRQHandler           (备选, DMAMUX remap)
 */
#ifndef LOVE_FINDER_LIB_TIMER_HPP
#define LOVE_FINDER_LIB_TIMER_HPP

#include "lf_config.h"      /* ★ 工程侧: 工程配置 + lf_get_tick() 声明 */

/*★ 不能只靠 lf_config.h → at32f421_conf.h: conf.h 按 TMR_MODULE_ENABLED /
 *  CRM_MODULE_ENABLED 等宏【条件包含】外设头, 不用定时器的工程 (如 907) 根本
 *  没有 tmr_type —— 所以本模块自己把要用的三个 BSP 头列全 (均自带 extern "C" 保护)。 */
#include "at32f421_tmr.h"    /* tmr_type / tmr_base_init / TMR_OVF_FLAG / TMR_OVF_INT */
#include "at32f421_crm.h"    /* crm_periph_clock_type / crm_periph_clock_enable */
#include "at32f421_misc.h"   /* nvic_irq_enable / nvic_irq_disable */

#include <cstdint>

namespace LoveFinderLib {

/*============================================================================
 * 配置
 *============================================================================*/
struct TIMER_Config {
    tmr_type     *tmr      = nullptr;  ///< TIM 实例: TMR1 / TMR3 / TMR6 / TMR14~17
    std::uint32_t clkHz    = 0;        ///< 该 TIM 的计数时钟 (AHB 经 APB 分频后, 常 = sysclk)
    std::uint16_t periodMs = 10;       ///< 中断周期 (ms)
};

/*============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief 用一路 TIM 起毫秒时基 (开外设时钟 + 分频 + 溢出中断 + NVIC)。
 * @param cfg TIM 实例 / 计数时钟 / 周期
 * @return false = 参数为空、该 TIM 不在本芯片上、或分频凑不出精确周期
 *         (此时保持现状, 继续用 lf_get_tick() 即可)。
 *
 * ★ 只能占用【空闲】的 TIM —— 会对该 TIM 重新 base_init。
 */
bool timerTickStart(const TIMER_Config &cfg) noexcept;

/** 停掉 timerTickStart 起的那路 TIM (关中断 / 关计数 / 关 NVIC)。 */
void timerTickStop() noexcept;

/**
 * @brief 在【库 init 的那路 TIM】的中断里调用: 清溢出标志 + 计时变量 +=周期。
 *
 * 必须先 timerTickStart() 成功 (否则 g_timerTmr 为空, 只自增不清标志)。
 * 自己 init 的 TIM 用 timerTickAdd(), 不要用本函数。
 */
void timerTickIsr() noexcept;

/**
 * @brief 在【工程自己 init 的 TIM】中断里调用: 计时变量 += ms。
 *
 * 中断标志由工程自己清。与 timerTickIsr() 二选一, 不要混用。
 */
void timerTickAdd(std::uint32_t ms) noexcept;

/** 当前毫秒。timerTickStart 之前恒为 0; 32-bit 读在 CM4 上原子。 */
[[nodiscard]] std::uint32_t timerNow() noexcept;

/** 已配置的中断周期 (ms), 未起过为 0。 */
[[nodiscard]] std::uint16_t timerTickPeriodMs() noexcept;

/** timerTickStart 是否成功起着。 */
[[nodiscard]] bool timerTickRunning() noexcept;

} // namespace LoveFinderLib

#endif /* LOVE_FINDER_LIB_TIMER_HPP */
