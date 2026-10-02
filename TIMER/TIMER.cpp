/**
 * @file    TIMER.cpp
 * @brief   可选的 TIM 毫秒时基实现 — C++17 (LoveFinderLibForAT32)
 *
 * 计时模型:
 *   TIM 溢出中断 → timerTickIsr() → g_timerMs += periodMs (例如 10)
 *   主循环       → timerNow()     ← 只读 g_timerMs
 *
 * 中断里只做「清标志 + 自增」两件事, 业务判定全部留在主循环。
 */
#include "TIMER.hpp"

namespace LoveFinderLib {

/*============================================================================
 * 内部状态
 *============================================================================*/
namespace {

volatile std::uint32_t g_timerMs       = 0;    ///< 计时变量 (中断里自增)
std::uint16_t          g_timerPeriodMs = 0;    ///< 每次自增量 (ms)
tmr_type              *g_timerTmr      = nullptr;
IRQn_Type              g_timerIrqn     = TMR6_GLOBAL_IRQn;
bool                   g_timerRunning  = false;

/**
 * @brief TIM 实例 → CRM 外设时钟 + 溢出中断号。
 *
 * AT32F421 共七个定时器 TMR1 / TMR3 / TMR6 / TMR14 / TMR15 / TMR16 / TMR17
 * (见 startup_at32f421.s 向量表与 at32f421.h 的 IRQn_Type); 中断号必须与实例
 * 匹配, 否则 NVIC 开错中断 → 时基永远不动, 所以在这里统一映射、不放进配置。
 *
 * 可用性: TMR3 常被 WorkBench 的 LED PWM 占用, TMR1 是高级定时器 (两套中断),
 * 其余 (TMR6 / TMR14~17) 都可作纯溢出时基 —— TMR6 无通道无引脚, 首选。
 */
bool mapTmr(tmr_type *tmr, crm_periph_clock_type &clk, IRQn_Type &irqn) noexcept
{
    if (tmr == TMR1)  { clk = CRM_TMR1_PERIPH_CLOCK;  irqn = TMR1_BRK_OVF_TRG_HALL_IRQn; return true; }
    if (tmr == TMR3)  { clk = CRM_TMR3_PERIPH_CLOCK;  irqn = TMR3_GLOBAL_IRQn;            return true; }
    if (tmr == TMR6)  { clk = CRM_TMR6_PERIPH_CLOCK;  irqn = TMR6_GLOBAL_IRQn;            return true; }
    if (tmr == TMR14) { clk = CRM_TMR14_PERIPH_CLOCK; irqn = TMR14_GLOBAL_IRQn;           return true; }
    if (tmr == TMR15) { clk = CRM_TMR15_PERIPH_CLOCK; irqn = TMR15_GLOBAL_IRQn;           return true; }
    if (tmr == TMR16) { clk = CRM_TMR16_PERIPH_CLOCK; irqn = TMR16_GLOBAL_IRQn;           return true; }
    if (tmr == TMR17) { clk = CRM_TMR17_PERIPH_CLOCK; irqn = TMR17_GLOBAL_IRQn;           return true; }
    return false;
}

/**
 * @brief 选一组 (预分频, 周期) 让溢出周期严格等于 periodMs, 且分频无误差。
 *
 * 计数频率候选 {1MHz, 100kHz, 10kHz, 1kHz}, 取第一个同时满足:
 *   - clkHz % f == 0          (整除, 否则实际周期与 periodMs 有偏差)
 *   - clkHz / f <= 65536      (预分频 16-bit)
 *   - periodMs * f / 1000 <= 65536 (周期寄存器 16-bit)
 *
 * @param psc 输出: 预分频寄存器值 (tmr_div)
 * @param pr  输出: 周期寄存器值   (tmr_pr)
 */
bool pickDivisors(std::uint32_t clkHz, std::uint16_t periodMs,
                  std::uint16_t &psc, std::uint16_t &pr) noexcept
{
    static constexpr std::uint32_t kTickHz[] = { 1000000u, 100000u, 10000u, 1000u };

    for (const std::uint32_t f : kTickHz) {
        if ((clkHz % f) != 0u) continue;

        const std::uint32_t div = clkHz / f;
        if (div == 0u || div > 65536u) continue;

        const std::uint32_t counts =
            (static_cast<std::uint64_t>(periodMs) * f) / 1000u;
        if (counts == 0u || counts > 65536u) continue;

        psc = static_cast<std::uint16_t>(div - 1u);
        pr  = static_cast<std::uint16_t>(counts - 1u);
        return true;
    }
    return false;
}

} // namespace

/*============================================================================
 * 启动 / 停止
 *============================================================================*/
bool timerTickStart(const TIMER_Config &cfg) noexcept
{
    crm_periph_clock_type clk  = CRM_TMR6_PERIPH_CLOCK;
    IRQn_Type             irqn = TMR6_GLOBAL_IRQn;

    if (cfg.tmr == nullptr || cfg.clkHz == 0u || cfg.periodMs == 0u) return false;
    if (!mapTmr(cfg.tmr, clk, irqn)) return false;

    std::uint16_t psc = 0;
    std::uint16_t pr  = 0;
    if (!pickDivisors(cfg.clkHz, cfg.periodMs, psc, pr)) return false;

    timerTickStop();                       /* 已经起过别的 TIM, 先停掉 */

    crm_periph_clock_enable(clk, TRUE);
    tmr_counter_enable(cfg.tmr, FALSE);
    tmr_base_init(cfg.tmr, pr, psc);       /* pr = 周期, psc = 预分频 */
    /* CNT 归零: 若这路 TIM 之前已被工程 (WorkBench) 启动跑过一段, 不清的话
     * 第一个溢出周期会偏短 —— 时间轴从 0 起但相位对不上, 后续才精确。 */
    tmr_counter_value_set(cfg.tmr, 0);
    tmr_flag_clear(cfg.tmr, TMR_OVF_FLAG);
    tmr_interrupt_enable(cfg.tmr, TMR_OVF_INT, TRUE);

    g_timerTmr      = cfg.tmr;
    g_timerIrqn     = irqn;
    g_timerPeriodMs = cfg.periodMs;
    g_timerMs       = 0;
    g_timerRunning  = true;

    tmr_counter_enable(cfg.tmr, TRUE);
    nvic_irq_enable(irqn, 0, 0);
    return true;
}

void timerTickStop() noexcept
{
    if (g_timerTmr == nullptr) {
        g_timerRunning = false;
        return;
    }
    tmr_interrupt_enable(g_timerTmr, TMR_OVF_INT, FALSE);
    tmr_counter_enable(g_timerTmr, FALSE);
    nvic_irq_disable(g_timerIrqn);
    g_timerRunning = false;
}

/*============================================================================
 * 中断侧
 *============================================================================*/
void timerTickIsr() noexcept
{
    /* 只做两件事: 清本模块那路 TIM 的溢出标志 + 计时变量自增 */
    if (g_timerTmr != nullptr) {
        tmr_flag_clear(g_timerTmr, TMR_OVF_FLAG);
    }
    g_timerMs += g_timerPeriodMs;
}

void timerTickAdd(std::uint32_t ms) noexcept
{
    g_timerMs += ms;                       /* 工程自己的 TIM: 标志由工程自己清 */
}

/*============================================================================
 * 读数
 *============================================================================*/
std::uint32_t timerNow() noexcept { return g_timerMs; }

std::uint16_t timerTickPeriodMs() noexcept { return g_timerPeriodMs; }

bool timerTickRunning() noexcept { return g_timerRunning; }

} // namespace LoveFinderLib
