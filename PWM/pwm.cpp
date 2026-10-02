/**
 * @file    pwm.cpp
 * @brief   AT32 定时器 PWM 占空比调整实现
 *
 * 设计要点见 pwm.hpp: 只写 CCR, 周期每次运行时读 ARR, 库不持有 ATWK 配置。
 */
#include "pwm.hpp"

namespace LoveFinderLib {

uint32_t pwmGetPeriod(tmr_type *tmr) noexcept
{
    if (tmr == nullptr) return 0u;
    return tmr_period_value_get(tmr);
}

void pwmSetDuty(tmr_type *tmr, tmr_channel_select_type ch, uint32_t value) noexcept
{
    if (tmr == nullptr) return;
    const uint32_t period = tmr_period_value_get(tmr);
    if (value > period) value = period;
    tmr_channel_value_set(tmr, ch, value);
}

void pwmSetPercent(tmr_type *tmr, tmr_channel_select_type ch, uint8_t percent) noexcept
{
    if (tmr == nullptr) return;
    if (percent > 100u) percent = 100u;
    /* period 最大 0xFFFF (TMR 是 16 位计数器) * 100 = 6553500, 远不到 32 位上限 */
    const uint32_t period = tmr_period_value_get(tmr);
    tmr_channel_value_set(tmr, ch, (period * percent) / 100u);
}

void pwmSetPermille(tmr_type *tmr, tmr_channel_select_type ch, uint16_t permille) noexcept
{
    if (tmr == nullptr) return;
    if (permille > 1000u) permille = 1000u;
    /* period * 1000 同样在 32 位安全范围内 */
    const uint32_t period = tmr_period_value_get(tmr);
    tmr_channel_value_set(tmr, ch, (period * permille) / 1000u);
}

} /* namespace LoveFinderLib */
