/**
 * @file    pwm.hpp
 * @brief   AT32 定时器 PWM 占空比调整 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  设计: 库只改【比较值】, 初始化全部交给 ATWK
 * ============================================================================
 *  - 前置条件: 工程已在 ATWK 上配好 TMR (周期 ARR、分频 PSC、通道 PWM 模式、
 *    引脚复用), 并在 main 里调用过 wk_tmr3_init() 之类的初始化。
 *  - 本模块【不调用】 tmr_output_channel_config / tmr_counter_value_set /
 *    gpio_init, 也【不在运行中修改】周期与分频 —— 只写 CCR 比较寄存器。
 *  - 周期不是库持有的配置: pwmSetPercent / pwmSetPermille 每次运行时用
 *    tmr_period_value_get() 读当前 ARR, 所以 ATWK 改了周期, 按百分比的调用
 *    会自动跟着走, 不会出现"库里的常量和实际周期对不上"。
 *  - 没配 PWM 的通道也能调 (写个寄存器而已), 只是没有波形输出。
 *
 * 用法:
 *    using namespace LoveFinderLib;
 *
 *    pwmSetPercent(TMR3, TMR_SELECT_CHANNEL_2, 30);   // 30%
 *    pwmSetPermille(TMR3, TMR_SELECT_CHANNEL_2, 750); // 呼吸灯细粒度
 *    pwmSetDuty(TMR3, TMR_SELECT_CHANNEL_2, 500);     // 直接给 ticks
 *
 *    uint32_t arr = pwmGetPeriod(TMR3);               // 当前周期 (ticks)
 */
#ifndef LOVEFINDER_PWM_HPP
#define LOVEFINDER_PWM_HPP

#include <cstdint>

/* ★ 自带 BSP 头 (库 AGENTS.md 已知约束一节的要求): conf.h 里的模块宏决定
 *    哪些头被条件包含, 本模块自己把用到的头列全, 不指望宿主工程的 conf
 *    顺带带出来 —— 否则宿主没勾 TMR 就编不过。 */
#include "at32f421_tmr.h"   /* tmr_type / tmr_channel_select_type / tmr_channel_value_set */

namespace LoveFinderLib {

/** 当前周期 (ARR, 单位 ticks)。ATWK 没配过则返回寄存器复位值。 */
uint32_t pwmGetPeriod(tmr_type *tmr) noexcept;

/**
 * @brief 直接设比较值 (ticks)。
 *
 * value 会被钳到 [0, period]: 越界值按满占空比处理, 不会写出比周期还大的
 * 比较值 (PWM_MODE_A 下那会得到与 100% 相同的波形, 但保留钳位以免读起来像
 * 疑似 bug 的数值)。
 */
void pwmSetDuty(tmr_type *tmr, tmr_channel_select_type ch, uint32_t value) noexcept;

/** 按百分比设占空比 (0..100, 越界钳到两端)。周期运行时从 ARR 读。 */
void pwmSetPercent(tmr_type *tmr, tmr_channel_select_type ch, uint8_t percent) noexcept;

/** 按千分比设占空比 (0..1000) —— 呼吸灯这类需要细粒度的场景用这个。 */
void pwmSetPermille(tmr_type *tmr, tmr_channel_select_type ch, uint16_t permille) noexcept;

} /* namespace LoveFinderLib */

#endif /* LOVEFINDER_PWM_HPP */
