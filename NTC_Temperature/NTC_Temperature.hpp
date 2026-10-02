/**
 * @file    NTC_Temperature.hpp
 * @brief   NTC 热敏电阻测温 — C++17 (LoveFinderLibForAT32)
 *
 * ============================================================================
 *  外部电路与配置
 * ============================================================================
 *  支持两种拓扑，由 NTC_POSITION 选择:
 *
 *   NTC_LOW_SIDE  (NTC 在低侧, 本项目):
 *       3V3 ──[R_series]──┬── ADC
 *                         └──[NTC]── GND
 *       Vadc = Vref * Rntc / (R_series + Rntc)
 *       → Rntc = R_series * Vadc / (Vref - Vadc)
 *
 *   NTC_HIGH_SIDE (NTC 在高侧):
 *       3V3 ──[NTC]──┬── ADC
 *                    └──[R_series]── GND
 *       Vadc = Vref * R_series / (R_series + Rntc)
 *       → Rntc = R_series * (Vref - Vadc) / Vadc
 *
 * ============================================================================
 *  lf_config.h 中需要配置的宏
 * ============================================================================
 *   NTC_POSITION          NTC_LOW_SIDE | NTC_HIGH_SIDE
 *   NTC_RESISTANCE        NTC 在 25℃ 的标称阻值 (Ω), 如 100000.0f
 *   NTC_SERIES_RESISTOR   串联的贴片电阻阻值 (Ω), 如 100000.0f
 *   NTC_B_VALUE           NTC 的 B 值 (K), 如 3950.0f
 *   NTC_R0_TEMPERATURE_C  R0 对应的温度 (℃), 通常 25.0f
 *   NTC_VREF_MV           上拉电源电压 (mV), 如 3300.0f
 *   NTC_ADC_RESOLUTION    ADC 满量程 (12-bit → 4095.0f)
 *   NTC_DMA_BUFFER_SIZE   DMA 环形缓冲深度 (滤波窗口), 如 16
 *   NTC_INVALID_TEMP_C    无效值哨兵, 如 -1000.0f
 *   NTC_ADC_INSTANCE      ADC 实例, 默认 ADC1 (直绑 AT32 BSP)
 *   NTC_DMA_CHANNEL       DMA 通道, 默认 DMA1_CHANNEL1
 *
 * ============================================================================
 *  用法
 * ============================================================================
 *    LoveFinderLib::NTC_Temperature ntc;
 *    ntc.init();        // 绑定 NTC_ADC_INSTANCE / NTC_DMA_CHANNEL (缺省值)
 *    ntc.start();
 *    ...
 *    float t = ntc.getTemp();                  // 直接取温度 (℃)
 */
#ifndef LOVE_FINDER_LIB_NTC_TEMPERATURE_HPP
#define LOVE_FINDER_LIB_NTC_TEMPERATURE_HPP

#include "lf_config.h"      /* ★ 工程侧: ADC/DMA 实例宏 (仿 PY32 的 gpio.hpp) */

#include <cstdint>
#include <cmath>

/*============================================================================
 * 位置枚举 —— 必须在 NTC_POSITION 的默认值之前定义,
 * 否则 #if (NTC_POSITION == NTC_LOW_SIDE) 会退化为 0 == 0 恒真。
 *============================================================================*/
#define NTC_LOW_SIDE   0   /* NTC 接 GND 侧 (上拉到 Vref) */
#define NTC_HIGH_SIDE  1   /* NTC 接 Vref 侧 (下拉到 GND) */

/*============================================================================
 * 缺省配置 (项目 lf_config.h 中的定义会覆盖)
 *============================================================================*/
#ifndef NTC_POSITION
#define NTC_POSITION        NTC_LOW_SIDE
#endif

#ifndef NTC_RESISTANCE
#define NTC_RESISTANCE      100000.0f
#endif

#ifndef NTC_SERIES_RESISTOR
#define NTC_SERIES_RESISTOR 100000.0f
#endif

#ifndef NTC_B_VALUE
#define NTC_B_VALUE         3950.0f
#endif

#ifndef NTC_R0_TEMPERATURE_C
#define NTC_R0_TEMPERATURE_C 25.0f
#endif

#ifndef NTC_VREF_MV
#define NTC_VREF_MV         3300.0f
#endif

#ifndef NTC_ADC_RESOLUTION
#define NTC_ADC_RESOLUTION  4095.0f
#endif

#ifndef NTC_DMA_BUFFER_SIZE
#define NTC_DMA_BUFFER_SIZE 16
#endif

#ifndef NTC_INVALID_TEMP_C
#define NTC_INVALID_TEMP_C  (-1000.0f)
#endif

/** 合理温度区间 (℃) —— 超出即判为无效 (断线 / 未贴 NTC / 短路)。
 *  默认 -40 ~ 150 ℃, 可在 lf_config.h 覆盖。 */
#ifndef NTC_TEMP_MIN_C
#define NTC_TEMP_MIN_C      (-40.0f)
#endif

#ifndef NTC_TEMP_MAX_C
#define NTC_TEMP_MAX_C      (150.0f)
#endif

/*---------------------------------------------------------------------------
 * ADC / DMA 直绑实例 (项目 lf_config.h 可覆盖; 仿 PY32: 驱动直调 LL_* 实例)
 *-------------------------------------------------------------------------*/
#ifndef NTC_ADC_INSTANCE
#define NTC_ADC_INSTANCE    ADC1
#endif

#ifndef NTC_DMA_CHANNEL
#define NTC_DMA_CHANNEL     DMA1_CHANNEL1
#endif

namespace LoveFinderLib {

/*============================================================================
 * 滤波模式
 *============================================================================*/
enum class e_NTC_Filter : std::uint8_t {
    NONE   = 0,   ///< 直接取最新一次采样
    MEAN   = 1,   ///< 算术平均
    MEDIAN = 2    ///< 中值滤波 (推荐, 抗脉冲干扰)
};

/*============================================================================
 * NTC_Temperature
 *============================================================================*/
class NTC_Temperature {
public:
    NTC_Temperature() = default;

    /**
     * @brief 绑定 ADC 与 DMA 通道 (缺省取 NTC_ADC_INSTANCE / NTC_DMA_CHANNEL)。
     *
     * 直绑 AT32 外设实例, 无平台抽象层 —— 与 PY32 库直调 LL_ADC_* 同一模式。
     * 采样缓冲为驱动内静态 m_buf[], 由 DMA 后台填充。
     */
    explicit NTC_Temperature(adc_type *adc, dma_channel_type *dmaCh) noexcept
    {
        init(adc, dmaCh);
    }

    void init(adc_type *adc = NTC_ADC_INSTANCE,
              dma_channel_type *dmaCh = NTC_DMA_CHANNEL) noexcept
    {
        m_adc = adc;
        m_dma = dmaCh;
    }

    /** 设置滤波模式 (默认中值)。 */
    void setFilter(e_NTC_Filter mode) noexcept { m_filter = mode; }

    /** 启动连续采样: 装载 DMA 缓冲 → 开通道 → 软触发 ADC。 */
    void start() noexcept
    {
        if (m_adc == nullptr || m_dma == nullptr) return;

        dma_channel_enable(m_dma, FALSE);
        m_dma->dtcnt = NTC_DMA_BUFFER_SIZE;
        m_dma->maddr = reinterpret_cast<std::uint32_t>(m_buf);
        dma_channel_enable(m_dma, TRUE);

        adc_ordinary_software_trigger_enable(m_adc, TRUE);
    }

    void stop() noexcept
    {
        if (m_adc == nullptr || m_dma == nullptr) return;

        dma_channel_enable(m_dma, FALSE);
        adc_ordinary_software_trigger_enable(m_adc, FALSE);
    }

    /**
     * @brief 从 DMA 缓冲取一个滤波后的原始 ADC 值。
     *
     * 应在主循环中定期调用 (如每 10ms), 或直接依赖 getTemp() 内部调用。
     * @return 滤波后的 ADC 值; 无数据返回 0。
     */
    std::uint16_t readRawADC()
    {
        if (m_dma == nullptr) return 0u;

        const volatile std::uint16_t *buf = m_buf;
        const std::uint16_t size = NTC_DMA_BUFFER_SIZE;

        if (m_filter == e_NTC_Filter::NONE) {
            m_lastRaw = buf[size - 1u];
            return m_lastRaw;
        }

        /* 拷贝快照 —— DMA 在后台持续写入, 避免读到撕裂值 */
        constexpr std::uint16_t kMaxWindow = 32u;
        std::uint16_t window[kMaxWindow];
        const std::uint16_t n = (size < kMaxWindow) ? size : kMaxWindow;
        for (std::uint16_t i = 0; i < n; ++i) {
            window[i] = buf[i];
        }

        if (m_filter == e_NTC_Filter::MEAN) {
            std::uint32_t sum = 0u;
            for (std::uint16_t i = 0; i < n; ++i) {
                sum += window[i];
            }
            m_lastRaw = static_cast<std::uint16_t>(sum / n);
            return m_lastRaw;
        }

        /* 中值: 插入排序后取中位 (n <= 32, 开销可忽略) */
        for (std::uint16_t i = 1; i < n; ++i) {
            const std::uint16_t key = window[i];
            std::int16_t j = static_cast<std::int16_t>(i) - 1;
            while (j >= 0 && window[j] > key) {
                window[j + 1] = window[j];
                --j;
            }
            window[j + 1] = key;
        }
        m_lastRaw = window[n / 2u];
        return m_lastRaw;
    }

    /**
     * @brief 获取温度 (℃)。内部自动取一次滤波后的 ADC。
     * @return 温度; 无有效数据返回 NTC_INVALID_TEMP_C。
     */
    float getTemp()
    {
        const std::uint16_t raw = readRawADC();
        if (raw == 0u) return NTC_INVALID_TEMP_C;
        return adcToCelsius(raw);
    }

    /** 取上一次的原始 ADC 值。 */
    [[nodiscard]] std::uint16_t getRawADC() const noexcept { return m_lastRaw; }

    /**
     * @brief 静态换算: ADC 原始值 → 温度 (℃)。
     * 不依赖实例, 可用于自测或其它 ADC 通道。
     */
    static float adcToCelsius(std::uint16_t adcValue) noexcept
    {
        if (adcValue == 0u) return NTC_INVALID_TEMP_C;

        const float vref = NTC_VREF_MV;
        const float vAdc = static_cast<float>(adcValue) * vref / NTC_ADC_RESOLUTION;

        /* 满量程或零电压 → 无意义 (断线 / 短路) */
        if (vAdc <= 0.0f || vAdc >= vref) return NTC_INVALID_TEMP_C;

        const float rSeries = NTC_SERIES_RESISTOR;

#if (NTC_POSITION == NTC_LOW_SIDE)
        /* NTC 在低侧: Vadc 取在 NTC 两端 */
        const float rNtc = rSeries * vAdc / (vref - vAdc);
#else
        /* NTC 在高侧: Vadc 取在串联电阻两端 */
        const float rNtc = rSeries * (vref - vAdc) / vAdc;
#endif

        if (rNtc <= 0.0f) return NTC_INVALID_TEMP_C;

        const float t = resistanceToCelsius(rNtc);

        /* 落在合理区间之外 → 无效 (NTC 未贴 / 断线 / 短路)。
         * 例: 只焊了上拉电阻、NTC 空焊时 ADC 贴近满量程, 会算出 -90℃ 这种假值。 */
        if (t < NTC_TEMP_MIN_C || t > NTC_TEMP_MAX_C) return NTC_INVALID_TEMP_C;

        return t;
    }

    /**
     * @brief 静态换算: NTC 阻值 (Ω) → 温度 (℃), 使用 B 值公式。
     *
     *  1/T = 1/T0 + (1/B) * ln(R/R0)
     */
    static float resistanceToCelsius(float rNtc) noexcept
    {
        if (rNtc <= 0.0f) return NTC_INVALID_TEMP_C;

        constexpr float kKelvin = 273.15f;
        const float t0 = NTC_R0_TEMPERATURE_C + kKelvin;   /* R0 对应的开尔文温度 */
        const float r0 = NTC_RESISTANCE;
        const float b  = NTC_B_VALUE;

        if (b <= 0.0f || r0 <= 0.0f) return NTC_INVALID_TEMP_C;

        const float invT = (1.0f / t0) + (1.0f / b) * std::log(rNtc / r0);
        if (invT <= 0.0f) return NTC_INVALID_TEMP_C;

        return (1.0f / invT) - kKelvin;
    }

    /** 反向换算: 温度 (℃) → NTC 阻值 (Ω)。用于阈值比较 / 自测。 */
    static float celsiusToResistance(float celsius) noexcept
    {
        constexpr float kKelvin = 273.15f;
        const float t0 = NTC_R0_TEMPERATURE_C + kKelvin;
        const float t  = celsius + kKelvin;
        if (t <= 0.0f) return 0.0f;

        const float b = NTC_B_VALUE;
        return NTC_RESISTANCE * std::exp(b * ((1.0f / t) - (1.0f / t0)));
    }

private:
    adc_type           *m_adc = nullptr;
    dma_channel_type   *m_dma = nullptr;
    /** DMA 后台填充的采样缓冲 (驱动自持, 替代原平台层的通道对象)。 */
    alignas(2) volatile std::uint16_t m_buf[NTC_DMA_BUFFER_SIZE]{};
    e_NTC_Filter    m_filter  = e_NTC_Filter::MEDIAN;
    std::uint16_t   m_lastRaw = 0u;
};

} // namespace LoveFinderLib

#endif /* LOVE_FINDER_LIB_NTC_TEMPERATURE_HPP */
