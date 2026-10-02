/**
 * @file    i2c_master.cpp
 * @brief   AT32 阻塞式硬件 I2C 主机驱动实现 — C++17 (LoveFinderLibForAT32)
 *
 * AT32 I2C 是流式寄存器接口, 需手动控制 START / 地址 / 数据 / STOP,
 * 每一步轮询标志位。实现逐条移植自原平台层 I2cBusHw (语义不变),
 * 差异仅在于: 外设实例作为参数传入, 超时用工程时基 lf_get_tick()。
 */
#include "i2c_master.hpp"

namespace LoveFinderLib {

/*============================================================================
 * 结果码
 *============================================================================*/
const char *i2cStatusName(I2cStatus s) noexcept
{
    switch (s) {
    case I2cStatus::OK:          return "OK";
    case I2cStatus::TIMEOUT:     return "TIMEOUT";
    case I2cStatus::NACK:        return "NACK";
    case I2cStatus::BUS_ERROR:   return "BUS_ERROR";
    case I2cStatus::PARAM_ERROR: return "PARAM_ERROR";
    }
    return "UNKNOWN";
}

namespace {

/*============================================================================
 * 内部辅助
 *============================================================================*/

/**
 * @brief 7-bit 从机地址 → AT32 驱动 i2c_7bit_address_send() 要求的 8 位地址字节。
 *
 * ⚠ AT32 BSP 的 i2c_7bit_address_send() 是把参数【原样】写进 DT, 只做
 *   `addr & 0xFE` (写) / `addr | 0x01` (读) —— 也就是说它要的是**已左移的
 *   8 位地址** (Artery 官方例程里 `#define EEPROM_ADDRESS 0xA0` 就是 0x50<<1)。
 *   本库对外统一用 7-bit 地址 (0x3C / 0x28), 必须在这里左移一次,
 *   否则总线上会出现 (addr>>1) 这个不存在的从机 → 全部 NACK。
 */
constexpr uint8_t addr8Of(uint8_t addr7) noexcept
{
    return static_cast<uint8_t>(addr7 << 1u);
}

/** 等待某标志为 set/清零, 超时返回 false。 */
bool waitFlag(i2c_type *i2c, uint32_t flag, bool set, uint32_t deadline) noexcept
{
    while (true) {
        const auto st = i2c_flag_get(i2c, flag);
        if (set ? (st != RESET) : (st == RESET)) {
            return true;
        }
        if (lf_get_tick() >= deadline) {
            return false;
        }
    }
}

/** 检查并清除错误标志 (NACK / 总线错误 / 仲裁丢失)。 */
I2cStatus checkError(i2c_type *i2c) noexcept
{
    if (i2c_flag_get(i2c, I2C_ACKFAIL_FLAG) != RESET) {
        i2c_flag_clear(i2c, I2C_ACKFAIL_FLAG);
        return I2cStatus::NACK;
    }
    if (i2c_flag_get(i2c, I2C_BUSERR_FLAG) != RESET) {
        i2c_flag_clear(i2c, I2C_BUSERR_FLAG);
        return I2cStatus::BUS_ERROR;
    }
    if (i2c_flag_get(i2c, I2C_ARLOST_FLAG) != RESET) {
        i2c_flag_clear(i2c, I2C_ARLOST_FLAG);
        return I2cStatus::BUS_ERROR;
    }
    return I2cStatus::OK;
}

void clearErrors(i2c_type *i2c) noexcept
{
    i2c_flag_clear(i2c, I2C_ACKFAIL_FLAG);
    i2c_flag_clear(i2c, I2C_BUSERR_FLAG);
    i2c_flag_clear(i2c, I2C_ARLOST_FLAG);
    i2c_flag_clear(i2c, I2C_STOPF_FLAG);
}

/** 等待总线空闲 (BUSYF 清零)。 */
bool waitBusFree(i2c_type *i2c, uint32_t deadline) noexcept
{
    while (i2c_flag_get(i2c, I2C_BUSYF_FLAG) != RESET) {
        if (lf_get_tick() >= deadline) return false;
    }
    return true;
}

/**
 * @brief 等待地址应答: ADDR7F 置位 = 从机 ACK; ACKFAIL 置位 = 从机 NACK。
 *
 * ⚠ 不能只等 ADDR7F —— 从机不存在 (NACK) 时 ADDR7F 永远不置位, 只能靠超时,
 *   一次探测就要等满整个 timeout (总线扫描会慢到不可用)。
 */
I2cStatus waitAddressAck(i2c_type *i2c, uint32_t deadline) noexcept
{
    while (true) {
        if (i2c_flag_get(i2c, I2C_ADDR7F_FLAG) != RESET) return I2cStatus::OK;
        if (i2c_flag_get(i2c, I2C_ACKFAIL_FLAG) != RESET) {
            i2c_flag_clear(i2c, I2C_ACKFAIL_FLAG);
            return I2cStatus::NACK;
        }
        if (lf_get_tick() >= deadline) return I2cStatus::TIMEOUT;
    }
}

} // namespace

/*============================================================================
 * 裸写 / 裸读
 *============================================================================*/
I2cStatus i2cMasterWrite(i2c_type *i2c, uint8_t addr7, const uint8_t *data,
                         uint16_t len, uint32_t timeoutMs) noexcept
{
    if (i2c == nullptr)               return I2cStatus::PARAM_ERROR;
    if (data == nullptr && len != 0u) return I2cStatus::PARAM_ERROR;

    const uint32_t deadline = lf_get_tick() + timeoutMs;

    clearErrors(i2c);
    if (!waitBusFree(i2c, deadline)) return I2cStatus::TIMEOUT;

    /* START */
    i2c_start_generate(i2c);
    if (!waitFlag(i2c, I2C_STARTF_FLAG, true, deadline)) {
        i2c_stop_generate(i2c);
        return I2cStatus::TIMEOUT;
    }

    /* 7-bit 地址左移成 AT32 要求的 8 位地址字节 */
    i2c_7bit_address_send(i2c, addr8Of(addr7), I2C_DIRECTION_TRANSMIT);
    if (const I2cStatus e = waitAddressAck(i2c, deadline); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }
    i2c_flag_clear(i2c, I2C_ADDR7F_FLAG);

    if (const I2cStatus e = checkError(i2c); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }

    /* 数据: 每字节前等发送缓冲空 (TDBE), 与官方例程一致 */
    for (uint16_t i = 0; i < len; ++i) {
        if (!waitFlag(i2c, I2C_TDBE_FLAG, true, deadline)) {
            i2c_stop_generate(i2c);
            return I2cStatus::TIMEOUT;
        }
        i2c_data_send(i2c, data[i]);
    }

    /* 最后一字节真正移位完成 (TDC) 再发 STOP, 否则可能被截断。
       (零长度写 = 只做地址探测的总线扫描, 没有数据可等, 别白等一个超时) */
    if (len != 0u) {
        (void)waitFlag(i2c, I2C_TDC_FLAG, true, deadline);
    }

    /* STOP */
    i2c_stop_generate(i2c);
    (void)waitFlag(i2c, I2C_STOPF_FLAG, true, lf_get_tick() + 5u);
    i2c_flag_clear(i2c, I2C_STOPF_FLAG);

    return I2cStatus::OK;
}

I2cStatus i2cMasterRead(i2c_type *i2c, uint8_t addr7, uint8_t *data,
                        uint16_t len, uint32_t timeoutMs) noexcept
{
    if (i2c == nullptr)               return I2cStatus::PARAM_ERROR;
    if (data == nullptr && len != 0u) return I2cStatus::PARAM_ERROR;
    if (len == 0u)                    return I2cStatus::OK;

    const uint32_t deadline = lf_get_tick() + timeoutMs;

    clearErrors(i2c);
    if (!waitBusFree(i2c, deadline)) return I2cStatus::TIMEOUT;

    i2c_start_generate(i2c);
    if (!waitFlag(i2c, I2C_STARTF_FLAG, true, deadline)) {
        i2c_stop_generate(i2c);
        return I2cStatus::TIMEOUT;
    }

    i2c_7bit_address_send(i2c, addr8Of(addr7), I2C_DIRECTION_RECEIVE);
    if (const I2cStatus e = waitAddressAck(i2c, deadline); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }
    i2c_flag_clear(i2c, I2C_ADDR7F_FLAG);

    if (const I2cStatus e = checkError(i2c); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }

    /* AT32 主机接收 ACK 控制: acken=1 回 ACK, acken=0 回 NACK。
     * 最后一个字节前必须关 ACK。 */
    for (uint16_t i = 0; i < len; ++i) {
        i2c_ack_enable(i2c, (i == static_cast<uint16_t>(len - 1u)) ? FALSE : TRUE);

        if (!waitFlag(i2c, I2C_RDBF_FLAG, true, deadline)) {
            i2c_ack_enable(i2c, TRUE);
            i2c_stop_generate(i2c);
            return I2cStatus::TIMEOUT;
        }
        data[i] = i2c_data_receive(i2c);
    }

    i2c_ack_enable(i2c, TRUE);   /* 恢复默认 ACK */
    i2c_stop_generate(i2c);
    (void)waitFlag(i2c, I2C_STOPF_FLAG, true, lf_get_tick() + 5u);
    i2c_flag_clear(i2c, I2C_STOPF_FLAG);

    return I2cStatus::OK;
}

/*============================================================================
 * 寄存器访问
 *============================================================================*/
I2cStatus i2cMemWrite(i2c_type *i2c, uint8_t addr7, uint8_t reg,
                      const uint8_t *data, uint16_t len, uint32_t timeoutMs) noexcept
{
    if (i2c == nullptr)               return I2cStatus::PARAM_ERROR;
    if (data == nullptr && len != 0u) return I2cStatus::PARAM_ERROR;
    if (len > 16u)                    return I2cStatus::PARAM_ERROR;

    uint8_t buf[17];
    buf[0] = reg;
    for (uint16_t i = 0; i < len; ++i) {
        buf[i + 1u] = data[i];
    }
    return i2cMasterWrite(i2c, addr7, buf, static_cast<uint16_t>(len + 1u), timeoutMs);
}

/**
 * @brief 寄存器读: START + addr(W) + reg + rSTART + addr(R) + data[] + STOP
 *
 * 用 repeated START (不发中间 STOP), 这是 I2C 规范要求的寄存器读时序,
 * 也是所有快充芯片数据手册给出的时序。
 */
I2cStatus i2cMemRead(i2c_type *i2c, uint8_t addr7, uint8_t reg, uint8_t *data,
                     uint16_t len, uint32_t timeoutMs) noexcept
{
    if (i2c == nullptr)               return I2cStatus::PARAM_ERROR;
    if (data == nullptr && len != 0u) return I2cStatus::PARAM_ERROR;
    if (len == 0u)                    return I2cStatus::OK;

    const uint32_t deadline = lf_get_tick() + timeoutMs;

    clearErrors(i2c);
    if (!waitBusFree(i2c, deadline)) return I2cStatus::TIMEOUT;

    /* ---- Phase 1: 写寄存器指针 (不发 STOP) ---- */
    i2c_start_generate(i2c);
    if (!waitFlag(i2c, I2C_STARTF_FLAG, true, deadline)) return I2cStatus::TIMEOUT;

    i2c_7bit_address_send(i2c, addr8Of(addr7), I2C_DIRECTION_TRANSMIT);
    if (const I2cStatus e = waitAddressAck(i2c, deadline); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }
    i2c_flag_clear(i2c, I2C_ADDR7F_FLAG);

    if (const I2cStatus e = checkError(i2c); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }

    if (!waitFlag(i2c, I2C_TDBE_FLAG, true, deadline)) {
        i2c_stop_generate(i2c);
        return I2cStatus::TIMEOUT;
    }
    i2c_data_send(i2c, reg);
    if (!waitFlag(i2c, I2C_TDC_FLAG, true, deadline)) {
        i2c_stop_generate(i2c);
        return I2cStatus::TIMEOUT;
    }

    /* ---- Phase 2: repeated START + 读 ---- */
    i2c_start_generate(i2c);
    if (!waitFlag(i2c, I2C_STARTF_FLAG, true, deadline)) {
        i2c_stop_generate(i2c);
        return I2cStatus::TIMEOUT;
    }

    i2c_7bit_address_send(i2c, addr8Of(addr7), I2C_DIRECTION_RECEIVE);
    if (const I2cStatus e = waitAddressAck(i2c, deadline); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }
    i2c_flag_clear(i2c, I2C_ADDR7F_FLAG);

    if (const I2cStatus e = checkError(i2c); e != I2cStatus::OK) {
        i2c_stop_generate(i2c);
        return e;
    }

    for (uint16_t i = 0; i < len; ++i) {
        i2c_ack_enable(i2c, (i == static_cast<uint16_t>(len - 1u)) ? FALSE : TRUE);

        if (!waitFlag(i2c, I2C_RDBF_FLAG, true, deadline)) {
            i2c_ack_enable(i2c, TRUE);
            i2c_stop_generate(i2c);
            return I2cStatus::TIMEOUT;
        }
        data[i] = i2c_data_receive(i2c);
    }

    i2c_ack_enable(i2c, TRUE);
    i2c_stop_generate(i2c);
    (void)waitFlag(i2c, I2C_STOPF_FLAG, true, lf_get_tick() + 5u);
    i2c_flag_clear(i2c, I2C_STOPF_FLAG);

    return I2cStatus::OK;
}

/*============================================================================
 * 便捷封装
 *============================================================================*/
bool i2cWriteReg8(i2c_type *i2c, uint8_t addr7, uint8_t reg,
                   uint8_t value, uint32_t timeoutMs) noexcept
{
    return i2cMemWrite(i2c, addr7, reg, &value, 1u, timeoutMs) == I2cStatus::OK;
}

std::optional<uint8_t> i2cReadReg8(i2c_type *i2c, uint8_t addr7, uint8_t reg,
                                   uint32_t timeoutMs) noexcept
{
    uint8_t v = 0;
    if (i2cMemRead(i2c, addr7, reg, &v, 1u, timeoutMs) != I2cStatus::OK) {
        return std::nullopt;
    }
    return v;
}

/*============================================================================
 * 总线恢复
 *============================================================================*/
/**
 * @brief 总线恢复: 从机把 SDA 拉死时, 复位 I2C 外设。
 *
 * 注意: 完整的恢复需要手动敲 SCL 脉冲 (依赖具体引脚), 由应用层自行实现;
 * 这里只做外设软复位 —— 与原平台层行为一致。
 */
void i2cMasterRecover(i2c_type *i2c) noexcept
{
    if (i2c == nullptr) return;

    i2c_enable(i2c, FALSE);
    i2c_software_reset(i2c, TRUE);
    i2c_software_reset(i2c, FALSE);
    i2c_ack_enable(i2c, TRUE);
    i2c_enable(i2c, TRUE);
}

} // namespace LoveFinderLib
