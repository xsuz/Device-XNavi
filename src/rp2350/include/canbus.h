#pragma once

#include <stdint.h>
#include <stddef.h>
#include <mavlink/mavlink_types.h>

namespace canbus {
    /// @brief TWELITE受信タスク
    /// @param pvParam 
    void task(void* pvParam);
    /// @brief CANパケットの送信
    /// @param pkt 
    void write_pkt(const mavlink_message_t& pkt);
};