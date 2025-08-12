#pragma once

#include <stdint.h>
#include <stddef.h>

#include "DeviceData.h"

namespace canbus {
    /// @brief TWELITE受信タスク
    /// @param pvParam 
    void task(void* pvParam);
    /// @brief CANパケットの送信
    /// @param pkt 
    void write_pkt(DeviceData::CANPacket pkt);
};