#pragma once

#include <timers.h>

namespace imu{
    /// @brief IMUによるKalman Filterの更新タスク
    /// @param pvParam 
    void task(void* pvParam);
    
    /// @brief IMUのサンプリング処理
    /// @param xTimer 
    void timer_callback(TimerHandle_t xTimer);
}