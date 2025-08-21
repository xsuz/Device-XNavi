#pragma once
#include <Arduino.h>
#include <timers.h>
#include <queue.h>

namespace gnss{
    
    /// @brief PPS信号の割り込みハンドラ
    /// @param gpio
    /// @param emask
    void pps_callback(uint gpio,uint32_t emask);
    
    /// @brief GPSレシーバのタスク
    /// @param pvParam
    void task(void* pvParam);

    /// @brief GPSレシーバのポーリングのタスク
    /// @param pvParam
    void task_polling(void* pvParam);
}