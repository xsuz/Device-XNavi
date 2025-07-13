#pragma once
#include <Arduino.h>
#include <timers.h>

namespace gnss{
    /// @brief PPS信号の割り込みハンドラ
    /// @param gpio
    /// @param emask
    void pps_callback(uint gpio,uint32_t emask);
    
    /// @brief GPSレシーバの受信タスク
    /// @param pvParam
    void task(void* pvParam);

    /// @brief 
    /// @param xTimer GPSのサンプリング処理
    void timer_callback(TimerHandle_t xTimer);
}