#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sd_logger
{
    /// @brief バイト列をログとして保存する
    /// @param buffer 
    /// @param size 
    void write_pkt(const uint8_t *buffer, size_t size);

    /// @brief SDカードにログを保存するタスク
    /// @param pvParam 
    void task(void *pvParam);

    /// @brief ログの時刻補正
    /// @param val UTC時刻
    void set_timestamp_offset(int64_t val);

    /// @brief ログの時刻補正
    /// @param year 現在の年（西暦）
    /// @param month 現在の月
    /// @param day 現在の日
    /// @param hour 現在の時
    /// @param minute 現在の分
    /// @param second 現在の秒
    void set_timestamp_offset(uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second);
}