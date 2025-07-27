#pragma once

#include <stdint.h>

namespace sys_clock{

    /// @brief UTC時刻
    /// @return 現在のUTC時刻をミリ秒単位で取得
    int64_t get_timestamp();

    /// @brief ログの時刻補正
    /// @param tick_millis 補正時の時刻
    /// @param year 現在の年（西暦）
    /// @param month 現在の月
    /// @param day 現在の日
    /// @param hour 現在の時
    /// @param minute 現在の分
    /// @param second 現在の秒
    void set_timestamp_offset(uint32_t tick_millis,uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second);

    /// @brief UTC時刻を取得
    /// @param year 年
    /// @param month 月
    /// @param day 日
    /// @param hour 時
    /// @param minutes 分
    /// @param seconds 秒
    bool get_datetime(uint16_t *year,uint8_t *month,uint8_t *day,uint8_t *hour,uint8_t *minutes,uint8_t *seconds);

    /// @brief 現在の時刻が有効かどうか
    /// @details get_timestamp()で取得した時刻が有効かどうかを確認する
    /// @return 
    bool is_valid();
}