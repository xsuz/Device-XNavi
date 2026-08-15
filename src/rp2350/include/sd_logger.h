#pragma once

#include <stdint.h>
#include <stddef.h>
#include <mavlink/mavlink_types.h>

namespace sd_logger
{
    /// @brief パケットをログとして保存する
    /// @param msg mavlinkのメッセージ
    /// @param utc timestamp
    void write_pkt(const mavlink_message_t *msg, int64_t utc);

    /// @brief バイト列をログとして保存する
    /// @param buffer 
    /// @param size 
    /// @param utc
    void write_bytes(const uint8_t *buffer, size_t size,int64_t utc);

    /// @brief SDカードにログを保存するタスク
    /// @param pvParam 
    void task(void *pvParam);

    /// @brief SDカードのステータスを取得する
    /// @return 0 = 無効, 1 = 有効
    uint8_t is_valid();

    size_t available_ring_buffer_size();
}