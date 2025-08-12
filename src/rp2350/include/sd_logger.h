#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sd_logger
{
    /// @brief パケットをログとして保存する
    /// @param buffer 
    /// @param size 
    /// @param utc
    void write_pkt(uint32_t id, const uint8_t *payload, size_t size,int64_t utc);

    /// @brief バイト列をログとして保存する
    /// @param buffer 
    /// @param size 
    /// @param utc
    void write_bytes(const uint8_t *buffer, size_t size,int64_t utc);

    /// @brief SDカードにログを保存するタスク
    /// @param pvParam 
    void task(void *pvParam);
}