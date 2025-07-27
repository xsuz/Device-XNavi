#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sd_logger
{
    /// @brief バイト列をログとして保存する
    /// @param buffer 
    /// @param size 
    /// @param utc
    void write_pkt(const uint8_t *buffer, size_t size,int64_t utc);

    /// @brief SDカードにログを保存するタスク
    /// @param pvParam 
    void task(void *pvParam);
}