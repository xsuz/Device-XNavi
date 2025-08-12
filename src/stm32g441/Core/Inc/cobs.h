#pragma once

#include<stdint.h>
#include<stddef.h>

namespace cobs
{
    size_t encode(const uint8_t *input, size_t input_len, uint8_t *output);
    size_t decode(const uint8_t *input,const size_t size, size_t *start_idx,size_t end_idx,uint8_t *output);
} // namespace cobs
