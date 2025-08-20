#include "cobs.h"

size_t cobs::encode(const uint8_t *input, size_t input_len, uint8_t *output)
{
    size_t enc_idx = 0;
    uint8_t cobs_buf_idx = 0;
    uint8_t cobs_buf[256];

    for (size_t i = 0; i < input_len; i++)
    {
        if (input[i] == 0)
        {
            output[enc_idx++] = cobs_buf_idx + 1; // Write the length of the segment
            for (uint8_t j = 0; j < cobs_buf_idx; j++)
            {
                output[enc_idx++] = cobs_buf[j]; // Write the segment data
            }
            cobs_buf_idx = 0; // Reset for next segment
        }
        else
        {
            cobs_buf[cobs_buf_idx++] = input[i];
        }
    }
    output[enc_idx++] = cobs_buf_idx + 1; // Write the length of the last segment
    for (uint8_t j = 0; j < cobs_buf_idx; j++)
    {
        output[enc_idx++] = cobs_buf[j]; // Write the last segment data
    }
    output[enc_idx++] = 0; // Append a zero byte to indicate end of data

    return enc_idx;
}

size_t cobs::decode(const uint8_t *input,const size_t size,size_t *start_idx,size_t end_idx,uint8_t *output)
{
    size_t enc_idx = *start_idx,i=0;
    size_t next_0x00 = 0;
    bool next_is_overhead=false;

    if(input[enc_idx]==0x00){
        enc_idx=(enc_idx+1)%size;
    }
    while(enc_idx!=end_idx){
        if(next_0x00!=0){
            output[i]=input[enc_idx];
            i++;
            enc_idx++;
            enc_idx=enc_idx%size;
        }else{
            if(input[enc_idx]==0x00){
                *start_idx=(enc_idx+1)%size;
                output[i]=next_0x00;
                for(size_t j=1;j<i;j++){
                    output[j-1]=output[j];
                }
                return i-1;
            }else if(!next_is_overhead){
                output[i]=0;
                i++;
            }
            next_0x00=input[enc_idx];
            enc_idx++;
            enc_idx=enc_idx%size;
            next_is_overhead=(next_0x00==0xFF);
        }
        next_0x00--;
    }
    return 0;
}