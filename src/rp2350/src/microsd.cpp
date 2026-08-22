#include "microsd.h"
#include "clock.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Arduino.h>

#include <ff.h>
#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

void uSDTask::run()
{
    mavlink_message_t msg;
    int64_t timestamp;

    FATFS fs;
    FIL fil;
    FRESULT res;

    char filename[128];

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    SEGGER_RTT_printf(0, "[%sINFO%s uSD] : Initializing uSD task.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    while ((res = f_mount(&fs, "/", 0)) != FR_OK)
    {
        if (_consumer.receive(msg, timestamp, 10))
        {
            SEGGER_RTT_printf(0, "[%sERROR%s uSD] : Failed to mount SD card, retrying...\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
            digitalWrite(LED, HIGH);
            vTaskDelay(1);
            digitalWrite(LED, LOW);
        }
    }
    SEGGER_RTT_printf(0, "[%sINFO%s uSD] : SD card mounted successfully.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    while (!sys_clock::is_valid())
    {
        if (_consumer.receive(msg, timestamp, 9))
        {
            digitalWrite(LED, HIGH);
            vTaskDelay(1);
            // SEGGER_RTT_printf(0, "[%sINFO%s uSD] : Waiting for clock's configuration...\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET, filename);
            digitalWrite(LED, LOW);
        }
    }

    get_filename(filename);

    while ((res = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS)) != FR_OK)
    {
        if (_consumer.receive(msg, timestamp, 100))
        {
            SEGGER_RTT_printf(0, "[%sERROR%s uSD] : Failed to open file %s, retrying...\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET, filename);
        }
    }
    SEGGER_RTT_printf(0, "[%sINFO%s uSD] : File %s opened successfully.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET, filename);
    f_sync(&fil);
    while (1)
    {
        if (_consumer.receive(msg, timestamp, 1))
        {
            write_pkt(msg, timestamp);
        }
        if (ready_blocks > 0)
        {
            digitalWrite(LED, HIGH);
            f_write(&fil, uSDTask::buffer[uSDTask::read_index], BLOCK_SIZE, NULL);
            f_sync(&fil);
            uSDTask::read_index = (uSDTask::read_index + 1) % BLOCK_COUNT;
            uSDTask::ready_blocks--;
            digitalWrite(LED, LOW);
        }
    }
}

inline void uSDTask::write_pkt(const mavlink_message_t &msg, int64_t timestamp)
{
    if (!sys_clock::is_valid())
    {
        return;
    }
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN + 8];

    // Big-Endianでtimestampを保存する
    
    uint64_t uvalue=static_cast<uint64_t>(timestamp);
    buffer[0]=static_cast<uint8_t>(uvalue>>(8*7)) & 0xFF;
    buffer[1]=static_cast<uint8_t>(uvalue>>(8*6)) & 0xFF;
    buffer[2]=static_cast<uint8_t>(uvalue>>(8*5)) & 0xFF;
    buffer[3]=static_cast<uint8_t>(uvalue>>(8*4)) & 0xFF;
    buffer[4]=static_cast<uint8_t>(uvalue>>(8*3)) & 0xFF;
    buffer[5]=static_cast<uint8_t>(uvalue>>(8*2)) & 0xFF;
    buffer[6]=static_cast<uint8_t>(uvalue>>(8*1)) & 0xFF;
    buffer[7]=static_cast<uint8_t>(uvalue>>(8*0)) & 0xFF;

    size_t size = mavlink_msg_to_send_buffer(buffer + 8, &msg);
    write_record(buffer, size);
}

void inline uSDTask::get_filename(char *filename)
{
    uint16_t year;
    uint8_t month, day, hour, minutes, seconds;
    sys_clock::get_datetime(&year, &month, &day, &hour, &minutes, &seconds);
    sprintf(filename, "log_%d%02d%02d_%02d%02d%02d.bin", year, month, day, hour, minutes, seconds);
}
void inline uSDTask::write_record(const uint8_t *buffer, size_t size)
{
    uint8_t cobs_buf_idx = 0;
    uint8_t cobs_buf[256];

    if (available_ring_buffer_size_unsafe() < size)
    {
        SEGGER_RTT_printf(0, "[%sWARNING%s uSD] : Ring buffer overflow. The record cannot be saved.\n", RTT_CTRL_TEXT_YELLOW, RTT_CTRL_RESET);
        return;
    }

    for (size_t i = 0; i < size; i++)
    {
        if (buffer[i] == 0)
        {
            write_ring_buffer(cobs_buf_idx + 1); // Write the length of the segment
            for (uint8_t j = 0; j < cobs_buf_idx; j++)
            {
                write_ring_buffer(cobs_buf[j]); // Write the segment data
            }
            cobs_buf_idx = 0; // Reset for next segment
        }
        else
        {
            cobs_buf[cobs_buf_idx++] = buffer[i];
            if (cobs_buf_idx == 254)
            {
                write_ring_buffer(0xFF); // Write the length of the segment
                for (uint8_t j = 0; j < 254; j++)
                {
                    write_ring_buffer(cobs_buf[j]); // Write the segment data
                }
                cobs_buf_idx = 0; // Reset for next segment
            }
        }
    }
    write_ring_buffer(cobs_buf_idx + 1); // Write the length of the last segment
    for (uint8_t j = 0; j < cobs_buf_idx; j++)
    {
        write_ring_buffer(cobs_buf[j]); // Write the last segment data
    }
    write_ring_buffer(0x00); // Append a zero byte to indicate end of data
}

void inline uSDTask::write_ring_buffer(uint8_t data)
{
    buffer[write_index][write_offset++] = data;
    if (write_offset >= BLOCK_SIZE)
    {
        write_offset = 0;
        if (ready_blocks >= BLOCK_COUNT - 1)
        {
            // Buffer overflow, handle error
            SEGGER_RTT_printf(0, "[%sERROR%s uSD] : Ring buffer overflow.\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
            return;
        }
        write_index = (write_index + 1) % BLOCK_COUNT;
        ready_blocks++;
    }
}

inline size_t uSDTask::available_ring_buffer_size_unsafe()
{
    return BLOCK_SIZE * (BLOCK_COUNT - ready_blocks - 1) + (BLOCK_SIZE - write_offset);
}

DWORD get_fattime(void)
{
    uint16_t year;
    uint8_t month, day, hour, minutes, seconds;
    sys_clock::get_datetime(&year, &month, &day, &hour, &minutes, &seconds);
    // calc fat-time

    DWORD fattime = 0;
    fattime |= seconds / 2;
    fattime |= minutes << 5;
    fattime |= hour << 11;
    fattime |= day << 16;
    fattime |= month << 21;
    fattime |= (year - 1980) << 25;
    return fattime;
}