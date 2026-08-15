#include "sd_logger.h"
#include "clock.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Arduino.h>

#include <ff.h>
#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

namespace sd_logger
{
    /* Semaphore for access control of buffer*/
    SemaphoreHandle_t xSemaphore = NULL;
    StaticSemaphore_t xMutexBuf;

    constexpr int LED = 10; // Use built-in LED for status indication

    FATFS fs;
    FIL fil;

    constexpr size_t MAX_WRITE_BYTES_SIZE = MAVLINK_MAX_PACKET_LEN - 1;

    constexpr size_t BLOCK_SIZE = 4096;
    constexpr size_t BLOCK_COUNT = 4;

    uint8_t buffer[BLOCK_COUNT][BLOCK_SIZE];

    size_t write_index = 0;
    size_t write_offset = 0;

    size_t read_index = 0;
    size_t ready_blocks = 0;

    uint8_t state = 0;

    char filename[128];
    void inline get_filename(char *filename);

    void inline write_ring_buffer(uint8_t);
    void inline write_record(const uint8_t *buffer, size_t size);
    size_t inline available_ring_buffer_size_unsafe();

    void task(void *pvParam)
    {
        sd_logger::xSemaphore = xSemaphoreCreateMutexStatic(&sd_logger::xMutexBuf);

        FRESULT res;

        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);

        while ((res = f_mount(&fs, "/", 0)) != FR_OK)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Failed to mount SD card, retrying...\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
            digitalWrite(LED, HIGH);
            vTaskDelay(50);
            digitalWrite(LED, LOW);
            vTaskDelay(50);
        }
        SEGGER_RTT_printf(0, "[%sINFO%s sd_logger] : SD card mounted successfully.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

        while (!sys_clock::is_valid())
        {
            vTaskDelay(10);
        }

        get_filename(sd_logger::filename);

        while ((res = f_open(&fil, sd_logger::filename, FA_WRITE | FA_CREATE_ALWAYS)) != FR_OK)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Failed to open file %s, retrying...\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET, sd_logger::filename);
            digitalWrite(LED, HIGH);
            vTaskDelay(100);
            digitalWrite(LED, LOW);
            vTaskDelay(100);
        }
        state = 1; // Set state to indicate SD logger is active
        SEGGER_RTT_printf(0, "[%sINFO%s sd_logger] : File %s opened successfully.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET, sd_logger::filename);
        f_sync(&fil);
        while (1)
        {
            if (ready_blocks > 0)
            {
                digitalWrite(LED, HIGH);
                f_write(&fil, sd_logger::buffer[sd_logger::read_index], BLOCK_SIZE, NULL);
                f_sync(&fil);
                sd_logger::read_index = (sd_logger::read_index + 1) % BLOCK_COUNT;
                sd_logger::ready_blocks--;
                digitalWrite(LED, LOW);
            }
            vTaskDelay(10);
        }
    }

    uint8_t is_valid()
    {
        return state && sys_clock::is_valid() && (!BOOTSEL);
    }

    void inline get_filename(char *filename)
    {
        uint16_t year;
        uint8_t month, day, hour, minutes, seconds;
        sys_clock::get_datetime(&year, &month, &day, &hour, &minutes, &seconds);
        sprintf(filename, "log_%d%02d%02d_%02d%02d%02d.bin", year, month, day, hour, minutes, seconds);
    }

    void write_pkt(const mavlink_message_t *msg, int64_t timestamp)
    {
        if (!state || !sys_clock::is_valid() || BOOTSEL)
        {
            return;
        }
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN + 8];
        
        memcpy(buffer,&timestamp,8);
        size_t size = mavlink_msg_to_send_buffer(buffer + 8, msg);
        write_record(buffer, size);
    }

    void write_bytes(const uint8_t *buffer, size_t size, int64_t timestamp)
    {
        const uint8_t ID_RAW_DATA = 0x00; // Define a unique ID for raw data packets

        if (size > MAX_WRITE_BYTES_SIZE)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Data size exceeds maximum allowed size of %d bytes.\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET, MAX_WRITE_BYTES_SIZE);
            return;
        }

        if (!state || !sys_clock::is_valid() || BOOTSEL)
        {
            return;
        }

        if (size == 0)
        {
            return;
        }

        uint8_t data_with_timestamp[size + 9]; // 1 byte for ID, 8 bytes for timestamp
        data_with_timestamp[0] = ID_RAW_DATA;
        memcpy(data_with_timestamp + 1, &timestamp,8);
        memcpy(data_with_timestamp + 9, buffer, size);

        write_record(data_with_timestamp, size + 9);
    }

    void inline write_record(const uint8_t *buffer, size_t size)
    {
        xSemaphoreTake(sd_logger::xSemaphore, (TickType_t)portMAX_DELAY);
        uint8_t cobs_buf_idx = 0;
        uint8_t cobs_buf[256];

        if (available_ring_buffer_size_unsafe()<size){
            SEGGER_RTT_printf(0, "[%sWARNING%s sd_logger] : Ring buffer overflow. The record cannot be saved.\n", RTT_CTRL_TEXT_YELLOW, RTT_CTRL_RESET);
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
        xSemaphoreGive(sd_logger::xSemaphore);
    }

void inline write_ring_buffer(uint8_t data)
{
    buffer[write_index][write_offset++] = data;
    if (write_offset >= BLOCK_SIZE)
    {
        write_offset = 0;
        if (ready_blocks >= BLOCK_COUNT - 1)
        {
            // Buffer overflow, handle error
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Ring buffer overflow.\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
            return;
        }
        write_index = (write_index + 1) % BLOCK_COUNT;
        ready_blocks++;
    }
}

    inline size_t available_ring_buffer_size_unsafe(){
        return BLOCK_SIZE*(BLOCK_COUNT-ready_blocks-1)+(BLOCK_SIZE-write_offset);
    }

    size_t available_ring_buffer_size(){
        xSemaphoreTake(sd_logger::xSemaphore, (TickType_t)portMAX_DELAY);
        const size_t size=available_ring_buffer_size_unsafe();
        xSemaphoreGive(sd_logger::xSemaphore);
        return size;
    }
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