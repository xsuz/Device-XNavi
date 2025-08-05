#include "sd_logger.h"
#include "byte_utils.h"
#include "clock.h"
#include "readme.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Arduino.h>

#include <ff.h>
#include <SEGGER_RTT.h>

namespace sd_logger
{
    /* Semaphore for access control of buffer*/
    SemaphoreHandle_t xSemaphore = NULL;
    StaticSemaphore_t xMutexBuf;

    constexpr int LED = 10; // Use built-in LED for status indication

    constexpr size_t buf_size_col = 4096;
    constexpr size_t buf_size_row = 4;
    volatile size_t num_to_write = 0;
    uint8_t buf[buf_size_row][buf_size_col];
    int idx = 0;
    uint8_t row = 0, track = 0;

    uint8_t state=0;

    char filename[128];
    void inline get_filename();

    void inline write_raw(uint8_t);

    void task(void *pvParam)
    {
        sd_logger::xSemaphore = xSemaphoreCreateMutexStatic(&sd_logger::xMutexBuf);

        FATFS fs;
        FIL fil;

        FRESULT res;

        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);

        while ((res = f_mount(&fs, "/", 0)) != FR_OK)
        {
            SEGGER_RTT_printf(0, "Failed to mount SD card, retrying...\n");
            digitalWrite(LED, HIGH);
            vTaskDelay(50);
            digitalWrite(LED, LOW);
            vTaskDelay(50);
        }
        SEGGER_RTT_printf(0, "SD card mounted successfully.\n");

        while((res = f_open(&fil,"README.md",FA_WRITE | FA_CREATE_ALWAYS))!=FR_OK)
        {
            SEGGER_RTT_printf(0, "Failed to open file README.md, retrying...\n");
            digitalWrite(LED, HIGH);
            vTaskDelay(500);
            digitalWrite(LED, LOW);
            vTaskDelay(500);
        }
        f_write(&fil,readme::README,strlen(readme::README),NULL);
        f_close(&fil);

        while (!sys_clock::is_valid())
        {
            vTaskDelay(10);
        }

        get_filename();

        while ((res = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS)) != FR_OK)
        {
            SEGGER_RTT_printf(0, "Failed to open file %s, retrying...\n", filename);
            digitalWrite(LED, HIGH);
            vTaskDelay(100);
            digitalWrite(LED, LOW);
            vTaskDelay(100);
        }
        state = 1; // Set state to indicate SD logger is active
        SEGGER_RTT_printf(0, "File %s opened successfully.\n", filename);
        f_sync(&fil);
        while (1)
        {
            if (sd_logger::row != sd_logger::track)
            {
                while ((sd_logger::row != sd_logger::track) && (!BOOTSEL))
                {
                    digitalWrite(LED, HIGH);
                    f_write(&fil, sd_logger::buf[sd_logger::track], sd_logger::buf_size_col, NULL);
                    f_sync(&fil);
                    digitalWrite(LED, LOW);

                    sd_logger::track++;
                    sd_logger::num_to_write--;
                    if (sd_logger::track == sd_logger::buf_size_row)
                    {
                        sd_logger::track = 0;
                    }
                }
            }
            vTaskDelay(10);
        }
    }

    void inline get_filename()
    {
        uint16_t year;
        uint8_t month, day, hour, minutes, seconds;
        sys_clock::get_datetime(&year, &month, &day, &hour, &minutes, &seconds);
        sprintf(sd_logger::filename, "log_%d%02d%02d_%02d%02d%02d.bin", year, month, day, hour, minutes, seconds);
    }

    void write_pkt(const uint8_t *buffer, size_t size,int64_t timestamp)
    {

        union
        {
            int64_t timestamp;
            uint8_t bytes[8];
        } t2u;

        uint8_t cobs_buf_idx = 0;
        uint8_t cobs_buf[256];

        if (!state||!sys_clock::is_valid()||BOOTSEL)
        {
            return;
        }

        t2u.timestamp = timestamp;
        swap64<int64_t>(&t2u.timestamp);

        if (size == 0)
        {
            return;
        }
        xSemaphoreTake(sd_logger::xSemaphore, (TickType_t)portMAX_DELAY);
        for (uint8_t i = 0; i < sizeof(t2u.timestamp); i++)
        {
            if (t2u.bytes[i] == 0x00)
            {
                sd_logger::write_raw(cobs_buf_idx + 1);
                for (uint8_t j = 0; j < cobs_buf_idx; j++)
                {
                    sd_logger::write_raw(cobs_buf[j]);
                }
                cobs_buf_idx = 0;
            }
            else
            {
                cobs_buf[cobs_buf_idx] = t2u.bytes[i];
                cobs_buf_idx++;
            }
        }
        for (uint8_t i = 0; i < size; i++)
        {
            if (buffer[i] == 0x00)
            {
                sd_logger::write_raw(cobs_buf_idx + 1);
                for (uint8_t j = 0; j < cobs_buf_idx; j++)
                {
                    sd_logger::write_raw(cobs_buf[j]);
                }
                cobs_buf_idx = 0;
            }
            else
            {
                cobs_buf[cobs_buf_idx] = buffer[i];
                cobs_buf_idx++;
            }
        }
        sd_logger::write_raw(cobs_buf_idx + 1);
        for (uint8_t j = 0; j < cobs_buf_idx; j++)
        {
            sd_logger::write_raw(cobs_buf[j]);
        }
        sd_logger::write_raw(0x00);
        xSemaphoreGive(sd_logger::xSemaphore);
    }

    void inline write_raw(uint8_t data)
    {
        sd_logger::buf[sd_logger::row][sd_logger::idx] = data;
        sd_logger::idx++;
        if (sd_logger::idx == sd_logger::buf_size_col)
        {
            sd_logger::row++;
            sd_logger::num_to_write++;
            if (sd_logger::row == sd_logger::buf_size_row)
            {
                sd_logger::row = 0;
            }
            if (sd_logger::num_to_write >= sd_logger::buf_size_row)
            {
                SEGGER_RTT_printf(0, "error : queue overflow!\n");
                while (1)
                    ;
            }
            sd_logger::idx = 0;
        }
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