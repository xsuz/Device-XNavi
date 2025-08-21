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

    FATFS fs;
    FIL fil;

    constexpr size_t buf_size_col = 4096;
    constexpr size_t buf_size_row = 4;
    volatile size_t num_to_write = 0;
    uint8_t buf[buf_size_row][buf_size_col];
    int idx = 0;
    uint8_t row = 0, track = 0;

    uint8_t state=0;

    char filename[128];
    void inline get_filename();

    
    uint8_t cobs_buf_idx = 0;
    uint8_t cobs_buf[256];
    void inline write_raw(uint8_t);
    void inline write_partial(const uint8_t *buffer,size_t size);

    void task(void *pvParam)
    {
        sd_logger::xSemaphore = xSemaphoreCreateMutexStatic(&sd_logger::xMutexBuf);

        FRESULT res;

        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);

        while ((res = f_mount(&fs, "/", 0)) != FR_OK)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Failed to mount SD card, retrying...\n",RTT_CTRL_TEXT_RED,RTT_CTRL_RESET);
            digitalWrite(LED, HIGH);
            vTaskDelay(50);
            digitalWrite(LED, LOW);
            vTaskDelay(50);
        }
        SEGGER_RTT_printf(0, "[%sINFO%s sd_logger] : SD card mounted successfully.\n",RTT_CTRL_TEXT_GREEN,RTT_CTRL_RESET);

        while((res = f_open(&fil,"README.md",FA_WRITE | FA_CREATE_ALWAYS))!=FR_OK)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Failed to open file README.md, retrying...\n",RTT_CTRL_TEXT_RED,RTT_CTRL_RESET);
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
            SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : Failed to open file %s, retrying...\n",RTT_CTRL_TEXT_RED,RTT_CTRL_RESET, filename);
            digitalWrite(LED, HIGH);
            vTaskDelay(100);
            digitalWrite(LED, LOW);
            vTaskDelay(100);
        }
        state = 1; // Set state to indicate SD logger is active
        SEGGER_RTT_printf(0, "[%sINFO%s sd_logger] : File %s opened successfully.\n",RTT_CTRL_TEXT_GREEN,RTT_CTRL_RESET, filename);
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

    uint8_t is_valid(){
        return state && sys_clock::is_valid() && (!BOOTSEL);
    }

    void inline get_filename()
    {
        uint16_t year;
        uint8_t month, day, hour, minutes, seconds;
        sys_clock::get_datetime(&year, &month, &day, &hour, &minutes, &seconds);
        sprintf(sd_logger::filename, "log_%d%02d%02d_%02d%02d%02d.bin", year, month, day, hour, minutes, seconds);
    }

    
    void write_pkt(uint32_t id, const uint8_t *buffer, size_t size,int64_t timestamp)
    {
        union
        {
            uint32_t id;
            uint8_t bytes[8];
        } i2u;
        union
        {
            int64_t timestamp;
            uint8_t bytes[8];
        } t2u;
        
        i2u.id=id;
        t2u.timestamp=timestamp;
        u32::to_le(&i2u.id);
        u64::to_le(&t2u.timestamp);

        if (!state||!sys_clock::is_valid()||BOOTSEL)
        {
            return;
        }

        if (size == 0)
        {
            return;
        }
        xSemaphoreTake(sd_logger::xSemaphore, (TickType_t)portMAX_DELAY);
        cobs_buf_idx=0;
        write_partial(t2u.bytes,8);
        write_partial(i2u.bytes,4);
        write_partial(buffer,size);
        sd_logger::write_raw(cobs_buf_idx + 1);
        for (uint8_t j = 0; j < cobs_buf_idx; j++)
        {
            sd_logger::write_raw(cobs_buf[j]);
        }
        sd_logger::write_raw(0x00);
        xSemaphoreGive(sd_logger::xSemaphore);
    }

    void write_bytes(const uint8_t *buffer, size_t size,int64_t timestamp)
    {
        union
        {
            int64_t timestamp;
            uint8_t bytes[8];
        } t2u;
        t2u.timestamp=timestamp;
        u64::to_le(&t2u.timestamp);

        if (!state||!sys_clock::is_valid()||BOOTSEL)
        {
            return;
        }

        if (size == 0)
        {
            return;
        }
        xSemaphoreTake(sd_logger::xSemaphore, (TickType_t)portMAX_DELAY);
        cobs_buf_idx=0;
        write_partial(t2u.bytes,8);
        write_partial(buffer,size);
        sd_logger::write_raw(cobs_buf_idx + 1);
        for (uint8_t j = 0; j < cobs_buf_idx; j++)
        {
            sd_logger::write_raw(cobs_buf[j]);
        }
        sd_logger::write_raw(0x00);
        xSemaphoreGive(sd_logger::xSemaphore);
    }

    void inline write_partial(const uint8_t *buffer,size_t size){
        for (uint8_t i = 0; i < size; i++)
        {
            if (buffer[i] == 0x00)
            {
                sd_logger::write_raw(sd_logger::cobs_buf_idx + 1);
                for (uint8_t j = 0; j < sd_logger::cobs_buf_idx; j++)
                {
                    sd_logger::write_raw(sd_logger::cobs_buf[j]);
                }
                sd_logger::cobs_buf_idx = 0;
            }
            else
            {
                sd_logger::cobs_buf[sd_logger::cobs_buf_idx] = buffer[i];
                sd_logger::cobs_buf_idx++;
            }
        }
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
                SEGGER_RTT_printf(0, "[%sERROR%s sd_logger] : queue overflow!\n",RTT_CTRL_TEXT_RED,RTT_CTRL_RESET);
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