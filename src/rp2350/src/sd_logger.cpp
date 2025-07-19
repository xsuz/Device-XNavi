#include "sd_logger.h"
#include "byte_utils.h"

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
    constexpr size_t buf_size_row = 16;
    uint8_t buf[buf_size_row][buf_size_col];
    int idx = 0;
    uint8_t row = 0, track = 0;

    char filename[128];
    volatile int64_t offset = 0;
    DWORD fattime = 0;
    void inline get_filename(char *buf, int64_t timestamp);

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
            vTaskDelay(500);
            digitalWrite(LED, LOW);
            vTaskDelay(500);
        }
        SEGGER_RTT_printf(0, "SD card mounted successfully.\n");
        digitalWrite(LED, LOW);

        while (sd_logger::offset == 0)
        {
            vTaskDelay(1000);
        }

        SEGGER_RTT_printf(0, "Timestamp offset set to %lld ms.\n", sd_logger::offset+ millis());
        get_filename(filename, sd_logger::offset + millis());

        while((res = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS)) != FR_OK)
        {
            SEGGER_RTT_printf(0, "Failed to open file %s, retrying...\n", filename);
            digitalWrite(LED, HIGH);
            vTaskDelay(500);
            digitalWrite(LED, LOW);
            vTaskDelay(500);
        }
        SEGGER_RTT_printf(0, "File %s opened successfully.\n", filename);
        f_sync(&fil);
        while (1)
        {

            while (sd_logger::row != sd_logger::track)
            {
                digitalWrite(LED, HIGH);
                f_write(&fil, sd_logger::buf[sd_logger::track], sd_logger::buf_size_col, NULL);
                f_sync(&fil);
                digitalWrite(LED, LOW);

                sd_logger::track++;
                if (sd_logger::track == sd_logger::buf_size_row)
                {
                    sd_logger::track = 0;
                }
            }
            vTaskDelay(10);
        }
    }

    void inline get_filename(char *buf, int64_t timestamp)
    {
        int64_t t = timestamp / 1000;
        uint8_t seconds = t % 60;
        uint8_t minutes = (t / 60) % 60;
        uint8_t hour = (t / 3600) % 24;
        t /= 86400; // days since 1970-01-01 (ex. 1970-01-02 -> 1)
        uint16_t year = 1970;
        uint8_t month = 0;
        uint8_t day = 0;
        int64_t unixtime = 0; // day
        while (unixtime <= t)
        {
            if ((year % 4 == 0) && ((year % 400 == 0) || !(year % 100 == 0)))
            {
                if (t - unixtime < 366)
                {
                    break;
                }
                else
                {
                    year++;
                    unixtime += 366;
                }
            }
            else
            {
                if (t - unixtime < 365)
                {
                    break;
                }
                else
                {
                    unixtime += 365;
                    year++;
                }
            }
        }
        t = t - unixtime;

        if ((year % 4 == 0) && ((year % 400 == 0) || !(year % 100 == 0)))
        {
            if (t >= 335)
            {
                month = 12;
                day += t - 335;
            }
            else if (t >= 304)
            {
                month = 11;
                day += t - 304;
            }
            else if (t >= 274)
            {
                month = 10;
                day += t - 274;
            }
            else if (t >= 243)
            {
                month = 9;
                day += t - 243;
            }
            else if (t >= 213)
            {
                month = 8;
                day += t - 213;
            }
            else if (t >= 182)
            {
                month = 7;
                day += t - 182;
            }
            else if (t >= 152)
            {
                month = 6;
                day += t - 152;
            }
            else if (t >= 121)
            {
                month = 5;
                day += t - 121;
            }
            else if (t >= 91)
            {
                month = 4;
                day += t - 91;
            }
            else if (t >= 60)
            {
                month = 3;
                day += t - 60;
            }
            else if (t >= 31)
            {
                month = 2;
                day = t - 31;
            }
            else
            {
                month = 1;
                day = t;
            }
        }
        else
        {
            if (t >= 334)
            {
                month = 12;
                day += t - 334;
            }
            else if (t >= 304)
            {
                month = 11;
                day += t - 304;
            }
            else if (t >= 273)
            {
                month = 10;
                day += t - 273;
            }
            else if (t >= 243)
            {
                month = 9;
                day += t - 243;
            }
            else if (t >= 212)
            {
                month = 8;
                day += t - 212;
            }
            else if (t >= 181)
            {
                month = 7;
                day += t - 181;
            }
            else if (t >= 151)
            {
                month = 6;
                day += t - 151;
            }
            else if (t >= 120)
            {
                month = 5;
                day += t - 120;
            }
            else if (t >= 90)
            {
                month = 4;
                day += t - 90;
            }
            else if (t >= 59)
            {
                month = 3;
                day += t - 59;
            }
            else if (t >= 31)
            {
                month = 2;
                day = t - 31;
            }
            else
            {
                month = 1;
                day = t;
            }
        }
        day=day+1;

        sprintf(buf, "log_%d%02d%02d_%02d%02d%02d.bin", year, month, day, hour, minutes, seconds);
    }

    void write_pkt(const uint8_t *buffer, size_t size)
    {

        union
        {
            int64_t timestamp;
            uint8_t bytes[8];
        } t2u;

        uint8_t cobs_buf_idx = 0;
        uint8_t cobs_buf[256];

        if (offset == 0)
        {
            return;
        }

        t2u.timestamp = millis() + sd_logger::offset;
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
            if (sd_logger::row == sd_logger::buf_size_row)
            {
                sd_logger::row = 0;
            }
            sd_logger::idx = 0;
        }
    }

    void set_timestamp_offset(int64_t val)
    {
        offset = val;
    }

    void set_timestamp_offset(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
    {
        // calc unix-time offset;
        int dl = year / 4 - year / 100 + year / 400;
        constexpr int dl1970 = 1970 / 4 - 1970 / 100 + 1970 / 400;
        int64_t t = (year - 1970) * 365 + dl - dl1970;
        switch (month)
        {
        case 1:
            t += 0;
            break;
        case 2:
            t += 31;
            break;
        case 3:
            t += 59;
            break;
        case 4:
            t += 90;
            break;
        case 5:
            t += 120;
            break;
        case 6:
            t += 151;
            break;
        case 7:
            t += 181;
            break;
        case 8:
            t += 212;
            break;
        case 9:
            t += 243;
            break;
        case 10:
            t += 273;
            break;
        case 11:
            t += 304;
            break;
        case 12:
            t += 334;
            break;
        default:
            break;
        }
        t += day - 1;
        t *= 24;
        t += hour;
        t *= 60;
        t += minute;
        t *= 60;
        t += second+1;
        offset = t * 1000 - millis();

        // calc fat-time
        if (fattime == 0)
        {
            fattime |= second / 2;
            fattime |= minute << 5;
            fattime |= hour << 11;
            fattime |= day << 16;
            fattime |= month << 21;
            fattime |= (year - 1980) << 25;
        }
    }
}

DWORD get_fattime(void)
{
    return sd_logger::fattime;
}