#include "clock.h"

#include <Arduino.h>

#include <SEGGER_RTT.h>

namespace sys_clock
{
    int64_t offset = 0;

    int64_t get_timestamp(){
        
        return millis()+offset;
    }

    void set_timestamp_offset(uint32_t tick_millis,uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
    {
        // SEGGER_RTT_printf(0,"%d -> %d/%02d/%02d %02d:%02d:%02d\n",tick_millis,year,month,day,hour,minute,second);
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
        if((year % 4 == 0 && year % 100 != 0 || year % 400 == 0) && (month > 2))
        {
            t += 1; // leap year
        }
        t += day - 1;
        t *= 24;
        t += hour;
        t *= 60;
        t += minute;
        t *= 60;
        t += second;
        offset = t * 1000 - (int64_t)tick_millis;
    }

    bool get_datetime(uint16_t *year,uint8_t *month,uint8_t *day,uint8_t *hour,uint8_t *minutes,uint8_t *seconds){

        if(!is_valid()){
            return false;
        }
        
        int64_t t = get_timestamp() / 1000;
        *seconds = t % 60;
        *minutes = (t / 60) % 60;
        *hour = (t / 3600) % 24;
        t /= 86400; // days since 1970-01-01 (ex. 1970-01-02 -> 1)
        *year = 1970;
        *month = 0;
        *day = 0;
        int64_t day_from_1970 = 0; // day
        while (day_from_1970 <= t)
        {
            if ((*year % 4 == 0) && ((*year % 400 == 0) || !(*year % 100 == 0)))
            {
                if (t - day_from_1970 < 366)
                {
                    break;
                }
                else
                {
                    (*year)++;
                    day_from_1970 += 366;
                }
            }
            else
            {
                if (t - day_from_1970 < 365)
                {
                    break;
                }
                else
                {
                    day_from_1970 += 365;
                    (*year)++;
                }
            }
        }
        t = t - day_from_1970;

        if ((*year % 4 == 0) && ((*year % 400 == 0) || !(*year % 100 == 0)))
        {
            if (t >= 335)
            {
                (*month) = 12;
                (*day) += t - 335;
            }
            else if (t >= 304)
            {
                (*month) = 11;
                (*day) += t - 304;
            }
            else if (t >= 274)
            {
                (*month) = 10;
                (*day) += t - 274;
            }
            else if (t >= 243)
            {
                (*month) = 9;
                (*day) += t - 243;
            }
            else if (t >= 213)
            {
                (*month) = 8;
                (*day) += t - 213;
            }
            else if (t >= 182)
            {
                (*month) = 7;
                (*day) += t - 182;
            }
            else if (t >= 152)
            {
                (*month) = 6;
                (*day) += t - 152;
            }
            else if (t >= 121)
            {
                (*month) = 5;
                (*day) += t - 121;
            }
            else if (t >= 91)
            {
                (*month) = 4;
                (*day) += t - 91;
            }
            else if (t >= 60)
            {
                (*month) = 3;
                (*day) += t - 60;
            }
            else if (t >= 31)
            {
                (*month) = 2;
                (*day) = t - 31;
            }
            else
            {
                (*month) = 1;
                (*day) = t;
            }
        }
        else
        {
            if (t >= 334)
            {
                (*month) = 12;
                (*day) += t - 334;
            }
            else if (t >= 304)
            {
                (*month) = 11;
                (*day) += t - 304;
            }
            else if (t >= 273)
            {
                (*month) = 10;
                (*day) += t - 273;
            }
            else if (t >= 243)
            {
                (*month) = 9;
                (*day) += t - 243;
            }
            else if (t >= 212)
            {
                (*month) = 8;
                (*day) += t - 212;
            }
            else if (t >= 181)
            {
                (*month) = 7;
                (*day) += t - 181;
            }
            else if (t >= 151)
            {
                (*month) = 6;
                (*day) += t - 151;
            }
            else if (t >= 120)
            {
                (*month) = 5;
                (*day) += t - 120;
            }
            else if (t >= 90)
            {
                (*month) = 4;
                (*day) += t - 90;
            }
            else if (t >= 59)
            {
                (*month) = 3;
                (*day) += t - 59;
            }
            else if (t >= 31)
            {
                (*month) = 2;
                (*day) = t - 31;
            }
            else
            {
                (*month) = 1;
                (*day) = t;
            }
        }
        (*day) = (*day) + 1;
        // SEGGER_RTT_printf(0, "date: %d/%02d/%02d %02d:%02d:%02d\n", *year, *month, *day, *hour, *minutes, *seconds);
        return true;
    }

    bool is_valid(){
        return offset > 0;
    }
} // namespace clock
