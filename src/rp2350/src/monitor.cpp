#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>

#include "byte_utils.h"
#include "monitor.h"
#include "sd_logger.h"
#include "SEGGER_RTT.h"
#include "DeviceData.h"
#include "clock.h"
#include "canbus.h"

namespace monitor
{
    union
    {
        DeviceData::XNaviStatus status;
        uint8_t raw[sizeof(status)];
    } u;

    /// @brief ログの状態を表す構造体
    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "monitor : Monitor task started.\n");
        analogReadResolution(12);
        pinMode(24,INPUT);

        while (1)
        {
            float voltage=analogRead(29) * (3.3f * 3.0f / 4.096f);
            u.status.timestamp = millis();
            u.status.voltage = (uint16_t)(voltage);
            u.status.percentage = (uint8_t)(123.0f*(1.0f-1.0f/pow(1.0f+pow(voltage/3700.0f,80.0f),0.165f))); // 電圧をパーセンテージに変換
            u.status.status = 0;
            if(sys_clock::is_valid()){
                u.status.status = 1;
            }
            u.status.status|=digitalRead(24)<<1;
            SEGGER_RTT_printf(0, "monitor : timestamp=%d[msec], voltage=%d[mV], percentage=%d[%%], status=%02x\n",
                              u.status.timestamp, u.status.voltage, u.status.percentage, u.status.status);
            sd_logger::write_pkt(DeviceData::SensorType::XNavi ,u.raw, sizeof(u.raw), sys_clock::get_timestamp());

            DeviceData::CANPacket pkt;
            pkt.id=0x235;
            pkt.size=8;
            u32::to_bytes(pkt.payload,0,u.status.timestamp,1);
            u16::to_bytes(pkt.payload,4,u.status.voltage,1);
            pkt.payload[6]=u.status.percentage;
            pkt.payload[7]=u.status.status;
            canbus::write_pkt(pkt);

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 1秒ごとにログを出力
        }
    }

} // namespace monitor