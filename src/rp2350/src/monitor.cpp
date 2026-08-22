#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>

#include <mavlink/swingby/mavlink.h>
#include <SEGGER_RTT.h>

#include "monitor.h"
#include "clock.h"
#include "config.hpp"

void MonitorTask::run()
{
    while (1)
    {
        mavlink_message_t msg;
        mavlink_battery_status_t battery;
        SEGGER_RTT_printf(0, "[%sINFO%s monitor] : Monitor task started.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
        analogReadResolution(12);
        pinMode(24, INPUT);

        while (1)
        {
            float voltage = analogRead(29) * (3.3f * 3.0f / 4.096f); // VSYSの電圧
            battery.charge_state = MAV_BATTERY_CHARGE_STATE::MAV_BATTERY_CHARGE_STATE_CHARGING;
            battery.current_consumed = -1;
            battery.energy_consumed = -1;
            battery.temperature = INT16_MAX;
            battery.voltages[0] = (uint16_t)(voltage);
            for(int i=1;i<10;i++){
                battery.voltages[i]=0;
            }
            for(int i=0;i<4;i++){
                battery.voltages_ext[i]=0;
            }
            battery.current_battery = -1;
            battery.id = 0;
            battery.battery_function = MAV_BATTERY_FUNCTION::MAV_BATTERY_FUNCTION_AVIONICS;
            battery.type = MAV_BATTERY_TYPE::MAV_BATTERY_TYPE_LIPO;
            battery.battery_remaining = (int8_t)(123.0f * (1.0f - 1.0f / pow(1.0f + pow(voltage / 3700.0f, 80.0f), 0.165f)));
            battery.time_remaining = 0;
            battery.charge_state = MAV_BATTERY_CHARGE_STATE::MAV_BATTERY_CHARGE_STATE_CHARGING;
            battery.mode = MAV_BATTERY_MODE::MAV_BATTERY_MODE_UNKNOWN;
            battery.fault_bitmask = 0;
            mavlink_msg_battery_status_encode(config::mavlink::system_id, config::mavlink::component_id, &msg, &battery);
            _publisher.publish(msg, sys_clock::get_timestamp());
            SEGGER_RTT_printf(0, "[%sINFO%s monitor] : timestamp=%d[msec], voltage=%d[mV], percentage=%d[%%]\n",
                              RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET,
                              millis(), battery.voltages[0], battery.battery_remaining);

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 1秒ごとにログを出力
        }
    }
}
namespace monitor
{
    /// @brief ログの状態を表す構造体
    void task(void *pvParam)
    {
    }

} // namespace monitor