#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>

#include <mavlink/swingby/mavlink.h>
#include <SEGGER_RTT.h>

#include "monitor.h"
#include "sd_logger.h"
#include "clock.h"
#include "canbus.h"

namespace monitor
{
    /// @brief ログの状態を表す構造体
    void task(void *pvParam)
    {
        mavlink_message_t msg;
        mavlink_sys_status_t sys_status;
        SEGGER_RTT_printf(0, "[%sINFO%s monitor] : Monitor task started.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
        analogReadResolution(12);
        pinMode(24, INPUT);

        while (1)
        {
            float voltage = analogRead(29) * (3.3f * 3.0f / 4.096f);
            sys_status.voltage_battery = (uint16_t)(voltage * 1000.0f);
            sys_status.current_battery = -1;                                                                                     // Current not sent by autopilot
            sys_status.battery_remaining = (int8_t)(123.0f * (1.0f - 1.0f / pow(1.0f + pow(voltage / 3700.0f, 80.0f), 0.165f))); // 電圧をパーセンテージに変換
            sys_status.load = 0;                                                                                                 // Load not sent by autopilot
            sys_status.onboard_control_sensors_present_extended = 0;                                                             // Sensors present not sent by autopilot
            sys_status.onboard_control_sensors_enabled_extended = 0;                                                             // Sensors enabled not sent by autopilot
            sys_status.onboard_control_sensors_health_extended = 0;                                                              // Sensors health not sent by autopilot
            sys_status.drop_rate_comm = 0;                                                                                       // Communication drop rate not sent by autopilot
            sys_status.errors_comm = !sd_logger::is_valid();                                                                     // Communication errors not sent by autopilot
            sys_status.errors_count1 = sys_clock::is_valid();                                                                    // Autopilot-specific errors not sent by autopilot
            sys_status.errors_count2 = 0;                                                                                        // Autopilot-specific errors not sent by autopilot
            sys_status.errors_count3 = 0;                                                                                        // Autopilot-specific errors not sent by autopilot
            sys_status.errors_count4 = 0;                                                                                        // Autopilot-specific errors not sent by autopilot
            mavlink_msg_sys_status_encode(1, 0, &msg, &sys_status);
            sd_logger::write_pkt(&msg, sys_clock::get_timestamp());
            SEGGER_RTT_printf(0, "[%sINFO%s monitor] : timestamp=%d[msec], voltage=%d[mV], percentage=%d[%%]\n",
                              RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET,
                              millis(), sys_status.voltage_battery, sys_status.battery_remaining);

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 1秒ごとにログを出力
        }
    }

} // namespace monitor