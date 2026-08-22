#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "imu.h"

#include "madgwick.hpp"

#include "clock.h"
#include "config.hpp"

#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

#include <ASM330LHHSensor.h>
#include <SPI.h>

namespace
{
    constexpr int SPI1_CS = 13,
                  SPI1_RX = 12,
                  SPI1_SCK = 14,
                  SPI1_TX = 15;

    constexpr int delta_t = 4;

    ASM330LHHSensor asm330lhh(&SPI1, SPI1_CS, 1000000); // SPI1, CS pin 13, SPI speed 1MHz
    volatile uint32_t imu_queue_overflow = 0;
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    constexpr float deg2rad = M_PI / 180.0f;
    constexpr float acc_sensitivity = 1e3 * ASM330LHH_ACC_SENSITIVITY_FS_2G * 0.00980665f;            // mm/s^2
    constexpr float gyro_sensitivity = 1e3 * ASM330LHH_GYRO_SENSITIVITY_FS_125DPS * 0.001f * deg2rad; // mrad/s
}

void IMUTask::run()
{
    int64_t utc;
    mavlink_message_t msg;
    mavlink_imu_t imu_msg;
    uint8_t count = 0;

    SEGGER_RTT_printf(0, "[%sINFO%s imu] : IMU task started.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    SEGGER_RTT_printf(0, "[%sINFO%s imu] : Initializing SPI1 with RX: %d, CS: %d, SCK: %d, TX: %d\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET, SPI1_RX, SPI1_CS, SPI1_SCK, SPI1_TX);
    // IMU setup
    SPI1.setRX(SPI1_RX);
    SPI1.setCS(SPI1_CS);
    SPI1.setSCK(SPI1_SCK);
    SPI1.setTX(SPI1_TX);
    SPI1.begin();
    SEGGER_RTT_printf(0, "[%sINFO%s imu] : SPI1 initialized.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    if (asm330lhh.begin() == ASM330LHH_OK)
    {
        SEGGER_RTT_printf(0, "[%sINFO%s imu] : ASM330LHH initialized successfully\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
    }
    else
    {
        SEGGER_RTT_printf(0, "[%sERROR%s imu] : Failed to initialize ASM330LHH\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
        while (1)
            ; // Halt if initialization fails
    }
    asm330lhh.Enable_X();
    asm330lhh.Enable_G();

    asm330lhh.Set_X_FS(ASM330LHH_2g);
    asm330lhh.Set_G_FS(ASM330LHH_125dps);

    SEGGER_RTT_printf(0, "[%sINFO%s imu] : ASM330LHH enabled for accelerometer and gyroscope.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&last_wake_time, delta_t);

        utc = sys_clock::get_timestamp();
        int16_t acc[3], gyr[3];
        asm330lhh.Get_X_AxesRaw(acc);
        asm330lhh.Get_G_AxesRaw(gyr);
        imu_msg.time_boot_ms = millis();
        imu_msg.xacc = (int16_t)(acc[0] * acc_sensitivity);   // a_x(m/s^2)
        imu_msg.yacc = (int16_t)(acc[1] * acc_sensitivity);   // a_y(m/s^2)
        imu_msg.zacc = (int16_t)(acc[2] * acc_sensitivity);   // a_z(m/s^2)
        imu_msg.xgyro = (int16_t)(gyr[0] * gyro_sensitivity); // // w_x(rad/s)
        imu_msg.ygyro = (int16_t)(gyr[1] * gyro_sensitivity); // // w_y(rad/s)
        imu_msg.zgyro = (int16_t)(gyr[2] * gyro_sensitivity); // // w_z(rad/s)

        mavlink_msg_imu_encode(config::mavlink::system_id, config::mavlink::component_id, &msg, &imu_msg);
        _publisher.publish(msg, utc);
    }
}