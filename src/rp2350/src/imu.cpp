#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "imu.h"

#include "madgwick.hpp"

#include "DeviceData.h"
#include "byte_utils.h"
#include "sd_logger.h"
#include "clock.h"
#include "canbus.h"

#include <SEGGER_RTT.h>

#include <ASM330LHHSensor.h>
#include <SPI.h>

namespace imu
{
    TaskHandle_t taskHandle;
    QueueHandle_t imuQueue;
    QueueHandle_t utcQueue;

    constexpr int SPI1_CS = 13,
                  SPI1_RX = 12,
                  SPI1_SCK = 14,
                  SPI1_TX = 15;
    
    constexpr int delta_t = 4;
    ASM330LHHSensor asm330lhh(&SPI1, SPI1_CS, 1000000); // SPI1, CS pin 13, SPI speed 1MHz
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    constexpr float deg2rad = M_PI / 180.0f;
    constexpr float acc_sensitivity = ASM330LHH_ACC_SENSITIVITY_FS_2G * 0.00980665f;            // m/s^2
    constexpr float gyro_sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_125DPS * 0.001f * deg2rad; // rad/s
    

    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "imu : IMU task started.\n");

        while(!sys_clock::is_valid())
        {
            // SEGGER_RTT_WriteString(0, "imu : Waiting for system clock to be valid...\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        SEGGER_RTT_printf(0, "imu : Initializing SPI1 with RX: %d, CS: %d, SCK: %d, TX: %d\n", SPI1_RX, SPI1_CS, SPI1_SCK, SPI1_TX);
        // IMU setup
        SPI1.setRX(SPI1_RX);
        SPI1.setCS(SPI1_CS);
        SPI1.setSCK(SPI1_SCK);
        SPI1.setTX(SPI1_TX);
        SPI1.begin();
        SEGGER_RTT_WriteString(0, "imu : SPI1 initialized.\n");

        if (asm330lhh.begin() == ASM330LHH_OK)
        {
            SEGGER_RTT_WriteString(0, "imu : ASM330LHH initialized successfully\n");
        }
        else
        {
            SEGGER_RTT_WriteString(0, "imu : Failed to initialize ASM330LHH\n");
            while (1)
                ; // Halt if initialization fails
        }
        asm330lhh.Enable_X();
        asm330lhh.Enable_G();

        asm330lhh.Set_X_FS(ASM330LHH_2g);
        asm330lhh.Set_G_FS(ASM330LHH_125dps);

        SEGGER_RTT_WriteString(0, "imu : ASM330LHH enabled for accelerometer and gyroscope.\n");

        // Initialize the queue for IMU data
        imuQueue = xQueueCreate(10, sizeof(DeviceData::IMUData));
        utcQueue = xQueueCreate(10, sizeof(int64_t));
        if (imuQueue == NULL)
        {
            SEGGER_RTT_WriteString(0, "imu : Failed to create IMU queue.\n");
            while (1)
                ; // Halt if queue creation fails
        }
        SEGGER_RTT_WriteString(0, "imu : IMU queue created successfully.\n");
        // Create a timer for periodic IMU data collection
        SEGGER_RTT_WriteString(0, "imu : Creating IMU timer...\n");
        auto timerIMU = xTimerCreate("imu", delta_t, pdTRUE, 0, imu::timer_callback);
        xTimerStart(timerIMU, 0);
        SEGGER_RTT_WriteString(0, "imu : IMU timer started.\n");
        while (1)
        {
            union
            {
                DeviceData::IMUData data;
                uint8_t bytes[sizeof(data)];
            } spkt;
            int64_t utc;
            while (uxQueueMessagesWaiting(imuQueue) > 0)
            {
                xQueueReceive(imuQueue, &spkt.data, 0);
                xQueueReceive(utcQueue,&utc,0);
                
                // SEGGER_RTT_printf(0, "imu : accl (%d, %d, %d)[mm/s^2]  gyro (%d, %d, %d)[mdps]\n",
                //                   (int)(spkt.data.a_x * 1000),            // Convert to mm/s^2
                //                   (int)(spkt.data.a_y * 1000),            // Convert to mm/s^2
                //                   (int)(spkt.data.a_z * 1000),            // Convert to mm/s^2
                //                   (int)(spkt.data.w_x / deg2rad * 1000),  // Convert to mdps
                //                   (int)(spkt.data.w_y / deg2rad * 1000),  // Convert to mdps
                //                   (int)(spkt.data.w_z / deg2rad * 1000)); // Convert to mdps

                // Update quaternion using Madgwick filter
                madgwick::update_imu(spkt.data.w_x, spkt.data.w_y, spkt.data.w_z, spkt.data.a_x, spkt.data.a_y, spkt.data.a_z, quat);

                u32::to_le(&spkt.data.timestamp);
                u32::to_le(&spkt.data.a_x);
                u32::to_le(&spkt.data.a_y);
                u32::to_le(&spkt.data.a_z);
                u32::to_le(&spkt.data.w_x);
                u32::to_le(&spkt.data.w_y);
                u32::to_le(&spkt.data.w_z);

                sd_logger::write_pkt(DeviceData::SensorType::IMU, spkt.bytes, sizeof(spkt.bytes),utc);

                union
                {
                    DeviceData::AttitudeData data;
                    uint8_t bytes[sizeof(data)];
                } spkt_att;
                spkt_att.data.timestamp = spkt.data.timestamp;
                spkt_att.data.q0 = u32::to_le(quat[0]);
                spkt_att.data.q1 = u32::to_le(quat[1]);
                spkt_att.data.q2 = u32::to_le(quat[2]);
                spkt_att.data.q3 = u32::to_le(quat[3]);
                sd_logger::write_pkt(DeviceData::SensorType::Attitude, spkt_att.bytes, sizeof(spkt_att.bytes),utc);
            }
            vTaskDelay(20); // Delay to prevent busy-waiting
        }
    }
    void timer_callback(TimerHandle_t xTimer)
    {
        union
        {
            DeviceData::IMUData data;
            uint8_t bytes[sizeof(data)];
        } spkt;
        int64_t utc;
        int16_t acc[3], gyr[3];
        asm330lhh.Get_X_AxesRaw(acc);
        asm330lhh.Get_G_AxesRaw(gyr);
        spkt.data.timestamp = millis();
        spkt.data.a_x = acc[0] * acc_sensitivity;  // a_x(m/s^2)
        spkt.data.a_y = acc[1] * acc_sensitivity;  // a_y(m/s^2)
        spkt.data.a_z = acc[2] * acc_sensitivity;  // a_z(m/s^2)
        spkt.data.w_x = gyr[0] * gyro_sensitivity; // // w_x(rad/s)
        spkt.data.w_y = gyr[1] * gyro_sensitivity; // // w_y(rad/s)
        spkt.data.w_z = gyr[2] * gyro_sensitivity; // // w_z(rad/s)

        utc = sys_clock::get_timestamp();
        xQueueSendFromISR(imuQueue, &spkt.data, NULL);
        xQueueSendFromISR(utcQueue,&utc,NULL);
    }
}