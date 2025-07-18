#include <FreeRTOS.h>
#include <task.h>

#include "imu.h"

#include "madgwick.hpp"

#include "SensorPacket.h"
#include "byte_utils.h"
#include "sd_logger.h"

#include <SEGGER_RTT.h>

#include <ASM330LHHSensor.h>
#include <SPI.h>

namespace imu
{
    constexpr int SPI1_CS=13,
                  SPI1_RX=12,
                  SPI1_SCK=14,
                  SPI1_TX=15;
    ASM330LHHSensor asm330lhh(&SPI1, SPI1_CS, 1000000); // SPI1, CS pin 13, SPI speed 1MHz
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    constexpr float deg2rad = M_PI / 180.0f;
    constexpr int LED = 11;
    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "IMU task started.\n");

        SEGGER_RTT_printf(0, "Initializing SPI1 with RX: %d, CS: %d, SCK: %d, TX: %d\n", SPI1_RX, SPI1_CS, SPI1_SCK, SPI1_TX);
        // IMU setup
        SPI1.setRX(SPI1_RX);
        SPI1.setCS(SPI1_CS);
        SPI1.setSCK(SPI1_SCK);
        SPI1.setTX(SPI1_TX);
        SPI1.begin();
        SEGGER_RTT_WriteString(0, "SPI1 initialized.\n");
        pinMode(LED,OUTPUT);
        digitalWrite(LED, LOW);
        
        if(asm330lhh.begin()==ASM330LHH_OK)
        {
            SEGGER_RTT_WriteString(0, "ASM330LHH initialized successfully\n");
        }
        else
        {
            SEGGER_RTT_WriteString(0, "Failed to initialize ASM330LHH\n");
            while(1); // Halt if initialization fails
        }
        asm330lhh.Enable_X();
        asm330lhh.Enable_G();

        SEGGER_RTT_WriteString(0, "ASM330LHH enabled for accelerometer and gyroscope.\n");
        auto timerIMU = xTimerCreate("imu", 10, pdTRUE, 0, imu::timer_callback);
        xTimerStart(timerIMU, 0);
        SEGGER_RTT_WriteString(0, "IMU timer started.\n");
        while (1)
        {
            vTaskDelay(100000);
        }
    }
    void timer_callback(TimerHandle_t xTimer)
    {
        union
        {
            IMUData data;
            uint8_t bytes[sizeof(data)];
        } spkt;
        int32_t acc[3], gyr[3];
        digitalWrite(LED, HIGH);
        asm330lhh.Get_X_Axes(acc);
        asm330lhh.Get_G_Axes(gyr);
        digitalWrite(LED, LOW);
        spkt.data.id = 0x40;
        spkt.data.timestamp = millis();
        spkt.data.a_x = acc[0] * 0.00980665f; // a_x(m/s^2)
        spkt.data.a_y = acc[1] * 0.00980665f; // a_y(m/s^2)
        spkt.data.a_z = acc[2] * 0.00980665f; // a_z(m/s^2)
        spkt.data.w_x = gyr[0] * 0.001 * deg2rad;// // w_x(rad/s)
        spkt.data.w_y = gyr[1] * 0.001 * deg2rad;// // w_x(rad/s)
        spkt.data.w_z = gyr[2] * 0.001 * deg2rad;// // w_x(rad/s)

        madgwick::update_imu(spkt.data.w_x, spkt.data.w_y, spkt.data.w_z, spkt.data.a_x, spkt.data.a_y, spkt.data.a_z, quat);
        spkt.data.q0 = quat[0];
        spkt.data.q1 = quat[1];
        spkt.data.q2 = quat[2];
        spkt.data.q3 = quat[3];

        swap32<uint32_t>(&spkt.data.timestamp);
        swap32<float>(&spkt.data.a_x);
        swap32<float>(&spkt.data.a_y);
        swap32<float>(&spkt.data.a_z);
        swap32<float>(&spkt.data.w_x);
        swap32<float>(&spkt.data.w_y);
        swap32<float>(&spkt.data.w_z);
        swap32<float>(&spkt.data.m_x);
        swap32<float>(&spkt.data.m_y);
        swap32<float>(&spkt.data.m_z);
        swap32<float>(&spkt.data.q0);
        swap32<float>(&spkt.data.q1);
        swap32<float>(&spkt.data.q2);
        swap32<float>(&spkt.data.q3);
        sd_logger::write_pkt(spkt.bytes, sizeof(spkt.bytes));
    }
}