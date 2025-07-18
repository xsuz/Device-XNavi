#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include <SEGGER_RTT.h>

#include "sd_logger.h"
#include "gnss.h"
#include "imu.h"
#include "twelite.h"

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

void setup()
{
    // put your setup code here, to run once:
    SEGGER_RTT_Init();
    Serial.begin(115200);

    xTaskCreate(gnss::task,"gps_task",512,NULL,3,&Task1);
    xTaskCreate(imu::task,"imu_task",512,NULL,4,&Task2);
    xTaskCreate(sd_logger::task,"sd_buf_task",1024,NULL,1,&Task3);
    xTaskCreate(twelite::task,"twelite_task",256,NULL,2,&Task4);
    vTaskStartScheduler();
}

void loop()
{
    // put your main code here, to run repeatedly:
}

void setup1(){
    
    // TWELITEのUARTを初期化
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1(){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
