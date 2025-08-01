#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <map>

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

    xTaskCreate(gnss::task,"gps",512,NULL,3,&Task1);
    xTaskCreate(imu::task,"imu",256,NULL,1,&Task2);
    xTaskCreate(sd_logger::task,"sd",1024,NULL,1,&Task3);
    xTaskCreate(twelite::task,"twelite",256,NULL,2,&Task4);
    vTaskStartScheduler();
}

void loop()
{
    // put your main code here, to run repeatedly:
}

std::map<eTaskState, const char *> eTaskStateName { {eReady, "Ready"}, { eRunning, "Running" }, {eBlocked, "Blocked"}, {eSuspended, "Suspended"}, {eDeleted, "Deleted"} };

void setup1(){
    
    // TWELITEのUARTを初期化
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1(){
    int tasks = uxTaskGetNumberOfTasks();
    unsigned long runtime;
    TaskStatus_t *pxTaskStatusArray= (TaskStatus_t *) pvPortMalloc(tasks * sizeof(TaskStatus_t));
    tasks = uxTaskGetSystemState(pxTaskStatusArray, tasks, &runtime);
    SEGGER_RTT_printf(0,"Tasks: %d\n",tasks);
    for(int i=0;i<tasks;i++){
        SEGGER_RTT_printf(0,"    %d: %-20s %-10s %d %lu\n", i, pxTaskStatusArray[i].pcTaskName, eTaskStateName[pxTaskStatusArray[i].eCurrentState], (int)pxTaskStatusArray[i].uxCurrentPriority, pxTaskStatusArray[i].ulRunTimeCounter);
    }
    vPortFree(pxTaskStatusArray);
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}
