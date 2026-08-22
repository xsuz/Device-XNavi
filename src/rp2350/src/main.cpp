#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <map>

#include <SEGGER_RTT.h>

#include "uSD.h"
#include "gnss.h"
#include "imu.h"
#include "canbus.h"
#include "monitor.h"
#include "clock.h"
#include "config.hpp"

Consumer usd_consumer(100);
Consumer canbus_consumer(10);
uSDTask usd_task(usd_consumer);
IMUTask imu_task;
GNSSTask gnss_task;
MonitorTask monitor_task;
CANBusTask canbus_task(canbus_consumer);

void setup()
{
    // put your setup code here, to run once:
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "\n[%sINFO%s xnavi] : power on\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    imu_task.subscribe(usd_consumer);
    monitor_task.subscribe(usd_consumer);
    gnss_task.subscribe(usd_consumer);

    // imu_task.subscribe(canbus_consumer);
    monitor_task.subscribe(canbus_consumer);
    gnss_task.subscribe(canbus_consumer);

    imu_task.start();
    monitor_task.start();
    gnss_task.start();
    usd_task.start();
    canbus_task.start();
    // vTaskStartScheduler();
}

void loop()
{
    // put your main code here, to run repeatedly:
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    heartbeat.autopilot = MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC;
    heartbeat.base_mode = MAV_MODE::MAV_MODE_PREFLIGHT;
    heartbeat.custom_mode = 0;
    heartbeat.mavlink_version = 3;
    heartbeat.system_status = MAV_STATE::MAV_STATE_ACTIVE;
    heartbeat.type = MAV_TYPE::MAV_TYPE_GENERIC;
    mavlink_msg_heartbeat_encode(config::mavlink::system_id, config::mavlink::component_id, &msg, &heartbeat);
    canbus_consumer.push(msg,sys_clock::get_timestamp());
    vTaskDelay(1000);
}

// std::map<eTaskState, const char *> eTaskStateName { {eReady, "Ready"}, { eRunning, "Running" }, {eBlocked, "Blocked"}, {eSuspended, "Suspended"}, {eDeleted, "Deleted"} };

void setup1()
{

    // TWELITEのUARTを初期化
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1()
{
    // int tasks = uxTaskGetNumberOfTasks();
    // unsigned long runtime;
    // TaskStatus_t *pxTaskStatusArray= (TaskStatus_t *) pvPortMalloc(tasks * sizeof(TaskStatus_t));
    // tasks = uxTaskGetSystemState(pxTaskStatusArray, tasks, &runtime);
    // SEGGER_RTT_printf(0,"Tasks: %d\n",tasks);
    // for(int i=0;i<tasks;i++){
    //     SEGGER_RTT_printf(0,"    %d: %-20s %-10s %d %lu\n", i, pxTaskStatusArray[i].pcTaskName, eTaskStateName[pxTaskStatusArray[i].eCurrentState], (int)pxTaskStatusArray[i].uxCurrentPriority, pxTaskStatusArray[i].ulRunTimeCounter);
    // }
    // vPortFree(pxTaskStatusArray);

    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}
