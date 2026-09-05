#pragma once

#include <FreeRTOS.h>
#include <queue.h>
#include <mavlink/swingby/mavlink.h>
#include<string.h>

class Subscriber
{
public:
    explicit Subscriber(size_t size){
        queue_=xQueueCreate(size,sizeof(mavlink_message_t)+8);
    }

    bool push(const mavlink_message_t &message,const int64_t timestamp, TickType_t timeout = 10)
    {
        uint8_t buffer[sizeof(mavlink_message_t)+8];
        memcpy(buffer,&timestamp,8);
        memcpy(buffer+8,&message,sizeof(mavlink_message_t));
        return (xQueueSend(queue_, buffer, timeout) == pdTRUE);
    }

    bool receive(mavlink_message_t &message, int64_t &timestamp, TickType_t timeout = portMAX_DELAY){
        uint8_t buffer[sizeof(mavlink_message_t)+8];
        if(xQueueReceive(queue_,buffer,timeout)!=pdTRUE){
            return false;
        }
        memcpy(&timestamp,buffer,8);
        memcpy(&message,buffer+8,sizeof(mavlink_message_t));
        return true;
    }

    size_t dropped() const{
        return dropped_;
    }

private:
    QueueHandle_t queue_;
    size_t dropped_{0};
};