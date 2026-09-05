#pragma once

#include "task.hpp"
#include "subscriber.hpp"

#include <Arduino.h>

class CANBusTask : public Task
{
public:
    CANBusTask(Subscriber subscriber)
        : Task({
              .name = "CANBus",
              .stack_size = 512,
              .priority = 1,
          }),
          _subscriber{subscriber}
    {
    }

protected:
    void run() override;

private:
    void onPacketReceived(const mavlink_message_t &msg);

    Subscriber _subscriber;
};