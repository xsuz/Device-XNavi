#pragma once

#include "task.hpp"
#include "subscriber.hpp"

#include <Arduino.h>

class CANBusTask : public Task
{
public:
    CANBusTask(Subscriber consumer)
        : Task({
              .name = "CANBus",
              .stack_size = 512,
              .priority = 1,
          }),
          _consumer{consumer}
    {
    }

protected:
    void run() override;

private:
    void onPacketReceived(const mavlink_message_t &msg);

    Subscriber _consumer;
};