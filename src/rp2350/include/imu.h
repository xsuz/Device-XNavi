#pragma once

#include <timers.h>
#include <queue.h>

#include <ASM330LHHSensor.h>
#include <SPI.h>

#include "task.hpp"
#include "publisher.hpp"

class IMUTask : public Task
{
public:
    IMUTask() : Task({.name = "IMU", .stack_size = 512, .priority = 3}), _publisher() {}
    void add_subscriber(Subscriber &subscriber)
    {
        _publisher.add_subscriber(subscriber);
    }

protected:
    void run() override;

private:
    Publisher _publisher;
};