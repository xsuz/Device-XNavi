#pragma once

#include <mavlink/swingby/mavlink.h>
#include "task.hpp"
#include "publisher.hpp"

#include <Arduino.h>

class MonitorTask : public Task
{
public:
    MonitorTask() : Task({
                .name = "Monitor",
                .stack_size = 1024,
                .priority = 1,
            }),
            _publisher()
    {
    }
    void add_subscriber(Subscriber &subscriber)
    {
        _publisher.add_subscriber(subscriber);
    }

protected:
    void run() override;

private:
    Publisher _publisher;
};