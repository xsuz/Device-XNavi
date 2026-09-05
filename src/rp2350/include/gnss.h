#pragma once
#include <Arduino.h>
#include <timers.h>
#include <queue.h>

#include <ubx.h>
#include <mavlink/swingby/mavlink.h>

#include "task.hpp"
#include "publisher.hpp"

class GNSSTask :public Task
{
public:
    GNSSTask() : Task({.name = "GNSS", .stack_size = 512, .priority = 2}), _publisher()
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
    void setup_gnss();
    static void callback_pps(uint gpio, uint32_t emask);
    static void callback_pvt(ubx::NAV_PVT pvt, void *);
    static void callback_reset(void *);
    /// @brief u-blox UBXパーサー
    ubx::parser ubx_parser;
    const int LED = 11;
};