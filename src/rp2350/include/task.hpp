#pragma once

#include<freertos/FreeRTOS.h>
#include<freertos/task.h>

class Task {
public:
    struct Config {
        const char* name;
        uint32_t stack_size;
        UBaseType_t priority;
    };

    explicit Task(Config config)
        : config_(config)
    {}

    bool start();

    TaskHandle_t handle() const
    {
        return handle_;
    }

protected:
    virtual void run() = 0;
    void log_debug(const char* format_string,...) const;
    void log_info(const char* format_string,...) const;
    void log_warn(const char* format_string,...) const;
    void log_error(const char* format_string,...) const;
    void log_fatal(const char* format_string,...) const;

private:
    static void task_entry(void* arg);

    Config config_;
    TaskHandle_t handle_{nullptr};
};