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

    bool start()
    {
        if (handle_ != nullptr) {
            return false;
        }

        return xTaskCreate(
            task_entry,
            config_.name,
            config_.stack_size,
            this,
            config_.priority,
            &handle_
        ) == pdPASS;
    }

    TaskHandle_t handle() const
    {
        return handle_;
    }

protected:
    virtual void run() = 0;

private:
    static void task_entry(void* arg)
    {
        auto* self = static_cast<Task*>(arg);
        self->run();

        self->handle_ = nullptr;
        vTaskDelete(nullptr);
    }

    Config config_;
    TaskHandle_t handle_{nullptr};
};