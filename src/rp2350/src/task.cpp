#include <cstdarg>

#include <task.hpp>

#include <SEGGER_RTT.h>

bool Task::start()
{
    if (handle_ != nullptr)
    {
        return false;
    }

    return xTaskCreate(task_entry, config_.name, config_.stack_size, this, config_.priority, &handle_) == pdPASS;
}

void Task::task_entry(void *arg)
{
    auto *self = static_cast<Task *>(arg);
    self->run();

    self->handle_ = nullptr;
    vTaskDelete(nullptr);
}

void Task::log_debug(const char *format_string, ...) const
{
    SEGGER_RTT_printf(0, "[%sDEBUG%s %s] : ", RTT_CTRL_TEXT_BLUE, RTT_CTRL_RESET, config_.name);

    va_list ParamList;
    va_start(ParamList, format_string);
    SEGGER_RTT_vprintf(0, format_string, &ParamList);
    va_end(ParamList);
}


void Task::log_info(const char *format_string, ...) const
{
    SEGGER_RTT_printf(0, "[%sINFO%s %s] : ", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET, config_.name);

    va_list ParamList;
    va_start(ParamList, format_string);
    SEGGER_RTT_vprintf(0, format_string, &ParamList);
    va_end(ParamList);
}


void Task::log_warn(const char *format_string, ...) const
{
    SEGGER_RTT_printf(0, "[%sWARN%s %s] : ", RTT_CTRL_TEXT_YELLOW, RTT_CTRL_RESET, config_.name);

    va_list ParamList;
    va_start(ParamList, format_string);
    SEGGER_RTT_vprintf(0, format_string, &ParamList);
    va_end(ParamList);
}


void Task::log_error(const char *format_string, ...) const
{
    SEGGER_RTT_printf(0, "[%sERROR%s %s] : ", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET, config_.name);

    va_list ParamList;
    va_start(ParamList, format_string);
    SEGGER_RTT_vprintf(0, format_string, &ParamList);
    va_end(ParamList);
}

void Task::log_fatal(const char *format_string, ...) const
{
    SEGGER_RTT_printf(0, "[%sFATAL%s %s] : ", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET, config_.name);

    va_list ParamList;
    va_start(ParamList, format_string);
    SEGGER_RTT_vprintf(0, format_string, &ParamList);
    va_end(ParamList);
}