#pragma once

#include <stdint.h>
#include <stddef.h>

#include "task.hpp"
#include "subscriber.hpp"

class uSDTask : public Task
{
public:
    uSDTask(Subscriber subscriber)
        : Task({
              .name = "uSD",
              .stack_size = 1024,
              .priority = 1,
          }),
          _subscriber{subscriber}
    {
    }

protected:
    void run() override;

private:
    void inline get_filename(char *filename);

    void inline write_pkt(const mavlink_message_t &msg, int64_t timestamp);
    void inline write_ring_buffer(uint8_t);
    void inline write_record(const uint8_t *buffer, size_t size);
    size_t inline available_ring_buffer_size_unsafe();

    Subscriber _subscriber;

    static constexpr int LED = 10; // Use built-in LED for status indication

    static constexpr size_t BLOCK_SIZE = 4096;
    static constexpr size_t BLOCK_COUNT = 4;

    uint8_t buffer[BLOCK_COUNT][BLOCK_SIZE];

    size_t write_index = 0;
    size_t write_offset = 0;

    size_t read_index = 0;
    size_t ready_blocks = 0;
};