#pragma once

#include<array>
#include"consumer.hpp"

class Publisher {
public:
    static constexpr size_t MaxConsumers = 4;

    bool subscribe(Consumer& consumer)
    {
        if (count_ >= MaxConsumers) {
            return false;
        }

        consumers_[count_++] = &consumer;
        return true;
    }

    void publish(const mavlink_message_t& message,int64_t timestamp,TickType_t timeout=0)
    {
        for (size_t i = 0; i < count_; ++i) {
            consumers_[i]->push(message,timestamp,timeout);
        }
    }

private:
    std::array<Consumer*, MaxConsumers> consumers_{};
    size_t count_{0};
};