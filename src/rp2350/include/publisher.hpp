#pragma once

#include<array>
#include"subscriber.hpp"

class Publisher {
public:
    static constexpr size_t MaxSubscribers = 4;

    bool add_subscriber(Subscriber& subscriber)
    {
        if (count_ >= MaxSubscribers) {
            return false;
        }

        _subscribers[count_++] = &subscriber;
        return true;
    }

    void publish(const mavlink_message_t& message,int64_t timestamp,TickType_t timeout=0)
    {
        for (size_t i = 0; i < count_; ++i) {
            _subscribers[i]->push(message,timestamp,timeout);
        }
    }

private:
    std::array<Subscriber*, MaxSubscribers> _subscribers{};
    size_t count_{0};
};