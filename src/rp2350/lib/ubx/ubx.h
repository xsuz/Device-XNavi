#pragma once

#include<Arduino.h>

#include <stdint.h>
#include <stddef.h>

#include <u-blox_struct.h>

/// @brief  Module for parsing u-blox UBX protocol messages
/// @details This module provides a class for parsing u-blox UBX protocol messages.
namespace ubx
{
    /// @brief  Class for parsing u-blox UBX protocol messages
    class parser
    {
    public:
        void parse(uint8_t);
        void reset();
        int64_t age() const{
            return millis()-last_commit_time;
        }

        NAV_PVT get_nav_pvt() const
        {
            return nav_pvt_data.nav_pvt;
        }

    private:
        uint8_t buf[1024];
        enum State
        {
            SYNC_CHAR1,
            SYNC_CHAR2,
            CLASS,
            ID,
            LENGTH_LSB,
            LENGTH_MSB,
            PAYLOAD,
            CK_A,
            CK_B
        };
        State state = State::SYNC_CHAR1;
        union
        {
            struct
            {
                uint8_t message_id;
                uint8_t message_class;
            };
            uint16_t u16;
        } msg_type;
        uint16_t length = 0;
        uint16_t idx = 0;
        uint8_t checksum_a = 0;
        uint8_t checksum_b = 0;
        uint8_t payload[1024] = {0};
        int64_t last_commit_time=0;
        union
        {
            NAV_PVT nav_pvt;
            uint8_t bytes[sizeof(NAV_PVT)];
        } nav_pvt_data;
        bool nav_pvt_valid = false;
    };
} // namespace ubx