#pragma once

#include <Arduino.h>

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
        inline int64_t get_last_commit_tick() const
        {
            return last_commit_time;
        }

        inline void get_nav_pvt(NAV_PVT *pvt) const{
            uint8_t *bytes=(uint8_t*)pvt;
            for(size_t i=0;i<sizeof(NAV_PVT);i++){
                bytes[i]=nav_pvt_data.bytes[i];
            }
        }

        void (*callbackPVT)(NAV_PVT) = nullptr;

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
        int64_t last_commit_time = -1;
        union
        {
            NAV_PVT nav_pvt;
            uint8_t bytes[sizeof(NAV_PVT)];
        } nav_pvt_data;
        bool nav_pvt_valid = false;
    };
} // namespace ubx