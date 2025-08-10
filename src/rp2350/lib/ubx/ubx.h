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

        inline void get_nav_pvt(NAV_PVT *pvt) const{
            uint8_t *bytes=(uint8_t*)pvt;
            for(size_t i=0;i<sizeof(NAV_PVT);i++){
                bytes[i]=nav_pvt_data.bytes[i];
            }
        }

        void (*callbackPVT)(NAV_PVT) = nullptr;

    private:
        uint8_t buf[512];
        enum State
        {
            SYNC_CHAR1,// Synchronization character (0xB5)
            SYNC_CHAR2,// Synchronization characters (0xB5, 0x62)
            CLASS,// Message class
            ID,// Message ID
            LENGTH_LSB,// Length of payload (LSB)
            LENGTH_MSB,// Length of payload (MSB)
            PAYLOAD,// Payload
            CK_A,// Checksum A
            CK_B// Checksum B
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
        uint8_t payload[512] = {0};
        union
        {
            NAV_PVT nav_pvt;
            uint8_t bytes[sizeof(NAV_PVT)];
        } nav_pvt_data;
    };
} // namespace ubx