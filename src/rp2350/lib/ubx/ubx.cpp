#include "ubx.h"

void ubx::parser::parse(uint8_t c)
{
    switch (state)
    {
    case State::SYNC_CHAR1:
    {
        if (c == 0xb5)
        {
            state = State::SYNC_CHAR2;
        }
    }
    break;
    case State::SYNC_CHAR2:
    {
        if (c == 0x62)
        {
            checksum_a = 0;
            checksum_b = 0;
            state = State::CLASS;
        }
    }
    break;
    case State::CLASS:
    {
        msg_type.message_class = c;
        checksum_a += c;
        checksum_b += checksum_a;
        state = State::ID;
    }
    break;
    case State::ID:
    {
        msg_type.message_id = c;
        checksum_a += c;
        checksum_b += checksum_a;
        state = State::LENGTH_LSB;
    }
    break;
    case State::LENGTH_LSB:
    {
        length = c;
        checksum_a += c;
        checksum_b += checksum_a;
        state = State::LENGTH_MSB;
    }
    break;
    case State::LENGTH_MSB:
    {
        length |= (c << 8);
        checksum_a += c;
        checksum_b += checksum_a;
        state = State::PAYLOAD;
        idx = 0;
    }
    break;
    case State::PAYLOAD:
    {
        buf[idx] = c;
        checksum_a = checksum_a + buf[idx];
        checksum_b = checksum_b + checksum_a;
        idx++;
        if (idx == length)
        {
            state = State::CK_A;
        }
    }
    break;
    case State::CK_A:
    {
        if (checksum_a == c)
        {
            state = State::CK_B;
        }
        else
        {
            state = State::SYNC_CHAR1;
        }
    }
    break;
    case State::CK_B:
    {
        state = State::SYNC_CHAR1;
        if (checksum_b == c)
        {
            if (msg_type.u16 == 0x0107)
            {
                // Serial.println("msg_type=PVT");
                for (int i = 0; i < sizeof(nav_pvt_data.bytes); i++)
                {
                    nav_pvt_data.bytes[i] = buf[i];
                }
                if(callbackPVT!=nullptr){
                    callbackPVT(nav_pvt_data.nav_pvt);
                }
                last_commit_time = millis();
            }
            else
            {
                // Serial.printf("msg_type=%04x", msg_type.u16);
            }
        }
        else
        {
            // Serial.println("failed..@ckb");
        }
    }
    break;
    default:
    {
        // Serial.println("error @ parse");
    };
    }
}

void ubx::parser::reset()
{
    state = State::SYNC_CHAR1;
    idx = 0;
    length = 0;
    msg_type.u16 = 0;
    checksum_a = 0;
    checksum_b = 0;
    for (int i = 0; i < sizeof(buf); i++)
    {
        buf[i] = 0;
    }
}