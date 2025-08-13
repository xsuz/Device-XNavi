#pragma once

#include <stdint.h>

namespace
{
    constexpr uint16_t x = 1;
    const uint8_t is_little = ((uint8_t *)&x)[0];
}

namespace u16
{
    template <typename T>
    T from_bytes(const uint8_t *bytes, size_t offset = 0, uint8_t endian = is_little)
    {
        union
        {
            T t;
            uint8_t raw[4];
        } u;
        if (is_little & endian)
        {
            u.raw[0] = bytes[offset];
            u.raw[1] = bytes[offset + 1];
        }
        else
        {
            u.raw[0] = bytes[offset + 0];
            u.raw[1] = bytes[offset + 1];
        }
        return u.t;
    }

    /// @brief 32bitの変数をバイト列に変換
    /// @param bytes 書き込み先
    /// @param offset オフセット
    /// @param value 変換する値
    /// @param endian 0(BigEndian), 1(Little Endian)
    template <typename T>
    void to_bytes(uint8_t *bytes, size_t offset, T value, uint8_t endian = is_little)
    {
        union
        {
            T t;
            uint8_t raw[4];
        } u;
        u.t = value;
        if (is_little != endian)
        {
            bytes[offset + 1] = u.raw[0];
            bytes[offset + 0] = u.raw[1];
        }
        else
        {
            bytes[offset + 0] = u.raw[0];
            bytes[offset + 1] = u.raw[1];
        }
    }

    template <typename T>
    void swap(T *val)
    {
        uint8_t *u8 = (uint8_t *)val;
        uint8_t tmp;
        // swap the bytes into a temporary buffer
        tmp = u8[0];
        u8[0] = u8[1];
        u8[1] = tmp;
    }

    template <typename T>
    T to_be(const T t)
    {
        T val = t;
        if (is_little)
        {
            swap(&val);
        }
        return val;
    }

    template <typename T>
    T to_le(const T t)
    {
        T val = t;
        if (!is_little)
        {
            swap(&val);
        }
        return val;
    }

    template <typename T>
    void to_be(T *t)
    {
        if (is_little)
        {
            swap(t);
        }
    }

    template <typename T>
    void to_le(T *t)
    {
        if (!is_little)
        {
            swap(t);
        }
    }
}

namespace u32
{
    template <typename T>
    T from_bytes(const uint8_t *bytes, size_t offset = 0, uint8_t endian = is_little)
    {
        union
        {
            T t;
            uint8_t raw[4];
        } u;
        if (is_little & endian)
        {
            u.raw[0] = bytes[offset];
            u.raw[1] = bytes[offset + 1];
            u.raw[2] = bytes[offset + 2];
            u.raw[3] = bytes[offset + 3];
        }
        else
        {
            u.raw[0] = bytes[offset + 3];
            u.raw[1] = bytes[offset + 2];
            u.raw[2] = bytes[offset + 1];
            u.raw[3] = bytes[offset];
        }
        return u.t;
    }

    /// @brief 32bitの変数をバイト列に変換
    /// @param bytes 書き込み先
    /// @param offset オフセット
    /// @param value 変換する値
    /// @param endian 0(BigEndian), 1(Little Endian)
    template <typename T>
    void to_bytes(uint8_t *bytes, size_t offset, T value, uint8_t endian = is_little)
    {
        union
        {
            T t;
            uint8_t raw[4];
        } u;
        u.t = value;
        if (is_little != endian)
        {
            bytes[offset + 3] = u.raw[0];
            bytes[offset + 2] = u.raw[1];
            bytes[offset + 1] = u.raw[2];
            bytes[offset + 0] = u.raw[3];
        }
        else
        {
            bytes[offset + 0] = u.raw[0];
            bytes[offset + 1] = u.raw[1];
            bytes[offset + 2] = u.raw[2];
            bytes[offset + 3] = u.raw[3];
        }
    }

    template <typename T>
    void swap(T *val)
    {
        uint8_t *u8 = (uint8_t *)val;
        uint8_t tmp;
        // swap the bytes into a temporary buffer
        tmp = u8[0];
        u8[0] = u8[3];
        u8[3] = tmp;
        tmp = u8[1];
        u8[1] = u8[2];
        u8[2] = tmp;
    }

    template <typename T>
    T to_be(const T t)
    {
        T val = t;
        if (is_little)
        {
            swap(&val);
        }
        return val;
    }

    template <typename T>
    T to_le(const T t)
    {
        T val = t;
        if (!is_little)
        {
            swap(&val);
        }
        return val;
    }

    template <typename T>
    void to_be(T *t)
    {
        if (is_little)
        {
            swap(t);
        }
    }

    template <typename T>
    void to_le(T *t)
    {
        if (!is_little)
        {
            swap(t);
        }
    }
};

namespace u64
{
    template <typename T>
    T from_be_bytes(const uint8_t *bytes, size_t offset = 0)
    {
        union
        {
            T t;
            uint8_t raw[4];
        } u;
        if (is_little)
        {
            u.raw[0] = bytes[offset + 7];
            u.raw[1] = bytes[offset + 6];
            u.raw[2] = bytes[offset + 5];
            u.raw[3] = bytes[offset + 4];
            u.raw[4] = bytes[offset + 3];
            u.raw[5] = bytes[offset + 2];
            u.raw[6] = bytes[offset + 1];
            u.raw[7] = bytes[offset];
        }
        else
        {
            u.raw[0] = bytes[offset];
            u.raw[1] = bytes[offset + 1];
            u.raw[2] = bytes[offset + 2];
            u.raw[3] = bytes[offset + 3];
            u.raw[4] = bytes[offset + 4];
            u.raw[5] = bytes[offset + 5];
            u.raw[6] = bytes[offset + 6];
            u.raw[7] = bytes[offset + 7];
        }
        return u.t;
    }

    template <typename T>
    T from_le_bytes(const uint8_t *bytes, size_t offset = 0)
    {
        union
        {
            T t;
            uint8_t raw[4];
        } u;
        if (is_little)
        {
            u.raw[0] = bytes[offset];
            u.raw[1] = bytes[offset + 1];
            u.raw[2] = bytes[offset + 2];
            u.raw[3] = bytes[offset + 3];
            u.raw[4] = bytes[offset + 4];
            u.raw[5] = bytes[offset + 5];
            u.raw[6] = bytes[offset + 6];
            u.raw[7] = bytes[offset + 7];
        }
        else
        {
            u.raw[0] = bytes[offset + 7];
            u.raw[1] = bytes[offset + 6];
            u.raw[2] = bytes[offset + 5];
            u.raw[3] = bytes[offset + 4];
            u.raw[4] = bytes[offset + 3];
            u.raw[5] = bytes[offset + 2];
            u.raw[6] = bytes[offset + 1];
            u.raw[7] = bytes[offset];
        }
        return u.t;
    }

    template <typename T>
    void swap(T *val)
    {
        uint8_t *u8 = (uint8_t *)val;
        uint8_t tmp;
        // swap the bytes into a temporary buffer
        tmp = u8[0];
        u8[0] = u8[7];
        u8[7] = tmp;

        tmp = u8[1];
        u8[1] = u8[6];
        u8[6] = tmp;

        tmp = u8[2];
        u8[2] = u8[5];
        u8[5] = tmp;

        tmp = u8[3];
        u8[3] = u8[4];
        u8[4] = tmp;
    }

    template <typename T>
    T to_be(const T t)
    {
        T val = t;
        if (is_little)
        {
            swap(&val);
        }
        return val;
    }

    template <typename T>
    T to_le(const T t)
    {
        T val = t;
        if (!is_little)
        {
            swap(&val);
        }
        return val;
    }
    
    template <typename T>
    void to_be(T *t)
    {
        if (is_little)
        {
            swap(t);
        }
    }

    template <typename T>
    void to_le(T *t)
    {
        if (!is_little)
        {
            swap(t);
        }
    }
};