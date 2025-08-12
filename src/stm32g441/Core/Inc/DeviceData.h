#pragma once

#include <stdint.h>

namespace DeviceData
{

    enum SensorType
    {
        XNavi = 0x00000000,
        GPS = 0x60000000,      // GPSデータ
        IMU = 0x40000000,      // IMUデータ
        Attitude = 0xA0000000, // 姿勢データ
    };

    /// @brief GPSのDeviceData
    /// @note 先頭はid
    struct GPSData
    {
        /// @brief 時刻
        uint32_t timestamp;
        int32_t latitude;  // Latitude: deg * 1e-7
        int32_t longitude; // Longitude: deg * 1e-7
        int32_t altitude;  // Height above ellipsoid: mm
        int32_t velN;      // NED north velocity: mm/s
        int32_t velE;      // NED east velocity: mm/s
        int32_t velD;      // NED down velocity: mm/s
        uint32_t hAcc;     // Horizontal accuracy estimate: mm
        uint32_t vAcc;     // Vertical accuracy estimate: mm
        uint16_t pDOP;     // Position DOP * 0.01
        uint8_t fixType;   // Fix type: 0 = no fix, 1 = dead reckoning only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckoning combined
        uint8_t flags;     // Flags: bit 0 = GNSS fix OK,
                           // bit 1 = differential corrections applied, bit 2 = PSM state, bit 3 = heading of vehicle valid,
                           // bit 4 = carrier phase range solution status (0 = no solution, 1 = floating, 2 = fixed)
    };

    /// @brief IMUのDeviceData
    /// @note 先頭はid
    struct IMUData
    {
        /// @brief 時刻 : ms
        uint32_t timestamp;
        /// @brief 加速度 x : m/s^2
        float a_x;
        /// @brief 加速度 y : m/s^2
        float a_y;
        /// @brief 加速度 z : m/s^2
        float a_z;
        /// @brief 角速度 x : rad/s
        float w_x;
        /// @brief 角速度 y : rad/s
        float w_y;
        /// @brief 角速度 z : rad/s
        float w_z;
    };

    /// @brief 姿勢角の計算結果
    struct AttitudeData
    {
        /// @brief 時刻
        uint32_t timestamp;
        /// @brief Quaternion w
        float q0;
        /// \brief Quaternion x
        float q1;
        /// @brief Quaternion y
        float q2;
        /// @brief Quaternion z
        float q3;
    };

    /// @brief CAN busの送受信データ
    struct CANPacket
    {
        /// @brief CANのデバイス識別子
        uint32_t id;
        /// @brief CANのデータのサイズ
        uint32_t size;
        /// @brief CANのデータ
        uint8_t payload[8];
    };

    struct XNaviStatus
    {
        /// @brief 時刻 : msec
        uint32_t timestamp;
        /// @brief 電源電圧 : mV
        uint16_t voltage;
        /// @brief バッテリー残量 : %
        uint8_t percentage;
        /// @brief ログの状態
        uint8_t status;
    };
}