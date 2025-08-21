#pragma once

#include <stdint.h>

namespace DeviceData
{

    enum SensorType
    {
        GPS = 0x30,      // GPSデータ
        IMU = 0x40,      // IMUデータ
        XNavi = 0xA0,    // XNaviデータ
        Attitude = 0xB0, // 姿勢データ
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
        union
        {
            uint8_t all;
            struct
            {
                uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
                uint8_t diffSoln : 1;  // 1 = differential corrections were applied
                uint8_t psmState : 3;
                uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
                uint8_t carrSoln : 2;     // Carrier phase range solution status:
                                          // 0: no carrier phase range solution
                                          // 1: carrier phase range solution with floating ambiguities
                                          // 2: carrier phase range solution with fixed ambiguities
            } bits;
        } flags;
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
        uint8_t payload[64];
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
        union
        {
            uint8_t all;
            struct
            {
                uint8_t sys_clock_valid : 1; // システムクロックが有効かどうか
                uint8_t usb_connected : 1; // USBが接続されているかどうか
                uint8_t sd_logger_valid :1; // loggingされているかどうか
                uint8_t reserved :5;
            } bits;
        } status;
    };
}