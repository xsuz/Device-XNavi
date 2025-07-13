#pragma once

/// @brief madgwick filterによる姿勢推定
namespace madgwick{
    /// @brief IMUによる時間更新
    /// @param gx 角速度x成分
    /// @param gy 角速度y成分
    /// @param gz 角速度z成分
    /// @param ax 加速度x成分
    /// @param ay 加速度y成分
    /// @param az 加速度z成分
    /// @param q 姿勢クォータニオン
    void update_imu(float gx, float gy, float gz, float ax, float ay, float az, float* q);
    
    /// @brief MARGによる時間更新
    /// @param gx 角速度x成分
    /// @param gy 角速度y成分
    /// @param gz 角速度z成分
    /// @param ax 加速度x成分
    /// @param ay 加速度y成分
    /// @param az 加速度z成分
    /// @param mx 地磁気x成分
    /// @param my 地磁気y成分
    /// @param mz 地磁気z成分
    /// @param q 姿勢クォータニオン
    /// @param wb ジャイロのバイアス推定値
    /// @param b 磁北の向きの推定値
    void update_marg(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float* q,float* wb,float* b);
    // void get_quaternion(float* q);
}