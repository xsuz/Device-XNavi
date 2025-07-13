#include "madgwick.hpp"
#include "cmath"

namespace madgwick
{
    constexpr float deltat = 0.020f;
    constexpr float gyroMeasError = M_PI * (5.0f / 180.0f);
    constexpr float beta = 0.866f * gyroMeasError;
    constexpr float gyroMeasDrift = M_PI * (0.2f / 180.0f);
    constexpr float zeta = 0.866f * gyroMeasDrift;

    void update_imu(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float *q)
    {
        // Local system variables
        float norm;
        // vector norm
        float SEqDot_omega_0, SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3; // quaternion derrivative from gyroscopes elements
        float f_0, f_1, f_2;
        // objective function elements
        float J_00or13, J_01or12, J_02or11, J_03or10, J_21, J_22;
        float SEqHatDot_0, SEqHatDot_1, SEqHatDot_2, SEqHatDot_3;

        // Normalise the accelerometer measurement
        norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
        a_x /= norm;
        a_y /= norm;
        a_z /= norm;

        // Compute the objective function and Jacobian
        f_0 = 2.0f * (q[1] * q[3] - q[0] * q[2]) - a_x;
        f_1 = 2.0f * (q[0] * q[1] + q[2] * q[3]) - a_y;
        f_2 = 1.0f - 2.0f * (q[1] * q[1] - q[2] * q[2]) - a_z;
        J_00or13 = 2.0f * q[2]; // J_11 negated in matrix multiplication
        J_01or12 = 2.0f * q[3];
        J_02or11 = 2.0f * q[0]; // J_12 negated in matrix multiplication
        J_03or10 = 2.0f * q[1];
        J_21 = 2.0f * J_03or10; // negated in matrix multiplication
        J_22 = 2.0f * J_00or13; // negated in matrix multiplication

        // Compute the gradient (matrix multiplication)
        SEqHatDot_0 = J_03or10 * f_1 - J_00or13 * f_0;
        SEqHatDot_1 = J_01or12 * f_0 + J_02or11 * f_1 - J_21 * f_2;
        SEqHatDot_2 = J_01or12 * f_1 - J_22 * f_2 - J_02or11 * f_0;
        SEqHatDot_3 = J_03or10 * f_0 + J_00or13 * f_1;

        // Normalise the gradient
        norm = sqrt(SEqHatDot_0 * SEqHatDot_0 + SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3);
        SEqHatDot_0 /= norm;
        SEqHatDot_1 /= norm;
        SEqHatDot_2 /= norm;
        SEqHatDot_3 /= norm;

        // Compute the quaternion derrivative measured by gyroscopes
        SEqDot_omega_0 = 0.5f * (-q[1] * w_x - q[2] * w_y - q[3] * w_z);
        SEqDot_omega_1 = 0.5f * (q[0] * w_x + q[2] * w_z - q[3] * w_y);
        SEqDot_omega_2 = 0.5f * (q[0] * w_y - q[1] * w_z + q[3] * w_x);
        SEqDot_omega_3 = 0.5f * (q[0] * w_z + q[1] * w_y - q[2] * w_x);

        // Compute then integrate the estimated quaternion derrivative
        q[0] += (SEqDot_omega_0 - (beta * SEqHatDot_0)) * deltat;
        q[1] += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
        q[2] += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
        q[3] += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;

        // Normalise quaternion
        norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }

    void update_marg(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float *q, float *wb, float *b)
    {
        // local system variables
        float norm;                                                           // vector norm
        float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
        float f_1, f_2, f_3, f_4, f_5, f_6;                                   // objective function elements
        float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,             // objective function Jacobian elements
            J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;
        float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
        float w_err_x, w_err_y, w_err_z;                          // estimated direction of the gyroscope error (angular)
        float h_x, h_y, h_z;                                      // computed flux in the earth frame

        // axulirary variables to avoid reapeated calcualtions
        float halfSEq_1 = 0.5f * q[0];
        float halfSEq_2 = 0.5f * q[1];
        float halfSEq_3 = 0.5f * q[2];
        float halfSEq_4 = 0.5f * q[3];
        float twoSEq_1 = 2.0f * q[0];
        float twoSEq_2 = 2.0f * q[1];
        float twoSEq_3 = 2.0f * q[2];
        float twoSEq_4 = 2.0f * q[3];
        float twob_x = 2.0f * b[0];
        float twob_z = 2.0f * b[1];
        float twob_xSEq_1 = 2.0f * b[0] * q[0];
        float twob_xSEq_2 = 2.0f * b[0] * q[1];
        float twob_xSEq_3 = 2.0f * b[0] * q[2];
        float twob_xSEq_4 = 2.0f * b[0] * q[3];
        float twob_zSEq_1 = 2.0f * b[1] * q[0];
        float twob_zSEq_2 = 2.0f * b[1] * q[1];
        float twob_zSEq_3 = 2.0f * b[1] * q[2];
        float twob_zSEq_4 = 2.0f * b[1] * q[3];
        float SEq_1SEq_2;
        float SEq_1SEq_3 = q[0] * q[2];
        float SEq_1SEq_4;
        float SEq_2SEq_3;
        float SEq_2SEq_4 = q[1] * q[3];
        float SEq_3SEq_4;
        float twom_x = 2.0f * m_x;
        float twom_y = 2.0f * m_y;
        float twom_z = 2.0f * m_z;

        // normalise the accelerometer measurement
        norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
        a_x /= norm;
        a_y /= norm;
        a_z /= norm;

        // normalise the magnetometer measurement
        norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
        m_x /= norm;
        m_y /= norm;
        m_z /= norm;

        // compute the objective function and Jacobian
        f_1 = twoSEq_2 * q[3] - twoSEq_1 * q[2] - a_x;
        f_2 = twoSEq_1 * q[1] + twoSEq_3 * q[3] - a_y;
        f_3 = 1.0f - twoSEq_2 * q[1] - twoSEq_3 * q[2] - a_z;
        f_4 = twob_x * (0.5f - q[2] * q[2] - q[3] * q[3]) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
        f_5 = twob_x * (q[1] * q[2] - q[0] * q[3]) + twob_z * (q[0] * q[1] + q[2] * q[3]) - m_y;
        f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - q[1] * q[1] - q[2] * q[2]) - m_z;
        J_11or24 = twoSEq_3;
        J_12or23 = 2.0f * q[3];
        J_13or22 = twoSEq_1;
        J_14or21 = twoSEq_2;
        J_32 = 2.0f * J_14or21;
        J_33 = 2.0f * J_11or24;
        J_41 = twob_zSEq_3;
        J_42 = twob_zSEq_4;
        J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
        J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
        J_51 = twob_xSEq_4 - twob_zSEq_2;
        J_52 = twob_xSEq_3 + twob_zSEq_1;
        J_53 = twob_xSEq_2 + twob_zSEq_4;
        J_54 = twob_xSEq_1 - twob_zSEq_3;
        J_61 = twob_xSEq_3;
        J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
        J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
        J_64 = twob_xSEq_2;

        // compute the gradient (matrix multiplication)
        SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
        SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
        SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
        SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

        // normalise the gradient to estimate direction of the gyroscope error
        norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
        SEqHatDot_1 = SEqHatDot_1 / norm;
        SEqHatDot_2 = SEqHatDot_2 / norm;
        SEqHatDot_3 = SEqHatDot_3 / norm;
        SEqHatDot_4 = SEqHatDot_4 / norm;

        // compute angular estimated direction of the gyroscope error
        w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
        w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
        w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

        // compute and remove the gyroscope baises
        wb[0] += w_err_x * deltat * zeta;
        wb[1] += w_err_y * deltat * zeta;
        wb[2] += w_err_z * deltat * zeta;
        w_x -= wb[0];
        w_y -= wb[1];
        w_z -= wb[2];

        // compute the quaternion rate measured by gyroscopes
        SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
        SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
        SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
        SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

        // compute then integrate the estimated quaternion rate
        q[0] += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
        q[1] += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
        q[2] += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
        q[3] += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

        // normalise quaternion
        norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;

        // compute flux in the earth frame
        SEq_1SEq_2 = q[0] * q[1];
        SEq_1SEq_3 = q[0] * q[2];
        SEq_1SEq_4 = q[0] * q[3];
        SEq_3SEq_4 = q[2] * q[3];
        SEq_2SEq_3 = q[1] * q[2];
        SEq_2SEq_4 = q[1] * q[3];

        // recompute axulirary variables
        h_x = twom_x * (0.5f - q[2] * q[2] - q[3] * q[3]) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
        h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - q[1] * q[1] - q[3] * q[3]) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
        h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - q[1] * q[1] - q[2] * q[2]);

        // normalise the flux vector to have only components in the x and z
        b[0] = sqrt((h_x * h_x) + (h_y * h_y));
        b[1] = h_z;
    }
}