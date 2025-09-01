#pragma once

namespace readme
{
    constexpr char README[]=R"EOF(
# XNavi Reference Manual

## 1. About XNavi

XNavi is a logging device designed to capture and store data from various sensors and systems. It generates log files in the format "log_YYYYMMDDhhmmss.bin", which can be used for further analysis.
To learn more about XNavi, visit the [XNavi GitHub repository](https://github.com/xsuz/Device-XNavi)

## 2. Log File Format

Data acquired by XNavi is encoded in COBS and stored on a uSD card.
Each packet in the log file has a GPS timestamp(int64) at the beginning, followed by the data payload.

### 2.1. GNSS

|offset   |content     |format |endian       |unit    |
|:-------:|:----------:|:-----:|:-----------:|:------:|
| [ 0- 3] |sensor type |uint32 |Little Endian|      |
| [ 4- 7] |timestamp   |uint32 |Little Endian|msec    |
| [ 8-11] |latitude    |int32  |Little Endian|1e-7 deg|
| [12-15] |longitude   |int32  |Little Endian|1e-7 deg|
| [16-19] |altitude    |int32  |Little Endian|mm      |
| [20-23] |velN        |int32  |Little Endian|mm/s    |
| [24-27] |velE        |int32  |Little Endian|mm/s    |
| [28-31] |velD        |int32  |Little Endian|mm/s    |
| [32-36] |hAcc        |int32  |Little Endian|mm      |
| [36-40] |vAcc        |int32  |Little Endian|mm      |
| [40-41] |pDOP        |uint16 |Little Endian|1e-2    |
| 42      |fix type    |uint8  |             |        |
| 43      |flags       |uint8  |             |        |

### 2.2. IMU

|offset   |content     |format |endian       |unit  |
|:-------:|:----------:|:-----:|:-----------:|:----:|
| [ 0- 3] |sensor type |uint32 |Little Endian|      |
| [ 4- 7] |timestamp   |uint32 |Little Endian|msec  |
| [ 8-11] |a_x         |float32|Little Endian|m/s^2 |
| [12-15] |a_y         |float32|Little Endian|m/s^2 |
| [16-19] |a_z         |float32|Little Endian|m/s^2 |
| [20-23] |w_x         |float32|Little Endian|rad/s |
| [24-27] |w_y         |float32|Little Endian|rad/s |
| [28-31] |w_z         |float32|Little Endian|rad/s |

### 2.3. CAN bus

|offset   |content     |format |endian       |unit  |
|:-------:|:----------:|:-----:|:-----------:|:----:|
| [ 0- 3] |ID          |uint32 |Little Endian|      |
| [ 4- 7] |timestamp   |uint32 |Little Endian|msec  |
| [ 8-11] |length      |uint32 |Little Endian|      |
| [12-16] |payload     |uint8  |             |      |


)EOF";
} // namespace readme
