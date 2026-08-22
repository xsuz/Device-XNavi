#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include "gnss.h"
#include "clock.h"
#include "config.hpp"

// #include <ctime>

#include <ubx.h>
#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

namespace
{
    /// @brief PPSによる割込みが発生した時刻
    static volatile uint32_t tick_last_pps;
};

void GNSSTask::run()
{
    int64_t utc;
    ubx::NAV_PVT pvt;
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps_raw;

    log_info("GNSS task started.\n");

    setup_gnss();

    ubx_parser.set_callback_NAV_PVT(GNSSTask::callback_pvt, this);
    ubx_parser.set_callback_reset(GNSSTask::callback_reset, this);

    // PPSによる割り込み設定
    tick_last_pps = 0;
    gpio_init(2);
    gpio_set_dir(2, GPIO_IN);
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &GNSSTask::callback_pps);

    Serial.begin(115200);
    while (1)
    {
        if (Serial)
        {
            while (Serial.available() > 0)
            {
                uint8_t c = Serial.read();
                Serial1.write(c);
            }
        }
        while (Serial1.available() > 0)
        {
            uint8_t c = Serial1.read();
            ubx_parser.parse(c);
            if (Serial)
            {
                Serial.write(c);
            }
        }
        vTaskDelay(1);
    }
}

void GNSSTask::setup_gnss()
{
    // UART0を初期化
    Serial1.setFIFOSize(2048);
    Serial1.begin(9600);
    delay(1000); // GPSレシーバの起動を待機
    const uint8_t UBX_HEADER1 = 0xb5, UBX_HEADER2 = 0x62, UBX_CFG = 0x06;

    // [0xB5 0x62] : UBX header, 0x06 : class=UBX-CFG, 0x08 : message ID=CFG-RATE, [0x06 0x00] : payload length, [0x64 0x00] : measRate=100ms, [0x01 0x00] : navRate=1, [0x01 0x00] : timeRef=1, [0x7A 0x12] : checksum
    uint8_t cmd0[] = {UBX_HEADER1, UBX_HEADER2, 0x06, 0x08, 6, 0, 100, 0, 1, 0, 1, 0, 122, 18};
    Serial1.write(cmd0, sizeof(cmd0)); // RATEを10Hzに設定
    delay(100);
    // NAV-PVT出力を有効化
    uint8_t cmd1[] = {UBX_HEADER1, UBX_HEADER2, 0x06, 1, 8, 0, 1, 7, 0, 1, 0, 0, 0, 0, 24, 225};
    Serial1.write(cmd1, sizeof(cmd1));
    delay(100);
    // UBX出力を有効化
    uint8_t cmd2[] = {UBX_HEADER1, UBX_HEADER2, 0x06, 0, 20, 0, 1, 0, 0, 0, 208, 8, 0, 0, 0, 194, 1, 0, 3, 0, 1, 0, 0, 0, 0, 0, 186, 82};
    Serial1.write(cmd2, sizeof(cmd2));
    delay(100);
    // PPSの基準を
    Serial1.println("$PUBX,41,1,0007,0003,115200,0*18"); // baudrateを115200に設定
    delay(1000);
    Serial1.flush();       // 無効なデータを破棄
    Serial1.begin(115200); // baudrate 115200で再度UART0を初期化
}

void GNSSTask::callback_pvt(ubx::NAV_PVT pvt, void *context)
{
    auto *self = static_cast<GNSSTask *>(context);
    if (pvt.valid.bits.validDate && pvt.valid.bits.validTime && tick_last_pps > 0)
    {
        sys_clock::set_timestamp_offset(tick_last_pps, pvt.year, pvt.month, pvt.day, pvt.hour, pvt.min, pvt.sec);
    }
    int64_t utc = sys_clock::get_timestamp();
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps_raw;

    gps_raw.time_usec = utc;
    gps_raw.lat = pvt.lat;
    gps_raw.lon = pvt.lon;
    gps_raw.alt = pvt.hMSL;
    gps_raw.eph = pvt.hAcc;
    gps_raw.epv = pvt.vAcc;
    gps_raw.vel = pvt.gSpeed * 10;
    gps_raw.cog = pvt.headMot;
    gps_raw.fix_type = pvt.fixType;
    gps_raw.satellites_visible = pvt.numSV;
    gps_raw.alt_ellipsoid = pvt.height;
    gps_raw.h_acc = pvt.hAcc;
    gps_raw.v_acc = pvt.vAcc;
    gps_raw.vel_acc = pvt.sAcc;
    gps_raw.hdg_acc = pvt.headAcc;
    gps_raw.yaw = 0;

    self->log_info("latitude: %d, longitude: %d, altitude: %d, velN: %d, velE: %d, velD: %d, hAcc: %u, vAcc: %u, fixType: %u, pDOP: %u\n",
                   pvt.lat, pvt.lon, pvt.height, pvt.velN, pvt.velE, pvt.velD, pvt.hAcc, pvt.vAcc, pvt.fixType, pvt.pDOP);

    mavlink_msg_gps_raw_int_encode(config::mavlink::system_id, config::mavlink::component_id, &msg, &gps_raw);
    self->_publisher.publish(msg, utc);
}

void GNSSTask::callback_reset(void * context)
{
    auto *self = static_cast<GNSSTask *>(context);
    Serial1.begin(115200);
    Serial1.flush();
    self->log_info("reset UBX parser.\n");
}

void GNSSTask::callback_pps(uint gpio, uint32_t emask)
{
    gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), false);
    tick_last_pps = millis();
    gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), true);
}