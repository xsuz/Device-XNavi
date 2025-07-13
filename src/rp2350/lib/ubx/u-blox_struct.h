/*
  This is a library written for the u-blox ZED-F9P and NEO-M8P-2
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/15005
  https://www.sparkfun.com/products/15733
  https://www.sparkfun.com/products/15193
  https://www.sparkfun.com/products/15210

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020

  This library handles configuring and handling the responses
  from a u-blox GPS module. Works with most modules from u-blox including
  the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

  https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.13

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2016 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __u_blox_structs_h__
#define __u_blox_structs_h__

#include <stdint.h>

namespace ubx
{
	
	// UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
	const uint16_t UBX_NAV_PVT_LEN = 92;

	struct NAV_PVT
	{
		uint32_t iTOW; // GPS time of week of the navigation epoch: ms
		uint16_t year; // Year (UTC)
		uint8_t month; // Month, range 1..12 (UTC)
		uint8_t day;   // Day of month, range 1..31 (UTC)
		uint8_t hour;  // Hour of day, range 0..23 (UTC)
		uint8_t min;   // Minute of hour, range 0..59 (UTC)
		uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
		union
		{
			uint8_t all;
			struct
			{
				uint8_t validDate : 1;	   // 1 = valid UTC Date
				uint8_t validTime : 1;	   // 1 = valid UTC time of day
				uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
				uint8_t validMag : 1;	   // 1 = valid magnetic declination
			} bits;
		} valid;
		uint32_t tAcc;	 // Time accuracy estimate (UTC): ns
		int32_t nano;	 // Fraction of second, range -1e9 .. 1e9 (UTC): ns
		uint8_t fixType; // GNSSfix Type:
						 // 0: no fix
						 // 1: dead reckoning only
						 // 2: 2D-fix
						 // 3: 3D-fix
						 // 4: GNSS + dead reckoning combined
						 // 5: time only fix
		union
		{
			uint8_t all;
			struct
			{
				uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
				uint8_t diffSoln : 1;  // 1 = differential corrections were applied
				uint8_t psmState : 3;
				uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
				uint8_t carrSoln : 2;	  // Carrier phase range solution status:
										  // 0: no carrier phase range solution
										  // 1: carrier phase range solution with floating ambiguities
										  // 2: carrier phase range solution with fixed ambiguities
			} bits;
		} flags;
		union
		{
			uint8_t all;
			struct
			{
				uint8_t reserved : 5;
				uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
				uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
				uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
			} bits;
		} flags2;
		uint8_t numSV;	  // Number of satellites used in Nav Solution
		int32_t lon;	  // Longitude: deg * 1e-7
		int32_t lat;	  // Latitude: deg * 1e-7
		int32_t height;	  // Height above ellipsoid: mm
		int32_t hMSL;	  // Height above mean sea level: mm
		uint32_t hAcc;	  // Horizontal accuracy estimate: mm
		uint32_t vAcc;	  // Vertical accuracy estimate: mm
		int32_t velN;	  // NED north velocity: mm/s
		int32_t velE;	  // NED east velocity: mm/s
		int32_t velD;	  // NED down velocity: mm/s
		int32_t gSpeed;	  // Ground Speed (2-D): mm/s
		int32_t headMot;  // Heading of motion (2-D): deg * 1e-5
		uint32_t sAcc;	  // Speed accuracy estimate: mm/s
		uint32_t headAcc; // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
		uint16_t pDOP;	  // Position DOP * 0.01
		union
		{
			uint8_t all;
			struct
			{
				uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height and hMSL
			} bits;
		} flags3;
		uint8_t reserved1[5];
		int32_t headVeh; // Heading of vehicle (2-D): deg * 1e-5
		int16_t magDec;	 // Magnetic declination: deg * 1e-2
		uint16_t magAcc; // Magnetic declination accuracy: deg * 1e-2

		// static UBX_NAV_PVT parse(const uint8_t msg[]) {
		//   UBX_NAV_PVT data;
		//   data.iTOW = extractLong(msg, 0);
		//   data.year = extractInt(msg, 4);
		//   data.month = extractByte(msg, 6);
		//   data.day = extractByte(msg, 7);
		//   data.hour = extractByte(msg, 8);
		//   data.min = extractByte(msg, 9);
		//   data.sec = extractByte(msg, 10);
		//   data.valid.all = extractByte(msg, 11);
		//   data.tAcc = extractLong(msg, 12);
		//   data.nano = extractSignedLong(msg, 16);  // Includes milliseconds
		//   data.fixType = extractByte(msg, 20);
		//   data.flags.all = extractByte(msg, 21);
		//   data.flags2.all = extractByte(msg, 22);
		//   data.numSV = extractByte(msg, 23);
		//   data.lon = extractSignedLong(msg, 24);
		//   data.lat = extractSignedLong(msg, 28);
		//   data.height = extractSignedLong(msg, 32);
		//   data.hMSL = extractSignedLong(msg, 36);
		//   data.hAcc = extractLong(msg, 40);
		//   data.vAcc = extractLong(msg, 44);
		//   data.velN = extractSignedLong(msg, 48);
		//   data.velE = extractSignedLong(msg, 52);
		//   data.velD = extractSignedLong(msg, 56);
		//   data.gSpeed = extractSignedLong(msg, 60);
		//   data.headMot = extractSignedLong(msg, 64);
		//   data.sAcc = extractLong(msg, 68);
		//   data.headAcc = extractLong(msg, 72);
		//   data.pDOP = extractInt(msg, 76);
		//   data.flags3.all = extractByte(msg, 78);
		//   data.headVeh = extractSignedLong(msg, 84);
		//   data.magDec = extractSignedInt(msg, 88);
		//   data.magAcc = extractInt(msg, 90);
		// }
	};
	// UBX-NAV-POSECEF (0x01 0x09): Navigation Position ECEF
	const uint16_t UBX_NAV_POSECEF_LEN = 20;
	struct UBX_NAV_POSECEF
	{
		uint32_t iTOW; // GPS time of week of the navigation epoch: ms
		int32_t ecefX; // ECEF X coordinate: mm
		int32_t ecefY; // ECEF Y coordinate: mm
		int32_t ecefZ; // ECEF Z coordinate: mm
		uint32_t pAcc; // Position accuracy estimate: mm

		// static UBX_NAV_POSECEF parse(const uint8_t msg[]) {
		//   UBX_NAV_POSECEF data;
		//   data.iTOW = extractLong(msg, 0);
		//   data.ecefX = extractSignedLong(msg, 4);
		//   data.ecefY = extractSignedLong(msg, 8);
		//   data.ecefZ = extractSignedLong(msg, 12);
		//   data.pAcc = extractLong(msg, 16);
		// }
	};
	// UBX-NAV-VELECEF (0x01 0x12): Navigation Velocity ECEF
	const uint16_t UBX_NAV_VELECEF_LEN = 16;
	struct UBX_NAV_VELECEF
	{
		uint32_t iTOW;	// GPS time of week of the navigation epoch: ms
		int32_t ecefVX; // ECEF X velocity: mm/s
		int32_t ecefVY; // ECEF Y velocity: mm/s
		int32_t ecefVZ; // ECEF Z velocity: mm/s
		uint32_t sAcc;	// Speed accuracy estimate: mm/s

		// static UBX_NAV_VELECEF parse(const uint8_t msg[]) {
		//   UBX_NAV_VELECEF data;
		//   data.iTOW = extractLong(msg, 0);
		//   data.ecefVX = extractSignedLong(msg, 4);
		//   data.ecefVY = extractSignedLong(msg, 8);
		//   data.ecefVZ = extractSignedLong(msg, 12);
		//   data.sAcc = extractLong(msg, 16);
		// }
	};

}

#endif