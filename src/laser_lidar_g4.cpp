#include "laser_lidar_g4.h"
#include "win_serial.h"
#include "comm_data.h"
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <Windows.h>

using namespace std;
Serial G4_serial;


static uint16_t find_head_offset(const uint8_t* buf, const uint16_t buf_len, const uint8_t* head_val, const uint16_t head_len) {
	uint16_t head_found_count = 0;
	uint16_t data_index = 0;
	for (; data_index < buf_len; ++data_index) {
		if (head_found_count >= head_len) {
			break;
		}
		if (buf[data_index] == head_val[head_found_count]) {
			++head_found_count;
		}
		else {
			head_found_count = 0;
		}
	}
	return data_index - head_found_count;
}

bool checksum_crc_g4(unsigned char buf[PER_PACKAGE_NUM_POINT_G4]) {
	uint16_t chk32 = 0;
	for (int i = 0; i < 4; ++i) {
		uint16_t s = buf[2 * i] + (buf[2 * i + 1] << 8);
		chk32 ^= buf[i];
	}
	for (int i = 10; i < PER_PACKAGE_NUM_POINT_G4; ++i) {
		chk32 ^= buf[i];
	}
	bool ret = (buf[9] == (chk32 & 0xff)) && (buf[8] = (chk32 / 256) & 0xff);
	if (!ret) {
		printf("check %x %x|%x %x\n", buf[9], buf[8], (chk32 / 256) & 0xff, (chk32 & 0xff));
		printf("checksum fail\r\n");
	}
	else {
		//printf("checksum successful!\r\n");
	}
	return ret;
}
void LiDAR_Pause() {
	uint8_t cmd_buf[3];
	if (G4_serial.isOpen()) {
		cmd_buf[0] = 0XA5;
		cmd_buf[1] = 0X65;
		G4_serial.write(cmd_buf,2);
	}
}
void LiDAR_Resume() {
	uint8_t cmd_buf[3];
	if (G4_serial.isOpen()) {
		cmd_buf[0] = 0XA5;
		cmd_buf[1] = 0X60;
		G4_serial.write(cmd_buf, 2);
	}
}
int Get_LiDAR_Data_G4(void) {
	bool retval = false;
	const uint32_t serial_baudrate = 230400;
	retval = G4_serial.open("COM7", 230400);
	if (retval) {
		printf("Serial com open successful.\n");
	}
	G4_serial.setDTR(true);
	G4_serial.flushInput();
	LiDAR_Resume();
	float laser_angle = 0.0;
	printf("laser_ydlidar_g4 angle=%f retval=%d\r\n", laser_angle, retval);
	uint16_t FirstSampleAngle, LastSampleAngle;
	float IntervalSampleAngle = 0.f, IntervalSampleAngle_LastPackage = 0.f;
	int prev_angle = 0, angle_count = 0, raw_angle_count = 0;
	struct laser_ranges laser_ranges;
	memset(laser_ranges.ranges, 0, sizeof(laser_ranges.ranges));
	laser_ranges.raw_count = 0;
	uint8_t *buf = new uint8_t[520];
	if (buf == NULL)
	{
		printf("new buffer failed ,no space");
		return -1;
	}
	int valid_in_buf = 0;
	int pre_v_angle = 0;
	float scanFrequenceHZ = 0.f;
	constexpr uint8_t head_size = 3;
	constexpr uint8_t head[head_size] = { 0xaa, 0x55, 0x00 };
	uint16_t angleRange = 0;
	uint16_t angleStep = 0;
	while (retval) {

		memset(buf, 0, sizeof(buf));
		valid_in_buf = 0;
		while (valid_in_buf < 4)
		{
			valid_in_buf += G4_serial.read((uint8_t*)buf + valid_in_buf, 4 - valid_in_buf);
			std::cout << "G4_read out="<< valid_in_buf << std::endl;
		}

		// now 4 bytes in buffer, check header
		int wasted = 0;
		while ((buf[0] != 0xaa || buf[1] != 0x55 ||(buf[2]!=0x0)|| (buf[3]!=0x28) )) {
			buf[0] = buf[1];
			buf[1] = buf[2];
			buf[2] = buf[3];
			G4_serial.read(buf + 3, 3);
			wasted++;
		}
		if (wasted > 0) {
			printf("wasted %d\n", wasted);
		}
		//while (valid_in_buf < 8) {
		//	valid_in_buf += G4_serial.read((uint8_t*)buf + valid_in_buf, 8-valid_in_buf);
		//}
		//while (valid_in_buf < 10) {
		//	valid_in_buf += G4_serial.read(buf + valid_in_buf, 10-valid_in_buf);
		//}
		const uint8_t sampleNum = buf[3];
		uint32_t frame_byte_len = (uint32_t)(sampleNum*2 + 4 + 4 + 2);
		std::cout << "frame_byte_len=" << frame_byte_len << std::endl;
		while (valid_in_buf < (int)(frame_byte_len)-4) {
			valid_in_buf += G4_serial.read(buf + valid_in_buf, (10 + sampleNum * 2) - valid_in_buf);
		}
		//const uint32_t serial_trans_delay = frame_byte_len * 1000 * 1000 / (serial_baudrate / 10);
		//const uint64_t pack_end_timestamp = GetTickCount()*1000.f - serial_trans_delay;
		//printf("buf:%x %x\r\n",buf[8],buf[9]);
		//CRC-16
		uint16_t crcXor = buf[8] | (buf[9] << 8);
		uint16_t FSA = buf[4] | (buf[5] << 8);
		uint16_t LSA = buf[6] | (buf[7] << 8);
		uint8_t CT = buf[2];
		uint16_t PH = buf[0] | (buf[1] << 8);

		crcXor ^= PH;
		crcXor ^= FSA;
		crcXor ^= LSA;
		crcXor ^= CT | (sampleNum << 8);
		for (int i = 0; i < sampleNum; ++i) {
			crcXor ^= (buf[10 + i * 2] | (buf[11 + i * 2] << 8));
		}
		if (crcXor != 0) {
			printf("crcXor failed:0x%x\r\n", crcXor);
			continue;
		}
		if (CT & 0x01) {
			scanFrequenceHZ = (float)((CT & 0XFE) >> 1);
		}
		//std::cout << "FrameStart..." << std::endl;
		float rps = 5.0;// speed;
		FirstSampleAngle = FSA << 1;
		LastSampleAngle = LSA << 1;
		if (sampleNum == 1) {
			IntervalSampleAngle = 0.f;
		}
		else {
			if (LastSampleAngle < FirstSampleAngle) {
				if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64)) {
					IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle - 
						FirstSampleAngle) / ((sampleNum - 1) * 1.0f));
					IntervalSampleAngle_LastPackage = IntervalSampleAngle;
				}
				else {
					IntervalSampleAngle = IntervalSampleAngle_LastPackage;
				}
			}
			else {
				IntervalSampleAngle = (float)(LastSampleAngle - FirstSampleAngle) / ((sampleNum - 1)*1.0f);
			}
		}
		float first_angle = (float)FirstSampleAngle / 64.f;
		float last_angle = (float)LastSampleAngle / 64.f;
		float diff = last_angle - first_angle;
		//cout << "first_angle:" << first_angle <<" last_angle:"<<last_angle<<" diff:"<<diff<< endl;
		if (diff < 0) {
			diff += 360;
		}
		const uint32_t us_per_360 = 1000 * 1000 / rps;
		const uint32_t time_step = (uint32_t)(diff*us_per_360 / 360 / (sampleNum - 1));
		//static uint64_t time_start = pack_end_timestamp - (sampleNum - 1)*time_step;
		diff /= (sampleNum - 1);
/*
		for (int i = 0; i < sampleNum; ++i) {
			float a = (first_angle + diff*i) + laser_angle;

			while (a >= 360.)a -= 360.;
			while (a < 0.)a += 360;
			const int angle = (int)a;
			//cout << "a=" << a <<" angle="<<angle<< " prev_angle=" << prev_angle << endl;
			if (angle < prev_angle) {
				const uint64_t next_start = pack_end_timestamp - ((sampleNum - 1) - i)*time_step;
				const uint64_t time_end = next_start - time_step;
				const uint64_t time_dur = time_end - time_start;
				const uint32_t round_time_min = us_per_360 * 3 / 4;
				const uint32_t round_time_max = us_per_360 * 5 / 4;
				if (angle_count > 200 && time_dur > round_time_min&&time_dur < round_time_max) {
					printf("SetRanges....\r\n");
				}
				else {
					printf("laser not good to push..%d %lld %d %d %f\r\n", angle_count, time_dur, us_per_360, sampleNum, diff);
				}
				time_start = next_start;
				angle_count = 0;

			}
			float dist = ((buf[14 + 3 * i] << 8) | buf[15 + 3 * i])*0.25f;
			prev_angle = angle;
			angle_count++;
		}
*/
		printf("FrameEnd...\r\n");
	}
	LiDAR_Pause();
	G4_serial.close();
	return 0;
}