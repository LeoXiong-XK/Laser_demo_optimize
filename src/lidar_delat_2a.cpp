#include "lidar_delat_2a.h"
#include "win_serial.h"
#include "comm_data.h"
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <Windows.h>

using namespace std;
Serial delat_serial;
bool retval = false;
	
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

bool checksum_crc(unsigned char buf[PER_PACKAGE_NUM_POINT]) {
	uint16_t chk32 = 0;
	for (int i = 0; i < PER_PACKAGE_NUM_POINT - 2; ++i) {
		chk32 += buf[i];
	}
	bool ret = (buf[PER_PACKAGE_NUM_POINT - 1] == (chk32 & 0xff)) && (buf[PER_PACKAGE_NUM_POINT - 2] = (chk32 / 256) & 0xff);
	if (!ret) {
		printf("check %x %x|%x %x\n",buf[PER_PACKAGE_NUM_POINT - 2], buf[PER_PACKAGE_NUM_POINT - 1], (chk32 / 256) & 0xff, (chk32 & 0xff));
		printf("checksum fail\r\n");
	}
	else {
		//printf("checksum successful!\r\n");
	}
	return ret;
}
int lidar_delat_get_data(void) {
	const uint32_t serial_baudrate = 115200;
	retval = delat_serial.open("COM7", 115200);
	if (retval) {
		printf("Serial com open successful.\n");
	}
	delat_serial.setDTR(true);
	delat_serial.flushInput();

	float laser_angle = 0.0;
	printf("laser_ydlidar_delat_2a angle=%f retval=%d\r\n", laser_angle, retval);

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
	int valid_in_buf =0;
	int pre_v_angle = 0;
	//uint8_t simpleNum;
	//float scanFrequenceHz = 0.0;
	//uint16_t FirstSampleAngle, LastSampleAngle;
	//float IntervalSampleAngle = 0, IntervalSampleAngle_LastPackage = 0;
	constexpr uint8_t head_size = 6;
	constexpr uint8_t head[head_size] = { 0xaa, 0x00, 0x40, 0x01, 0x61, 0xad };
	uint16_t angleRange = 0;
	uint16_t angleStep = 0;
	bool	flagOfNewNodes = false;
	size_t recvNodeCount = 0;
	while (retval) {

		memset(buf, 0, sizeof(buf));
#if 1
		while (valid_in_buf < 6)
		{
			valid_in_buf += delat_serial.read((uint8_t*)buf + valid_in_buf, 6 - valid_in_buf);
		}

		// now 4 bytes in buffer, check header
		int wasted = 0;
		while ((buf[0] != 0xaa || buf[1] != 0x00 || buf[2] != 0x40 || buf[3] != 0x01 || buf[4] != 0x61 || buf[5] != 0xad)) {
			buf[0] = buf[1];
			buf[1] = buf[2];
			buf[2] = buf[3];
			buf[3] = buf[4];
			buf[4] = buf[5];
			delat_serial.read(buf+5, 1);
			wasted++;
		}
		if (wasted > 0) {
			printf("wasted %d\n", wasted);
		}
		while (valid_in_buf < PER_PACKAGE_NUM_POINT - 6) {
			valid_in_buf += delat_serial.read((uint8_t*)buf + valid_in_buf, PER_PACKAGE_NUM_POINT - 6);
		}
#else
		int wasted = 0;
		while (valid_in_buf < 66) {
			valid_in_buf += delat_serial.read((uint8_t*)buf + valid_in_buf, PER_PACKAGE_NUM_POINT);
		}
		const int head_index = find_head_offset(buf, valid_in_buf,head, 6);
		valid_in_buf -= head_index;
		if (valid_in_buf < 66) {
			if (valid_in_buf > 0) {
				memcpy(buf, buf + head_index, valid_in_buf);
			}
			printf("wasted %d\n", wasted);
			continue;
		}
#endif
		const uint32_t serial_trans_delay = 66 * 1000 * 1000 / (serial_baudrate / 10);
		const uint64_t pack_end_timestamp = GetTickCount()*1000.f - serial_trans_delay;
		//check CRC-16	
		if (!checksum_crc(buf)) {
			std::cout << "Checksum failed!" << std::endl;
			continue;
		}
		uint16_t frameLen = buf[1] << 8 | buf[2];
		uint16_t param_lens = (buf[6] << 8) | buf[7];
		float speed = buf[8] * 0.05f;

		cout << "Freq_hz:" << speed <<"hz"<< endl;

		uint16_t sampleNum = (param_lens - 5) / 3;

		//std::cout << "FrameStart..." << std::endl;
		float rps= speed;
		float first_angle = ((buf[11] << 8) | buf[12])*0.01f;
		float last_angle = first_angle + 22.5*(sampleNum - 1) / sampleNum;
		float diff = last_angle - first_angle;
		//cout << "first_angle:" << first_angle <<" last_angle:"<<last_angle<<" diff:"<<diff<< endl;
		if (diff < 0) {
			diff += 360;
		}
		const uint32_t us_per_360 = 1000 * 1000 / rps;
		const uint32_t time_step = (uint32_t)(diff*us_per_360 / 360 / (sampleNum - 1));
		static uint64_t time_start = pack_end_timestamp - (sampleNum - 1)*time_step;
		diff /= (sampleNum - 1);

		for (int i = 0; i < sampleNum; ++i) {
			float a = (first_angle + diff*i)+ laser_angle;

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
					printf("laser not good to push..%d %lld %d %d %f\r\n",angle_count,time_dur,us_per_360,sampleNum, diff);
				}
				time_start = next_start;
				angle_count = 0;

			}
			float dist = ((buf[14 + 3 * i] << 8) | buf[15 + 3 * i])*0.25f;
			cout << "dist:" << dist << endl;
			prev_angle = angle;
			angle_count++;
		}
		 
		printf("FrameEnd...\r\n");
		valid_in_buf = 0;
	}
	return 0;
}