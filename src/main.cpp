#include "stdlib.h"
#include "lidar_delat_2a.h"

unsigned char buf[PER_PACKAGE_NUM_POINT] = { 0xAA,0x00,0x40,0x01,0x61,0xAD,0x00,0x38,0x94,0x00,0x76,0x7B,0x0C,0x96,0x18,0x47,0x98,0x16,0x33,0x9A,0x16,0x47,0x99,0x16,0xC5,0x97,0x17,0x40,0x9B,0x17,0xD3,0x97,0x18,0x69,0x98,0x18,0xF3,0x8A,0x19,0x81,0x5F,0x1B,0x4A,0x63,0x1C,0x0C,0x46,0x1C,0xFB,0x3E,0x1F,0x06,0x38,0x14,0xB0,0x3C,0x13,0xC5,0x4B,0x13,0x4E,0x00,0x00,0x00,0x14,0x20 };
int main(int argc ,char* argv[]) {
	lidar_delat_get_data();
	//bool ret = checksum_crc(buf);
	//printf("ret=%d\r\n",ret);
	//system("pause");
	return 0;
}