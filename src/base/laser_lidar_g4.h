#ifndef __LASER_LIDAR_G4_H__
#define __LASER_LIDAR_G4_H__
#define PER_PACKAGE_NUM_POINT_G4 0X5A//0x28+4+4+CRC(2)
bool checksum_crc_g4(unsigned char buf[PER_PACKAGE_NUM_POINT_G4]);
int Get_LiDAR_Data_G4(void);
#endif