#pragma once
#ifndef __LIDAR_DELAT_2A_H__
#define __LIDAR_DELAT_2A_H__
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define PER_PACKAGE_NUM_POINT 0X40+2

#define angle_resolution 0.01f
#define dist_resolution 0.25f
#define NUMBER_OF_TEETH	16
int lidar_delat_get_data(void);
bool checksum_crc(unsigned char buf[PER_PACKAGE_NUM_POINT]);

/////////////ͨ��֡�ṹ////////////////////////////
typedef struct _comm_frame_t
{
	uint8_t		frameStart;		//֡ͷ
	uint16_t	frameLen;		//֡����
	uint8_t		addr;			//��ַ��
	uint8_t		frameType;		//֡����
	uint8_t		cmd;			//������
	uint16_t	paramLen;		//��������
	uint8_t		paramBuf[0];	//����
} COMM_FRAME_T;

//�����״ﵥ�����������Ϣ
typedef struct _rslidar_signal_distance_unit_t {
	uint8_t			signalValue;	//�Ƕ�ֵ
	float			speed;
	float			angleoffset;
	uint16_t		angle;			//�Ƕ�
	uint16_t		distanceValue;	//����ֵ
}RSLIDAR_SIGNAL_DISTANCE_UNIT_T;
#endif