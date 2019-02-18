/*
 * gps.h
 *
 *  Created on: 2019¦~1¤ë22¤é
 *      Author: John
 */

#ifndef GPS_H_
#define GPS_H_
#include "cmsis_os.h"

#ifdef __cplusplus
 extern "C" {
#endif

//#define gps_buffer_size  512
#define buffer_num_1  0
#define buffer_num_2  (gps_buffer_size/2)
#define GPS_BUFFER_SIZE  256 //256
#define NMEA_MAX_SIZE  128
#define ONE_GPS_BUFFER_SIZE 256
#define GGA_LAT_COUNT 18
#define GGA_LAT_DIR_COUNT 29
#define GGA_LONG_COUNT 31
#define GGA_LONG_DIR_COUNT 43
#define GGA_HDOP_COUNT 50

#define GSA_FIX_TYPE_COUNT 9

void HLGPS_GetDataFromDriver(void);
uint8_t* Read_GPS_To_GPSBuffer(uint32_t*);
void HLGPS_StartToParseNMEA(void);
void HLGPS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H_ */
