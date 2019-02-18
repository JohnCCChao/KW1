/*
 * nbiot.c
 *
 *  Created on: 2019¦~1¤ë25¤é
 *      Author: John
 */

#include "stm32l4xx_hal.h"
#include "stdio.h"
#include "nbiot.h"
#include "FreeRTOS.h"

#include <sys/time.h>

extern UART_HandleTypeDef huart1;
extern char glo_lat_value;
extern char glo_long_value;
extern char glo_hdop_value;
extern uint16_t nbiot_data_point;
extern uint8_t nbiot_buffer[1024];

static char const* LOCATION_SERVER_URL = "http://lts.c2dms.com/";
static char const* LOCATION_TAG = "FusedLocation/FIHLocationRequest";
static char const* REQUEST_PATH = "api/LocationService/ReportLocation";
static char const* HTTP_HEADER_CONTENT = "application/json";

long int LTimestampMilliscond(void)
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec*1000 + tp.tv_usec/1000;
}

void NBString_ASCII_HEX(char *src,char *dest)
{
	int i, len;

	len = strlen(src);
	if(src[len-1]=='\n')
		src[--len] = '\0';

	for(i = 0; i<len; i++){
		sprintf(dest+i*2, "%02X", src[i]);
	}
}

void NBIOT_Init(void)
{
	/* Disable Module Sleep Mode */
}

void NBIOT_Network_Ready(void)
{

}

uint8_t NBIOT_AT_Command_Send(const char *at_cmd,const char *at_respond,int timeout)
{
	HAL_StatusTypeDef status;
	uint8_t return_status = 0;

	status = HAL_UART_Transmit(&huart1, (char*)at_cmd, strlen(at_cmd), 0xFF);
	vTaskDelay(timeout);

	if(strstr(nbiot_buffer,at_respond) != NULL)
	{
		printf("%s\r\n",nbiot_buffer);
		return_status = 1;
		printf("Get ACK - %d\r\n",timeout);
	}
	else
	{
		printf("%s\r\n",nbiot_buffer);
		printf("No ACK - %d\r\n",timeout);
		return_status = 0;
	}
	HAL_UART_AbortReceive_IT(&huart1);
	//memset(nbiot_buffer,'3',1024);
	HAL_UART_Receive_IT(&huart1,nbiot_buffer,1024);
	return return_status;
}

void NBIOT_LocationUpdate_Request(void)
{
	char 	strLocationServer[128] = "";
	char 	strLocationRequest[512] = "";
	char 	strLocationContentHex[512] = "";

	char	strDeviceIMEI[20] = "353148070025903";
	int		iPowerStatus = 92;
	uint8_t status = 0;

	/*	Check GPS Fix Status	*/

	printf("Send AT command to NBIOT start\r\n");
	// Don't let chip enter to sleep mode

	NBIOT_AT_Command_Send("AT\r\n","OK",2000);

#if 0
	NBIOT_AT_Command_Send("AT+QSCLK=0\r\n","OK",2000);

	status = NBIOT_AT_Command_Send("AT+CGACT?\r\n","+CGACT=1,1",3000);

	if(status == 0)
	{
		printf("NBIOT not connect network\r\n");
	    return;
	}
	/* 	Create HTTP Client */
#if 0
	sprintf(strLocationServer, "AT+QHTTPCREATE=0,%d,%d,\"\"%s\",,,0,,0,,0,\"\r\n", 13+strlen(LOCATION_SERVER_URL),13+strlen(LOCATION_SERVER_URL),LOCATION_SERVER_URL);
	NBIOT_AT_Command_Send(strLocationServer,"OK",2000);

	/* Connect to Server */
	NBIOT_AT_Command_Send("AT+QHTTPCON=0\r\n","OK",2000);	// Connect to server

	/* Send HTTP Post method to the user content data with path is ¡§/api/LocationService/ReportLocation¡¨ */

#if 0
	sprintf(strLocationRequest, "{\"DeviceID\":\"%s\",\"TimeStamp\":%lu,\"gpsinfo\":[[%s,%s]],\"Accuracy\":\"%s\",\"PowerStatus\":%d,\"ExtraInformation\":[{\"Key\":\"isGPS\",\"Value\":true}]}", strDeviceIMEI, LTimestampMilliscond(), glo_lat_value, glo_long_value, glo_hdop_value, iPowerStatus);
#else
	sprintf(strLocationRequest, "{\"DeviceID\":\"%s\",\"TimeStamp\":%lu,\"gpsinfo\":[[24.96190224,121.41495005]],\"Accuracy\":\"35.0\",\"PowerStatus\":%d,\"ExtraInformation\":[{\"Key\":\"isGPS\",\"Value\":true}]}", strDeviceIMEI, LTimestampMilliscond(), iPowerStatus);
#endif
	NBString_ASCII_HEX(strLocationRequest, strLocationContentHex);
	sprintf(strLocationRequest, "AT+QHTTPSEND=0,%d,%d,\"0,1,35,\"/api/LocationService/ReportLocation\",0,,16,\"application/json\",%d,\"%s\"\"\r\n", 76+strlen(strLocationContentHex), 76+strlen(strLocationContentHex), strlen(strLocationContentHex), strLocationContentHex);
	NBIOT_AT_Command_Send(strLocationRequest,"+QHTTPNMIC",5000);

	NBIOT_AT_Command_Send("AT+QHTTPDISCON=0\r\n","OK",2000);	// Disconnect to server
	NBIOT_AT_Command_Send("AT+QHTTPDESTROY=0\r\n","OK",2000);	// Destroy http client instance 0
#endif
#endif
	printf("Send AT command to NBIOT end\r\n");
}
