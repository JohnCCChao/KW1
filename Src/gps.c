/*
 * gps.c
 *
 *  Created on: 2019¦~1¤ë22¤é
 *      Author: John
 */
#include "main.h"
#include "gps.h"
#include "cmsis_os.h"
#include "task.h"
#include "stdbool.h"
#include "stdio.h"

bool HLGPS_ParseGGA(char* sentence_temp);
bool HLGPS_ParseGSA(char* sentence_temp);
// TODO: Add dummy gps parser functions
bool HLGPS_ParseRMC(char* sentence_temp);
bool HLGPS_ParseVTG(char* sentence_temp);
bool HLGPS_ParseGSV(char* sentence_temp);
bool HLGPS_ParseGLL(char* sentence_temp);
bool HLGPS_ParseSkip(char* sentence_temp);

void HLString_CharGetLeft(char*,int);
void HLString_CharGetRight(char*,int);

uint8_t* m_pGPS;
uint32_t m_GPSSize;
extern uint16_t gps_data_point;
extern uint16_t gps_buffer_base;
// TODO: Modify gps_buffer
extern uint8_t gps_buffer[2][GPS_BUFFER_SIZE];
char m_fragment[NMEA_MAX_SIZE] = "";
char glo_lat_value[11] = "";  //Global latitude for NBIOT
char glo_long_value[12] = ""; //Global longitude for NBIOT
char glo_hdop_value[10] =""; //Global hdop for NBIOT
uint8_t glo_fix_type = 1;  //GPS status

// TODO: Modify gps parser functions
char NMEA_PREFIX[][8] = {
		"$GNGGA",
		"$GPGGA",
		"$GNGSA",
		"$GPGSA",
		"$GNRMC",
		"$GNVTG",
		"$GPGSV",
		"$GLGSV",
		"$GNGLL"
};

typedef bool (*FP_CHAR)(char*);
// TODO: Add dummy gps parser functions
FP_CHAR fp_GPS[] = {
		HLGPS_ParseGGA,
		HLGPS_ParseGGA,
		HLGPS_ParseGSA,
		HLGPS_ParseGSA,
		HLGPS_ParseSkip, //HLGPS_ParseRMC,
		HLGPS_ParseSkip, //HLGPS_ParseVTG,
		HLGPS_ParseSkip, //HLGPS_ParseGSV,
		HLGPS_ParseSkip, //HLGPS_ParseGSV,
		HLGPS_ParseSkip  //HLGPS_ParseGLL
};

typedef enum
{
	HLVECTOR_INT32S = 0,
	HLVECTOR_INT32U,
	HLVECTOR_FP32,
	HLVECTOR_STR
};

typedef struct GPGGA
{
	int32_t latitude; //10^6
	int32_t longitude; //10^6
	int32_t hdop; //10^2
}GPGGA;

typedef struct GPGSA
{
	uint8_t fixType;
}GPGSA;

GPGSA m_gsa_data;
GPGGA m_gga_data;

void HLGPS_InitGPSGSA(void)
{
	memset(&m_gsa_data,0,sizeof(GPGSA));
	m_gsa_data.fixType = 1;
}
void HLGPS_InitGPSGGA(void)
{
	memset(&m_gga_data,0,sizeof(m_gga_data));

	m_gga_data.latitude = 66535;
	m_gga_data.longitude = 65535;
	m_gga_data.hdop = 65535;
}

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
void HLGPS_ubx_2_checksum(uint8_t *ck_a, uint8_t *ck_b, uint8_t* inString, uint16_t len)
{
	uint16_t strLen = len;
	uint16_t i = 0;

	*ck_a = 0;
	*ck_b = 0;
	while(strLen--)
	{
		*ck_a = ( *ck_a + *(inString+i)) & 0xff;
		*ck_b = ( *ck_b + *(ck_a)) & 0xff;
		i++;
	}
	*(inString+len) = *ck_a;
	*(inString+len+1) = *ck_b;

}

// UBX-CFG-RATE
void HLGPS_SendUbxCmd(uint16_t measInterval)
{
   	HAL_StatusTypeDef status;
   	uint8_t ck_a, ck_b;
   	// UBX-CFG-PRT
   	unsigned char cmd1[]={
    // UBX-CFG-RATE - 10 sec update once
    // b5 62 06 08 06 00 10 27 01 00 01 00  4d dd
    			0xb5, 		// [00] SYNC1
				0x62, 		// [01] SYNC2
    			0x06, 		// [02] class
				0x08, 		// [03] id
				0x06, 		// [04] len1
				0x00, 		// [05] len2
				0x10, 		// [06] measRate1
				0x27, 		// [07] measRate2
    			0x01,		// [08] navRate1
				0x00,		// [09] navRate2
				0x01,		// [10] timeRef
				0x00,		// [11] timeref
				0x4d,		// [12] CK_A
				0xdd		// [13] CK_B
   	};

   	cmd1[6] = (uint8_t) measInterval & 0xff;
   	cmd1[7] = (uint8_t) (measInterval>>8) & 0xff;

   	HLGPS_ubx_2_checksum(&ck_a, &ck_b, &cmd1[2], 10);
   	status = HAL_UART_Transmit(&huart4, (char*)cmd1, 14, 0xFF);

}

void HLGPS_Init(void)
{
    HLGPS_InitGPSGGA();
    HLGPS_InitGPSGSA();
    // Set 20 sec as measurement interval
    HLGPS_SendUbxCmd(20*1000);
}

int HLString_IndexOf(const char* str,const char token)
{
	bool isFind = false;
	int pos = 0;

	for(;pos<(int)strlen(str);pos++)
	{
		if(!strncmp(str+pos,&token,1))
		{
			isFind = true;
			break;
		}
	}

	if(isFind)
		return pos;
	else
		return -1;
}

void HLGPS_DecodeLong(const char* value,const char* direction)
{
	int index = HLString_IndexOf(value,'.');

	if(index == -1)
	{
	    glo_long_value[0] = '\0';
		return;
	}

	strncpy(glo_long_value,value,11);
	strncpy(glo_long_value+11,direction,1);
}

void HLGPS_DecodeLat(const char* value,const char* direction)
{
	int index = HLString_IndexOf(value,'.');

	if(index == -1)
	{
		glo_lat_value[0] = '\0';
	    return;
	}

	strncpy(glo_lat_value,value,10);
	strncpy(glo_lat_value+10,direction,1);
}

// TODO: Modify gps parser functions
//#define GPS_MAX_MSG_TYPE 3
#define GPS_MAX_FIELDID_MAX 32
#define GPS_MAX_FIELD_LEN 20
char gpsFields[GPS_MAX_FIELDID_MAX][GPS_MAX_FIELD_LEN];

void HRString_CharSplit_For_GSA(const char *src,const char *ch)
{
	int count = 0;
	int field_index = 0;
	int i, size;
	//return;

	for(i=0; i<=(int)strlen(src); i++)
	{
		if(!strncmp(src+i,ch,1) || i == strlen(src))
		{
#if 1
	    	memset(&gpsFields[field_index][0], 0, GPS_MAX_FIELD_LEN);
			size = i - count;
			if( size > 0 )
		    {
		    	if(size < GPS_MAX_FIELD_LEN )
		    		strncpy(&gpsFields[field_index][0],src+count,(i-count));
		    	else
		    		return;
		    		//while (1);
		    }

		    field_index++;
	    	if(field_index >= GPS_MAX_FIELDID_MAX)
	    		return;
#endif
		    count = i+1;
		}
	}
#if 1
	printf("%s-%s--(%d)\r\n",
			gpsFields[0], gpsFields[2], strlen(src));
	//printf("gs=%s\r\n",src);
#else
	printf("GSA\r\n");
#endif


}

//TODO: Extract data to form gpsFields

void HRString_CharSplit_For_GGA(const char *src,const char *ch)
{
	int count = 0;
	int i, size;
	int field_index = 0;

	//return;

	for(i=0; i<=(int)strlen(src); i++)
	{
		if(!strncmp(src+i,ch,1) || i == strlen(src))
		{
#if 1
	    	memset(&gpsFields[field_index][0], 0, GPS_MAX_FIELD_LEN);
			size = i - count;
			if( size > 0 )
		    {
		    	if(size < GPS_MAX_FIELD_LEN )
		    		strncpy(&gpsFields[field_index][0],src+count,(i-count));
		    	else
		    		//while (1);
		    		return;
		    }

		    field_index++;
	    	if(field_index >= GPS_MAX_FIELDID_MAX)
	    		return;
#endif
		    count = i+1;
		}
	}
#if 1
	printf("%s-%s-%s-%s-%s-%s-%s-%s--(%d)\r\n",
			gpsFields[0], gpsFields[2], gpsFields[3],
			gpsFields[4], gpsFields[5], gpsFields[6],
			gpsFields[7], gpsFields[8], strlen(src));
	//printf("gs=%s\r\n",src);
#else
	printf("GGA\r\n");
#endif

}

bool HLGPS_ParseGSA(char* sentence_temp)
{
	char ch = ',';
	HRString_CharSplit_For_GSA(sentence_temp,&ch);

	return true;
}

bool HLGPS_ParseGGA(char* sentence_temp)
{
	char ch = ',';
	HRString_CharSplit_For_GGA(sentence_temp,&ch);

	return true;
}

#if 0
bool HLGPS_ParseRMC(char* sentence_temp){return true;}
bool HLGPS_ParseVTG(char* sentence_temp){return true;}
bool HLGPS_ParseGSV(char* sentence_temp){return true;}
bool HLGPS_ParseGLL(char* sentence_temp){return true;}
#else
bool HLGPS_ParseRMC(char* sentence_temp)
{
	char ch = ',';
	HRString_CharSplit_For_GGA(sentence_temp,&ch);

	return true;
}

bool HLGPS_ParseVTG(char* sentence_temp)
{
	char ch = ',';
	HRString_CharSplit_For_GGA(sentence_temp,&ch);

	return true;
}

bool HLGPS_ParseGSV(char* sentence_temp)
{
	char ch = ',';
	HRString_CharSplit_For_GGA(sentence_temp,&ch);

	return true;
}

bool HLGPS_ParseGLL(char* sentence_temp)
{
	char ch = ',';
	HRString_CharSplit_For_GGA(sentence_temp,&ch);

	return true;
}

// TODO: Add dummy function for garbage messages
bool HLGPS_ParseSkip(char* sentence_temp)
{
	//char ch = ',';
	//printf("skipped\n");

	return true;
}

#endif

bool HLString_CharStartWith(const char *src,const char *ch)
{
	if(!strncmp(src,ch,strlen(ch)))
		return true;
	else
		return false;
}

void HLString_CharGetRight(char *src,int ptr)
{
	int strSize = strlen(src);
	if(strSize <= 0 || ptr < 0 || ptr > strSize)
		return;

	char temp[128];

	memset(temp,0,strSize);

	strncpy(temp,src+ptr+1,strSize);
	if(strlen(temp) > strSize)
		*(temp+strSize) = '\0';

	memset(src,0,strlen(src));
	strcpy(src,temp);
}

void HLString_CharGetLeft(char *src,int ptr)
{
	int strSize = strlen(src);
	if(strSize <= 0 || ptr < 0 || ptr > strSize)
		return;

	memset(src+ptr , 0, NMEA_MAX_SIZE-ptr);

}

bool HLGPS_VerifyChecksum(char *nmeaStr)
{
	unsigned char calculated_cksum = 0;
	unsigned char received_cksum = 0;
	char 	ch = '0';
	char 	token = '*';
	int 	size;

	size = strlen(nmeaStr);
	if( size > NMEA_MAX_SIZE)
		return false;

	char *digit = strrchr(nmeaStr, token);
	if(digit == NULL)
		return false;

	calculated_cksum = 0;
	for (int i = 0; i < size; i++)
	{
		ch = (char) *(nmeaStr+i);
		if(ch == '*')
			break;

		if(ch != '$')
		{
			calculated_cksum ^= ch;
		}
	}

	if(ch != '*')
		return false;


	received_cksum = (unsigned char) (strtol((digit+1), NULL, 16) & 0xff);
	//return true;

	for(int i=0;i<size;i++)
	{
		if(*(nmeaStr+i) == token )
		{
			HLString_CharGetLeft(nmeaStr,i);
			break;
		}
	}
	if(received_cksum == calculated_cksum)
		return true;
	else
		return false;
}


// TODO: for GPS uart debug
#define DBG_REQUEST_SIZE 200
#define DBG_BUF_SIZE 10
#define DBG_RX_SIZE 512*4

//uint32_t test_info[DBG_BUF_SIZE][6];
uint32_t test_gCount = 0;
//uint32_t test_memo[DBG_BUF_SIZE][6];
uint32_t test_sCount = 0;
uint32_t test_total_bytes = 0;
uint32_t test_zero_times = 0;

uint16_t remaining_bytes = 0;
uint16_t bytes_received = 0;
uint16_t total_bytes = 0;
//uint8_t temp_buf[DBG_RX_SIZE];
uint8_t* temp_current_pos;

// TODO: reduce stack size
char gps_data_temp[ONE_GPS_BUFFER_SIZE];
char gps_sentence[NMEA_MAX_SIZE];

void HLGPS_StartToParseNMEA(void)
{
    char* ptr = gps_sentence;
    static uint8_t err_count = 0;
    static uint8_t nmeaEnd = 0;

    // TODO: return if there is no data received
    if(m_GPSSize == 0)
    {
    	test_zero_times++;
    	if(test_zero_times > 1000) test_zero_times = 0;
    	return;
    }

    int prefix_size = sizeof(NMEA_PREFIX) / sizeof(NMEA_PREFIX[0]);

    //Clear buffer
    memset(gps_data_temp,0,ONE_GPS_BUFFER_SIZE);
    memset(gps_sentence,0,sizeof(gps_sentence));

    //Copy GPS raw data
    memcpy(gps_data_temp, m_pGPS, m_GPSSize);

#if 1
	test_gCount++;
	test_zero_times = 0;

	if(test_gCount==DBG_BUF_SIZE*1000*100)
		while(1);

	// Just return for verification
	// return;
#endif
    //Only for debug purpose
#if 0
    char test_gps_sentence[128] = "$GPGGA,142407.010,2502.69955,N,12132.74165,E,1,04,1.9,-04.81,M,15.2,M,,*72\r\n";
    //char test_gps_sentence[128] = "$GPGSA,A,3,20,32,31,14,,,,,,,,,5.4,1.9,5.0*3A\r\n";
    m_GPSSize = strlen(test_gps_sentence);
    memcpy(gps_data_temp,test_gps_sentence,m_GPSSize);
#endif

    //I try to combine NMEA sentence as completely sentence if I have fragment sentence before
    if(strlen(m_fragment))
    {
    	//printf("--> frmgt size=%d\r\n", strlen(m_fragment));
    	strcpy(gps_sentence,m_fragment);
    	ptr = gps_sentence + strlen(gps_sentence);
    }

    //Handle for new coming sentence
    for(int i = 0;i < m_GPSSize;i++)
    {
    	if(*(gps_data_temp+i) == '\n')
    		nmeaEnd = 1;

    	strncpy(ptr++,gps_data_temp+i,1);

    	//Now,NMEA is ready
    	if(nmeaEnd == 1)
    	{
    		if(*(--ptr - 1) == '\r')
    			ptr--;
    		*ptr = '\0';
    		//printf("gs=%d\r\n", strlen(gps_sentence));
#if 1
    		if(!HLGPS_VerifyChecksum(gps_sentence))
	    			err_count++;
	    	else
#endif
	    	{
    			for(int index = 0;index < prefix_size; index++)
    			{
    				// TODO: Check the GPS TAG comparison
    				if(HLString_CharStartWith(gps_sentence,NMEA_PREFIX[index]))
    				{
    					(*(fp_GPS+index))(gps_sentence);
    					//printf("=\n");
    					break;
    				}
    			}
    		}

    		//Clear string buffer
    		memset(m_fragment,0,sizeof(m_fragment));
    		memset(gps_sentence,0,sizeof(gps_sentence));

    		ptr = gps_sentence;

    		nmeaEnd = 0;
    	}
    }
    strcpy(m_fragment,gps_sentence);

    return;
}



// TODO: for GPS uart modification

extern uint8_t buf_index;
extern uart4_get_rxXferCount(uint16_t*);
extern void uart4_rx_it(uint8_t buf_id, uint16_t size);
extern void uart4_abort_rx_it(void);

uint8_t* Read_GPS_To_GPSBuffer(uint32_t* data_size)
{
#if 0
	taskENTER_CRITICAL();

	if(gps_data_point != 0)
	{
		*data_size = gps_data_point;
		gps_data_point = 0;
		if(gps_buffer_base == buffer_num_1)
		{
			gps_buffer_base = buffer_num_2;
			taskEXIT_CRITICAL();
			return (uint8_t*)&gps_buffer;

		}
		else
		{
			gps_buffer_base = buffer_num_1;
			taskEXIT_CRITICAL();
			return (uint8_t*)&gps_buffer + (gps_buffer_size / 2);
		}
	}
	else
	{
		*data_size = 0;
		taskEXIT_CRITICAL();
		return (uint8_t*)&gps_buffer;
	}
#else

	// Code replacement of gps buffer management

	// TODO: Need to check if the zero size check and critical session is
	// necessary.
	taskENTER_CRITICAL();

	/* Enquire remaining bytes in this UART Rx session */
	uart4_get_rxXferCount(&remaining_bytes);

	/* If the remaining bytes below the water mark, swap buf_index */
	bytes_received = GPS_BUFFER_SIZE - remaining_bytes;
	if( bytes_received != 0 )
	{

		if(buf_index == 0)
			buf_index = 1;
		else
			buf_index = 0;

		// FIXME: abort RX IT will discard the data kept in UART FIFO
		uart4_abort_rx_it();

		// Designate the new block of gps_buffer to receive GPS UART output
		uart4_rx_it(buf_index, GPS_BUFFER_SIZE);
	}
	taskEXIT_CRITICAL();

	*data_size = bytes_received;
	return (uint8_t*) &gps_buffer[buf_index][0];
#endif
}


void HLGPS_GetDataFromDriver(void)
{
	m_pGPS = Read_GPS_To_GPSBuffer(&m_GPSSize);

	// TODO: Check point of UART4 RX interface
	{
		static int32_t i = 0;

		if( m_GPSSize!=0 )
		{
			printf("%4d |%4d\r\n", i, m_GPSSize);
			//printf("a\n");
			if( ++i > 300000 ) i = 0;
		}

	}

	return;
}
