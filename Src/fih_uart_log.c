#include "stm32l4xx_hal.h"
#include "stdio.h"


extern UART_HandleTypeDef huart3;

int _write (int fd, char *ptr, int len)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, 0xFF);

    if (status != HAL_OK)
        return -1;

    return len;
}
