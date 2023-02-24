#include "printing.h"
#include <stdio.h>

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
