#pragma once

#ifndef CPP_MAIN_H_
#define CPP_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

// includes
#include "main.h"
#include "fdcan.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// function declaration
int dxl_main();

#ifdef __cplusplus
}
#endif

#endif
