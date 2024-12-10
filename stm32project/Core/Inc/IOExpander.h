/**
 * @file IOExpander.c
 * @author Aliou LY (aliou.ly@ensea.fr)
 * @brief 
 * @version 0.1
 * @date 2024-12-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdint.h>
#ifndef __IOExpander__
#define __IOExpander__

#define IODIRA 0x00
#define IODIRB 0x01
#define IOCON 0x0A

#define DEVICEADD 0x40

typedef enum
{
    PortA=0x12,
    PortB=0x13,
} IOExpanderGPIO_Port_t;

typedef enum

{
    Pin0=0,
    Pin1,
    Pin2,
    Pin3,
    Pin4,
    Pin5,
    Pin6,
    Pin7,
} IOExpanderGPIO_Pin_t;

uint8_t IOExpanderInit(void) ;
uint8_t IOExpanderWrite(uint8_t reg, uint8_t data) ;
uint8_t IOExpanderGPIO_WritePin(IOExpanderGPIO_Port_t GPIO_Port,IOExpanderGPIO_Pin_t GPIO_Pin,uint8_t level ) ;
uint8_t IOExpanderGPIO_TogglePin(IOExpanderGPIO_Port_t GPIO_Port,IOExpanderGPIO_Pin_t GPIO_Pin) ;

#endif
