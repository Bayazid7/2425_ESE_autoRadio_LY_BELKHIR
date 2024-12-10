/**
 * @file sgtl5000.c
 * @author Aliou LY (aliou.ly@ensea.fr)
 * @brief 
 * @version 0.1
 * @date 2024-12-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SGTL__
#define __SGTL__


#define SGTL_ADESSE 0x14
#define CHIP_ANA_POWER 0x0030
#define CHIP_LINREG_CTRL  0x0026
#define CHIP_REF_CTRL 0x0028
#define CHIP_LINE_OUT_CTRL 0x002C
#define CHIP_SHORT_CTRL 0x003C
#define CHIP_ANA_CTRL 0x0024
#define CHIP_DIG_POWER  0x0002
#define CHIP_LINE_OUT_VOL  0x002E
#define CHIP_CLK_CTRL  0x0004
#define CHIP_I2S_CTRL  0x0006
#define CHIP_ADCDAC_CTRL 0x000E
#define CHIP_DAC_VOL  0x0010
#define  CHIP_SSS_CTRL  0x000A

void sgtl5000_Init(void) ;

#endif

