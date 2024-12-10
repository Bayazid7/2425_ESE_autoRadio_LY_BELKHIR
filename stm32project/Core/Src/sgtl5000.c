#include "sgtl5000.h"
#include "i2c.h"


static uint8_t writeRegister(uint16_t reg, uint16_t data)

{
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF;
    buffer[1] = data & 0xFF;
    if (HAL_I2C_Mem_Write(&hi2c2, SGTL_ADESSE, reg, I2C_MEMADD_SIZE_16BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

static uint8_t readRegister(uint16_t reg, uint16_t *data)
{
    uint8_t buffer[2];
    if (HAL_I2C_Mem_Read(&hi2c2, SGTL_ADESSE, reg, I2C_MEMADD_SIZE_16BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return 1;
    }
    *data = (buffer[0] << 8) | buffer[1];
    return 0;
}


void modify(uint16_t usRegister , uint16_t usClearMask , uint16_t usSetValue)
{
	uint16_t usData;
	readRegister(usRegister,&usData);
	usData = usData & usClearMask;
	usData = usData | usSetValue;
	writeRegister(usRegister, usData);
}

void sgtl5000_Init(void)
{
	    writeRegister(CHIP_ANA_POWER, 0x6AFF);
	    writeRegister(CHIP_LINREG_CTRL, 0x006C);
	    writeRegister(CHIP_REF_CTRL, 0x01FF);
	    writeRegister(CHIP_LINE_OUT_CTRL, 0x031E);
	    writeRegister(CHIP_SHORT_CTRL, 0x1106);
	    writeRegister(CHIP_ANA_CTRL, 0x0004);
	    writeRegister(CHIP_DIG_POWER, 0x0073);
	    writeRegister(CHIP_LINE_OUT_VOL, 0x1f1f);
	    writeRegister(CHIP_CLK_CTRL, 0x0004);
	    writeRegister(CHIP_I2S_CTRL, 0x0130);
	    writeRegister(CHIP_ADCDAC_CTRL, 0x0000);
	    writeRegister(CHIP_DAC_VOL, 0x3C3C);
	    writeRegister(CHIP_SSS_CTRL , 0x0151);
}




