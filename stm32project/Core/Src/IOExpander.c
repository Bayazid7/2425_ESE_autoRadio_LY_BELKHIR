#include <stdio.h>
#include "IOExpander.h"
#include "spi.h"
#include "gpio.h"

uint8_t IOExpanderWrite(uint8_t reg, uint8_t data)
{

	uint8_t txBuffer[3];
	txBuffer[0] = (DEVICEADD & 0xFE);
	txBuffer[1] = reg;
	txBuffer[2] = data;

	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi3, txBuffer, 3, HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
		return HAL_ERROR;
	}
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
	return HAL_OK;
}

static uint8_t IOExpanderRead(uint8_t reg, uint8_t *data) {
	uint8_t txBuffer[3];
	txBuffer[0] = (DEVICEADD | 0x01);
	txBuffer[1] = reg;
	txBuffer[2] = 0;


	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi3, txBuffer, 3, HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
		return HAL_ERROR;
	}
	if (HAL_SPI_Receive(&hspi3, data, 1, HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
		return HAL_ERROR;
	}
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
	return HAL_OK;
}

uint8_t IOExpanderInit(void) {
	HAL_GPIO_WritePin(VU_NRST_GPIO_Port, VU_NRST_Pin, GPIO_PIN_SET);
	uint8_t config = 0x00;

	if (IOExpanderWrite(IODIRA, config) != HAL_OK) {
		printf("Erreur : Configuration IODIRA\n");
		return HAL_ERROR;
	}

	if (IOExpanderWrite(IODIRB, config) != HAL_OK) {
		printf("Erreur : Configuration IODIRB\n");
		return HAL_ERROR;
	}
	IOExpanderWrite(PortA, 0xff) ;
	IOExpanderWrite(PortB, 0xff);
	printf("Tous les GPIO configurés en mode sortie.\n");
	return HAL_OK;
}

uint8_t IOExpanderGPIO_WritePin(uint8_t GPIO_Port, uint8_t GPIO_Pin, uint8_t level) {
	uint8_t currentData;

	if (IOExpanderRead(GPIO_Port, &currentData) != HAL_OK) {
		return HAL_ERROR;
	}

	if (level) {
		currentData |= (1 << GPIO_Pin);
	} else {
		currentData &= ~(1 << GPIO_Pin);
	}

	return IOExpanderWrite(GPIO_Port, currentData);
}

uint8_t IOExpanderGPIO_TogglePin(IOExpanderGPIO_Port_t GPIO_Port, IOExpanderGPIO_Pin_t GPIO_Pin) {
	uint8_t currentData;

	if (IOExpanderRead(GPIO_Port, &currentData) != HAL_OK) {
		return HAL_ERROR;
	}

	if (currentData & (1 << GPIO_Pin)) {
		currentData &= ~(1 << GPIO_Pin);
	} else {
		currentData |= (1 << GPIO_Pin);
	}

	return IOExpanderWrite(GPIO_Port, currentData);
}
