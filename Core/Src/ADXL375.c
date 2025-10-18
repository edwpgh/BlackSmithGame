/*
 * ADXL375.c
 *
 *  Created on: Apr 8, 2024
 *      Author: HP
 */


#include "ADXL375.h"


/*
 * INITIALISATION
 */

int ADXL375_Initialise(ADXL375 *dev, SPI_HandleTypeDef *spiHandle)
{
	ToggleCSHigh();

	dev->spiHandle = spiHandle;

	HAL_StatusTypeDef status;

	int errorNum = 0;

	/* confirm device address as 0xE5 */
	uint8_t confirmID;
	while(1) {
		status = ReadData(dev, ADXL375_DEVID, &confirmID, 1);
		if (status == HAL_OK) {
			if (confirmID == 0xE5)
				break;
		}
	}

	/* set the required data rate */
	status = WriteData(dev, ADXL375_BW_RATE, BW_RATE_DATA_RATE, 1);
	if (status != HAL_OK) {
		errorNum++;
	}

	/* if all well till now, enable measurement mode */
	status = WriteData(dev, ADXL375_POWER_CTL, POWER_CTL_MEASURE_MODE, 1);
	if (status != HAL_OK)
		errorNum++;

	return errorNum;
}


/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef ADXL375_ReadAcceleration(ADXL375 *dev)
{
	HAL_StatusTypeDef status = ReadData(dev, ADXL375_DATAX0, dev->rawAccData, 6);
	return status;
}

void ADXL375_CleanRawValues(ADXL375 *dev)
{
	int16_t val;

	/* CONVERSION FOR ACC_X */
	val = dev->rawAccData[1];
	val = (val << 8) + (dev->rawAccData[0]);
	if (val & 0x8000) {
		val = -((~val & 0xFFFF) + 1);
	}

	dev->accData[0] = (float)val * 0.049f;

	/* CONVERSION FOR ACC_Y */
	val = dev->rawAccData[3];
	val = (val << 8) + (dev->rawAccData[2]);
	if (val & 0x8000) {
		val = -((~val & 0xFFFF) + 1);
	}
	dev->accData[1] = (float)val * 0.049f ;

	/* CONVERSION FOR ACC_Z */
	val = dev->rawAccData[5];
	val = (val << 8) + (dev->rawAccData[4]);
	if (val & 0x8000) {
		val = -((~val & 0xFFFF) + 1);
	}
	dev->accData[2] = (float)val * 0.049f ;
}




/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef WriteData(ADXL375 *dev, uint8_t address, uint8_t data, uint16_t len)
{
	ToggleCSLow();

	HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->spiHandle, &address, 1, HAL_MAX_DELAY);
	if (status == HAL_OK)
		status = HAL_SPI_Transmit(dev->spiHandle, &data, 1, HAL_MAX_DELAY);

	ToggleCSHigh();

	return status;
}

HAL_StatusTypeDef ReadData(ADXL375 *dev, uint8_t address, uint8_t *data, uint16_t len)
{
	ToggleCSLow();

	uint8_t txBuffer = (address | 0x80);

	if (len > 1) {
		txBuffer = (txBuffer | 0xC0);
	}

	HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->spiHandle, &txBuffer, 1, HAL_MAX_DELAY);
	HAL_Delay(3);
	if (status == HAL_OK)
		status = HAL_SPI_Receive(dev->spiHandle, data, len, HAL_MAX_DELAY);

	ToggleCSHigh();

	return status;
}

void ToggleCSHigh()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void ToggleCSLow()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}
////// shock
void ADXL375_EnableShockDetection(ADXL375 *dev, uint8_t axes_mask, uint8_t thresh, uint8_t dur)
{
	WriteData(dev, ADXL375_THRESH_SHOCK, thresh, 1);
		WriteData(dev, ADXL375_SHOCK_DURATION, dur, 1);
		WriteData(dev, ADXL375_SHOCK_AXES, axes_mask, 1);
		WriteData(dev, ADXL375_INT_ENABLE, 0x40, 1);
}

/////////////// FIFO_ENABLE
void ADXL375_EnableFIFO (ADXL375 *dev , uint8_t mode , uint8_t sampels )
{
	uint8_t FIFO_CTL = (mode << 6) | (sampels & 0x1F);
	WriteData(dev , ADXL375_FIFO_CTL , FIFO_CTL ,1 );

}
uint32_t ADXL375_Read_peak_from_100(ADXL375 *dev)
{
	HAL_StatusTypeDef status;
	uint32_t peak = 0, val = 0;

	ADXL375_CleanRawValues(&dev);

	for(uint8_t i=0; i<100; i++)
	{
		uint8_t FIFOraw[6];
		status = ReadData(dev, ADXL375_DATAX0, FIFOraw, 6);
		if(status != HAL_OK) return status;

		int16_t X = (int16_t)((FIFOraw[1] << 8)|FIFOraw[0]);
		int16_t Y = (int16_t)((FIFOraw[3] << 8)|FIFOraw[2]);
		int16_t Z = (int16_t)((FIFOraw[5] << 8)|FIFOraw[4]);

		val = abs((int32_t)X) + abs((int32_t)Y) + abs((int32_t)Z);
		if(val > peak)
			peak = val;
	}

	return peak;
}


_Bool ADXL375_CheckShock(ADXL375 *dev, uint8_t *out_act_shock_status)
{
	uint8_t int_source;
	if(ReadData(dev, ADXL375_INT_SOURCE, &int_source, 1) != HAL_OK)
		return 0;

	if(int_source & 0x40)
	{
		uint8_t act;
		ReadData(dev, ADXL375_ACT_SHOCK_STAT, &act, 1);
		if(out_act_shock_status) *out_act_shock_status = act;
		return 1;
	}

	return 0;
}
