/*
 * spi_pressure_snsr_api.c
 *
 *  Created on: April 30, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "spi_pressure_snsr_api.h"

#define PSnsrCSADCOut_Pin GPIO_PIN_6
#define PSnsrCSADCOut_GPIO_Port GPIOD
#define PSnsrCSEEOut_Pin GPIO_PIN_7
#define PSnsrCSEEOut_GPIO_Port GPIOD

void SPIPrsrSnsrInit(SPIPressureSensor_S *sensor_, SPI_HandleTypeDef *spi_)
{
	sensor_->spi_handle = spi_;
	HAL_GPIO_WritePin(PSnsrCSADCOut_GPIO_Port, PSnsrCSADCOut_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSnsrCSEEOut_GPIO_Port, PSnsrCSEEOut_Pin, GPIO_PIN_SET);
}

void SPSReadEEPROMCfg(SPIPressureSensor_S *sensor_)
{
	/* TODO: Read Pressure Range Bytes */
	/* TODO: Read Pressure Minimum Bytes */
	/* TODO: Read Pressure Units Bytes */
	/* TODO: Read Offset Coefficients Bytes */
	/* TODO: Read Span Coefficients Bytes */
	/* TODO: Read Shape Coefficients Bytes */
}

void SPSInitSnsrADC(SPIPressureSensor_S *sensor_)
{
	/* TODO: Configure ADC default register values */
}

void SPSSnsrADCTempRead(SPIPressureSensor_S *sensor_)
{
	/* TODO: Set ADC to temperature reading */ // 0000 0110
	/* TODO: Read Temperature */
	/* TODO: Compensate Temperature */
}

void SPSSnsrADCPressureRead(SPIPressureSensor_S *sensor_)
{

}

void SPSCompPrsrReading(SPIPressureSensor_S *sensor_)
{

}
