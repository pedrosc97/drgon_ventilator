/*
 * spi_pressure_snsr_api.h
 *
 *  Created on: May 1, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#ifndef INC_SPI_PRESSURE_SNSR_API_H_
#define INC_SPI_PRESSURE_SNSR_API_H_

#include "stm32f4xx_hal.h"

// Commands for SPI Communication

#define EAD_EEPROM_CMD		0x03

/*
 * Steps:
 * 		1 Read ADC Settings and compensation values from EEPROM
 * 		2 Initialize ADC Converter with settings from EEPROM
 * 		3 Adjust sample rate if desired
 * 		4 Command ADC to take temp reading, store it (temp reading can be done every 100ms)
 * 		5 Delay (1/(SAMPLES_PER_SECOND))
 * 		6 Take a pressure reading, store it
 * 		7 Apply compensation formula
 * 		8 Repeat 4 to 6
 *
 * EEPROM:
 * 		BYTES		VALUE						Order		TYPE
 * 		0-15		Sensor Catalog Listing		MSB...LSB	ASCII CHAR
 * 		16-19		Serial number YYYY			MSB...LSB	ASCII CHAR
 * 		20-22		Serial number DDD			MSB...LSB	ASCII CHAR
 * 		23-26		Serial number XXXX			MSB...LSB	ASCII CHAR
 * 		27-30		Pressure range				LSB...MSB	Float
 * 		31-34 		Pressure minimum			LSB...MSB	Float
 * 		35-39		Pressure unit				MSB...LSB	ASCII CHAR
 * 		40			Pressure reference						ASCII CHAR
 * 		61			ADC_CONFIG_00							Unsigned char
 * 		63			ADC_CONFIG_01							Unsigned char
 * 		65			ADC_CONFIG_02							Unsigned char
 * 		67			ADC_CONFIG_03							Unsigned char
 * 		130-133		OffsetCoeff0				LSB...MSB	Float
 * 		134-137		OffsetCoeff1				LSB...MSB	Float
 * 		138-141		OffsetCoeff2				LSB...MSB	Float
 * 		142-145		OffsetCoeff3				LSB...MSB	Float
 * 		210-213		SpanCoeff0					LSB...MSB	Float
 * 		214-217		SpanCoeff1					LSB...MSB	Float
 * 		218-221		SpanCoeff2					LSB...MSB	Float
 * 		222-225		SpanCoeff3					LSB...MSB	Float
 * 		290-293		ShapeCoeff0					LSB...MSB	Float
 * 		294-297		ShapeCoeff1					LSB...MSB	Float
 * 		298-301		ShapeCoeff2					LSB...MSB	Float
 * 		302-305		ShapeCoeff3					LSB...MSB	Float
 * 		450-451		Checksum					LSB...MSB	Unsigned short int
 */

/* Memory Addresses */

#define PRESSURE_RANGE_BYTE_0		27u // LSB
#define PRESSURE_RANGE_BYTE_1		28u
#define PRESSURE_RANGE_BYTE_2		29u
#define PRESSURE_RANGE_BYTE_3		30u // MSB

#define PRESSURE_MINIMUM_BYTE_0		31u // LSB
#define PRESSURE_MINIMUM_BYTE_1		32u
#define PRESSURE_MINIMUM_BYTE_2		33u
#define PRESSURE_MINIMUM_BYTE_3		34u // MSB

#define PRESSURE_UNIT_BYTE_0		35u // LSB
#define PRESSURE_UNIT_BYTE_1		36u
#define PRESSURE_UNIT_BYTE_2		37u
#define PRESSURE_UNIT_BYTE_3		38u
#define PRESSURE_UNIT_BYTE_4		39u // MSB

/* ADC CONFIG COEFFICIENTS */

#define ADC_CONFIG_0				61u // LSB
#define ADC_CONFIG_1				63u
#define ADC_CONFIG_2				65u
#define ADC_CONFIG_3				67u // MSB

/* OFFSET COEFFICIENTS */

#define OFFSET_COEF_0_BYTE_0		130u
#define OFFSET_COEF_0_BYTE_1		131u
#define OFFSET_COEF_0_BYTE_2		132u
#define OFFSET_COEF_0_BYTE_3		133u

#define OFFSET_COEF_1_BYTE_0		134u
#define OFFSET_COEF_1_BYTE_1		135u
#define OFFSET_COEF_1_BYTE_2		136u
#define OFFSET_COEF_1_BYTE_3		137u

#define OFFSET_COEF_2_BYTE_0		138u
#define OFFSET_COEF_2_BYTE_1		139u
#define OFFSET_COEF_2_BYTE_2		140u
#define OFFSET_COEF_2_BYTE_3		141u

#define OFFSET_COEF_3_BYTE_0		142u
#define OFFSET_COEF_3_BYTE_1		143u
#define OFFSET_COEF_3_BYTE_2		144u
#define OFFSET_COEF_3_BYTE_3		145u

/* SPAN COEFFICIENTS */

#define SPAN_COEF_0_BYTE_0			210u
#define SPAN_COEF_0_BYTE_1			211u
#define SPAN_COEF_0_BYTE_2			212u
#define SPAN_COEF_0_BYTE_3			213u

#define SPAN_COEF_1_BYTE_0			214u
#define SPAN_COEF_1_BYTE_1			215u
#define SPAN_COEF_1_BYTE_2			216u
#define SPAN_COEF_1_BYTE_3			217u

#define SPAN_COEF_2_BYTE_0			218u
#define SPAN_COEF_2_BYTE_1			219u
#define SPAN_COEF_2_BYTE_2			220u
#define SPAN_COEF_2_BYTE_3			221u

#define SPAN_COEF_3_BYTE_0			222u
#define SPAN_COEF_3_BYTE_1			223u
#define SPAN_COEF_3_BYTE_2			224u
#define SPAN_COEF_3_BYTE_3			225u

/* SHAPE COEFFICIENTS */

#define SHAPE_COEF_0_BYTE_0			290u
#define SHAPE_COEF_0_BYTE_1			291u
#define SHAPE_COEF_0_BYTE_2			292u
#define SHAPE_COEF_0_BYTE_3			293u

#define SHAPE_COEF_1_BYTE_0			294u
#define SHAPE_COEF_1_BYTE_1			295u
#define SHAPE_COEF_1_BYTE_2			296u
#define SHAPE_COEF_1_BYTE_3			297u

#define SHAPE_COEF_2_BYTE_0			298u
#define SHAPE_COEF_2_BYTE_1			299u
#define SHAPE_COEF_2_BYTE_2			300u
#define SHAPE_COEF_2_BYTE_3			301u

#define SHAPE_COEF_3_BYTE_0			302u
#define SHAPE_COEF_3_BYTE_1			303u
#define SHAPE_COEF_3_BYTE_2			304u
#define SHAPE_COEF_3_BYTE_3			305u


typedef struct SPIPressureSensor_S
{
	ADC_HandleTypeDef	*adc_handle;
	float				current_pressure;
	float				current_temperature;
	float				pressure_range;
	float				pressure_offset;
	uint8_t				engineering_units[4];
	uint8_t				adc_config_coefs[4];
	float				offset_coefficients[4];
	float				span_coefficients[4];
	float				shape_coefficients[4];
} SPIPressureSensor_S;

void SPIPressureSensorInit(SPIPressureSensor_S *sensor_, ADC_HandleTypeDef *adc_);
void ReadEEPROM(uint16_t *address_);

#endif /* INC_SPI_PRESSURE_SNSR_API_H_ */
