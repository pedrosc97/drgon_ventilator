/*
 * ventilator_api.h
 *
 *  Created on: 17.04.2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#ifndef INC_VENTILATOR_API_H_
#define INC_VENTILATOR_API_H_

#include "stm32f4xx_hal.h"
#include "potentiometer_api.h"

// Macro Definitions ////////////////////////////////////////

#define ADC_MAX_VAL						4096
#define PWM_PERIOD_VALUE				4200.0f
#define ARM_SWEEP_ANGLE					21.6f

#define DEFAULT_INITIAL_ANGLE			0.0f

#define TIDAL_VOLUME_INCREMENT			1.0f
#define TIDAL_VOLUME_MIN_VAL			0.0f
#define TIDAL_VOLUME_MAX_VAL			800.0f

#define PRESSURE_LEVEL_ALARM_INCREMENT	5.0f
#define PRESSURE_LEVEL_ALARM_MIN_VAL	0.0f
#define PRESSURE_LEVEL_ALARM_MAX_VAL	80.0f

#define RESPIRATION_FREQ_STEP			1.0f
#define RESPIRATION_FREQ_MIN_VAL		8.0f
#define RESPIRATION_FREQ_MAX_VAL		40.0f

// Ventilator Parameters Typedefs /////////////////////////////

typedef uint8_t 	RespirationFrequency_T;
typedef uint8_t 	PressureValue_T;
typedef uint16_t 	TidalVolume_T;

// I:E Ratio Enumeration Values ///////////////////////////////

typedef enum VentilatorIERatio_E
{
	I_E_RATIO_1_1 = 1,
	I_E_RATIO_1_2 = 2,
	I_E_RATIO_1_3 = 3,
	I_E_RATIO_1_4 = 4,
} VentilatorIERatio_E;

// Alarms Configuration Enumeration ////////////////////////////

typedef enum VentilatorAlarmsConfig_E
{
	REPORT_ALARMS = 0,
	SILENCE_ALARMS = 1,
} VentilatorAlarmsConfig_E;

// Alarms Enumeration ////////////////////////////

typedef enum VentilatorAlarms_E
{
	NO_ALARMS = 0,
	ALARM_1 = 1,
	ALARM_2 = 2,
	ALARM_3 = 3,
	ALARM_4 = 4,
	ALARM_5 = 5,
	ALARM_6 = 6,
	ALARM_7 = 7,d
} VentilatorAlarms_E;

// State Machine States //////////////////////////////////////
// First Level

typedef enum VentilatorMainStateMachine_E
{
	MAIN_STATE_STANDBY = 0,
	MAIN_STATE_SETUP = 1,
	MAIN_STATE_RESPIRATION = 2,
} VentilatorMainStateMachine_E;

// Second Level

typedef enum VentilatorSetupStateMachine_E
{
	SETUP_STATE_STANDBY = 0,
	SETUP_STATE_CALIBRATION = 1,
	SETUP_STATE_MANUAL_SPIN = 2,
} VentilatorSetupStateMachine_E;

typedef enum VentilatorRespirationStateMachine_E
{
	RESPIRATION_STATE_STANDBY = 0,
	RESPIRATION_STATE_FORCED_VOLUME = 1,
	RESPIRATION_STATE_ASSISTED_VOLUME = 2,
} VentilatorRespirationStateMachine_E;

// Third Level

typedef enum VentilatorCalibrationStateMachine_E
{
	CALIBRATION_STATE_UNWIND = 0,
	CALIBRATION_STATE_WAIT_AMBU = 1,
	CALIBRATION_STATE_WIND = 2,
	CALIBRATION_STATE_STOP = 3,
} VentilatorCalibrationStateMachine_E;

typedef enum VentilatorManualSpinStateMachine_E
{
	MANUAL_SPIN_STATE_STOP = 0,
	MANUAL_SPIN_STATE_UNWIND = 1,
	MANUAL_SPIN_STATE_WIND = 2,
} VentilatorManualSpinStateMachine_E;

typedef enum VentilatorForcedVolumeStateMachine_E
{
	FORCED_VOLUME_STATE_STANDBY = 0,
	FORCED_VOLUME_STATE_GENERATE_TRAJECTORY = 1,
	FORCED_VOLUME_STATE_INHALE = 2,
	FORCED_VOLUME_STATE_INHALE_PAUSE = 3,
	FORCED_VOLUME_STATE_EXHALE = 4,
	FORCED_VOLUME_STATE_EXHALE_PAUSE = 5,
} VentilatorForcedVolumeStateMachine_E;

typedef enum VentilatorAssistedVolumeStateMachine_E
{
	ASSISTED_VOLUME_STATE_STANDBY = 0,
	ASSISTED_VOLUME_STATE_GENERATE_TRAJECTORY = 1,
	ASSISTED_VOLUME_STATE_WAIT_FOR_RESPIRATION = 2,
	ASSISTED_VOLUME_STATE_INHALE = 3,
	ASSISTED_VOLUME_STATE_INHALE_PAUSE = 4,
	ASSISTED_VOLUME_STATE_EXHALE = 5,
	ASSISTED_VOLUME_STATE_EXHALE_PAUSE = 6,
} VentilatorAssistedVolumeStateMachine_E;

// Ventilator Main Struct /////////////////////////////////////////////

typedef struct Ventilator_S
{
	/* User Input Parameters Registers */
	VentilatorIERatio_E			i_e_ratio; 						// 1 to 4
	RespirationFrequency_T		respiration_frequency; 			// 8 to 40 Hz
	PressureValue_T				pressure_level_alarm_value; 	// 0 to 80 in 5 increments
	TidalVolume_T				tidal_volume; 					// 0 to 800 ml in one increment

	/* Respiration Parameters */
	uint16_t	respiration_period_ms;
	uint16_t	inhalation_period_ms;
	uint16_t	exhalation_period_ms;
	float		end_angle;

	/* State Machines */
	/* First Level State Machine */
	volatile VentilatorMainStateMachine_E				main_state_machine;

	/* Second Level State Machines */
	volatile VentilatorSetupStateMachine_E				setup_state_machine;
	volatile VentilatorRespirationStateMachine_E		respiration_state_machine;

	/* Third Level State Machines: Setup */
	volatile VentilatorCalibrationStateMachine_E		calibration_state_machine;
	volatile VentilatorManualSpinStateMachine_E			manual_spin_state_machine;

	/* Third Level State Machines: Respiration */
	volatile VentilatorForcedVolumeStateMachine_E		forced_volume_state_machine;
	volatile VentilatorAssistedVolumeStateMachine_E		assisted_volume_state_machine;

	volatile uint16_t cycle_counter;
	volatile uint8_t  parameters_changed;

	/* Alarms */
	volatile VentilatorAlarms_E			ventilator_alarms;
	volatile VentilatorAlarmsConfig_E	ventilator_alarms_config;

	volatile uint32_t	prev_systick;

} Ventilator_S;

// Ventilator Function Prototypes /////////////////////////////////////

void VentilatorInit(Ventilator_S *ventilator_);
void UpdateVentilatorParams(Ventilator_S *ventilator_, Potentiometer_S *potentiometer_);

#endif /* INC_VENTILATOR_API_H_ */
