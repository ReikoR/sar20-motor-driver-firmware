#ifndef INC_DRV8323_H_
#define INC_DRV8323_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

typedef enum DRV8301_RegisterAddress {
  DRV8323_RegisterAddress_fault_status_1 = 0b0000,
  DRV8323_RegisterAddress_Vgs_Status_2 = 0b0001 << 11,
  DRV8323_RegisterAddress_driver_control = 0b0010 << 11,
  DRV8323_RegisterAddress_gate_drive_HS = 0b0011 << 11,
  DRV8323_RegisterAddress_gate_drive_LS = 0b0100 << 11,
  DRV8323_RegisterAddress_OCP_control = 0b0101 << 11,
  DRV8323_RegisterAddress_CSA_control = 0b0110 << 11
} DRV8323_RegisterAddress;

enum DRV8323_driver_control_charge_pump_UVLO {
  DRV8323_driver_control_charge_pump_UVLO_enabled = 0, // default
  DRV8323_driver_control_charge_pump_UVLO_disabled = 1 << 9
};

enum DRV8323_driver_control_gate_drive_fault {
  DRV8323_driver_control_gate_drive_fault_enabled = 0, // default
  DRV8323_driver_control_gate_drive_fault_disabled = 1 << 8
};

enum DRV8323_driver_control_OTW_reporting {
  DRV8323_driver_control_OTW_reporting_disabled = 0, // default
  DRV8323_driver_control_OTW_reporting_enabled = 1 << 7
};

enum DRV8323_driver_control_PWM_mode {
  DRV8323_driver_control_PWM_mode_6x = 0, // default
  DRV8323_driver_control_PWM_mode_3x = 0b01 << 5,
  DRV8323_driver_control_PWM_mode_1x = 0b10 << 5,
  DRV8323_driver_control_PWM_mode_independent = 0b11 << 5
};

enum DRV8323_driver_control_1x_PWM_mode_commutation {
  DRV8323_driver_control_1x_PWM_mode_commutation_sync_rectification = 0, // default
  DRV8323_driver_control_1x_PWM_mode_commutation_async_rectification = 1 << 4
};

enum DRV8323_driver_control_1x_PWM_mode_direction {
  DRV8323_driver_control_1x_PWM_mode_direction_0 = 0, // default
  DRV8323_driver_control_1x_PWM_mode_direction_1 = 1 << 3
};

enum DRV8323_driver_control_1x_PWM_mode_coast {
  DRV8323_driver_control_1x_PWM_mode_coast_disabled = 0, // default
  DRV8323_driver_control_1x_PWM_mode_coast_enabled = 1 << 2
};

enum DRV8323_driver_control_1x_PWM_mode_brake {
  DRV8323_driver_control_1x_PWM_mode_brake_disabled = 0, // default
  DRV8323_driver_control_1x_PWM_mode_brake_enabled = 1 << 1
};

enum DRV8323_driver_control_clear_fault {
  DRV8323_driver_control_clear_fault_disabled = 0, // default
  DRV8323_driver_control_clear_fault_enabled = 1
};

enum DRV8323_gate_drive_HS_lock {
  DRV8323_gate_drive_HS_lock_disabled = 0b011 << 8, // default
  DRV8323_gate_drive_HS_lock_enabled = 0b110 << 8
};

enum DRV8323_gate_drive_IDRIVEP {
  DRV8323_gate_drive_IDRIVEP_10mA = 0,
  DRV8323_gate_drive_IDRIVEP_30mA = 0b0001 << 4,
  DRV8323_gate_drive_IDRIVEP_60mA = 0b0010 << 4,
  DRV8323_gate_drive_IDRIVEP_80mA = 0b0011 << 4,
  DRV8323_gate_drive_IDRIVEP_120mA = 0b0100 << 4,
  DRV8323_gate_drive_IDRIVEP_140mA = 0b0101 << 4,
  DRV8323_gate_drive_IDRIVEP_170mA = 0b0110 << 4,
  DRV8323_gate_drive_IDRIVEP_190mA = 0b0111 << 4,
  DRV8323_gate_drive_IDRIVEP_260mA = 0b1000 << 4,
  DRV8323_gate_drive_IDRIVEP_330mA = 0b1001 << 4,
  DRV8323_gate_drive_IDRIVEP_370mA = 0b1010 << 4,
  DRV8323_gate_drive_IDRIVEP_440mA = 0b1011 << 4,
  DRV8323_gate_drive_IDRIVEP_570mA = 0b1100 << 4,
  DRV8323_gate_drive_IDRIVEP_680mA = 0b1101 << 4,
  DRV8323_gate_drive_IDRIVEP_820mA = 0b1110 << 4,
  DRV8323_gate_drive_IDRIVEP_1000mA = 0b111 << 4, // default
};

enum DRV8323_gate_drive_IDRIVEN {
  DRV8323_gate_drive_IDRIVEN_20mA = 0,
  DRV8323_gate_drive_IDRIVEN_60mA = 0b0001,
  DRV8323_gate_drive_IDRIVEN_120mA = 0b0010,
  DRV8323_gate_drive_IDRIVEN_160mA = 0b0011,
  DRV8323_gate_drive_IDRIVEN_240mA = 0b0100,
  DRV8323_gate_drive_IDRIVEN_280mA = 0b0101,
  DRV8323_gate_drive_IDRIVEN_340mA = 0b0110,
  DRV8323_gate_drive_IDRIVEN_380mA = 0b0111,
  DRV8323_gate_drive_IDRIVEN_520mA = 0b1000,
  DRV8323_gate_drive_IDRIVEN_660mA = 0b1001,
  DRV8323_gate_drive_IDRIVEN_740mA = 0b1010,
  DRV8323_gate_drive_IDRIVEN_880mA = 0b1011,
  DRV8323_gate_drive_IDRIVEN_1140mA = 0b1100,
  DRV8323_gate_drive_IDRIVEN_1360mA = 0b1101,
  DRV8323_gate_drive_IDRIVEN_1640mA = 0b1110,
  DRV8323_gate_drive_IDRIVEN_2000mA = 0b111, // default
};

enum DRV8323_driver_gate_drive_LS_IDRIVEN_OCP_off_time {
  DRV8323_gate_drive_IDRIVEN_OCP_cycle_by_cycle_disabled = 0,
  DRV8323_gate_drive_IDRIVEN_OCP_cycle_by_cycle_enabled = 1 << 10 // default
};

enum DRV8323_driver_gate_drive_LS_IDRIVEN_Tdrive {
  DRV8323_gate_drive_IDRIVEN_Tdrive_500ns = 0,
  DRV8323_gate_drive_IDRIVEN_Tdrive_1000ns = 0b01 << 8,
  DRV8323_gate_drive_IDRIVEN_Tdrive_2000ns = 0b10 << 8,
  DRV8323_gate_drive_IDRIVEN_Tdrive_4000ns = 0b11 << 8 // default
};

enum DRV8323_OCP_control_Tretry {
  DRV8323_OCP_control_Tretry_4ms = 0, // default
  DRV8323_OCP_control_Tretry_50us = 1 << 10
};

enum DRV8323_OCP_control_dead_time {
  DRV8323_OCP_control_dead_time_50ns = 0,
  DRV8323_OCP_control_dead_time_100ns = 0b01 << 8, // default
  DRV8323_OCP_control_dead_time_200ns = 0b10 << 8,
  DRV8323_OCP_control_dead_time_400ns = 0b11 << 8
};

enum DRV8323_OCP_control_mode {
  DRV8323_OCP_control_mode_latched_fault = 0,
  DRV8323_OCP_control_mode_retrying_fault = 0b01 << 6, // default
  DRV8323_OCP_control_mode_report_only = 0b10 << 6,
  DRV8323_OCP_control_mode_ignore = 0b11 << 6
};

enum DRV8323_OCP_control_deglitch_time {
  DRV8323_OCP_control_deglitch_time_2us = 0,
  DRV8323_OCP_control_deglitch_time_4us = 0b01 << 4, // default
  DRV8323_OCP_control_deglitch_time_6us = 0b10 << 4,
  DRV8323_OCP_control_deglitch_time_8us = 0b11 << 4
};

enum DRV8323_OCP_control_Vds_level {
  DRV8323_OCP_control_Vds_level_0_06V = 0,
  DRV8323_OCP_control_Vds_level_0_13V = 0b0001,
  DRV8323_OCP_control_Vds_level_0_2V = 0b0010,
  DRV8323_OCP_control_Vds_level_0_26V = 0b0011,
  DRV8323_OCP_control_Vds_level_0_31V = 0b0100,
  DRV8323_OCP_control_Vds_level_0_45V = 0b0101,
  DRV8323_OCP_control_Vds_level_0_53V = 0b0110,
  DRV8323_OCP_control_Vds_level_0_6V = 0b0111,
  DRV8323_OCP_control_Vds_level_0_68V = 0b1000,
  DRV8323_OCP_control_Vds_level_0_75V = 0b1001, // default
  DRV8323_OCP_control_Vds_level_0_94V = 0b1010,
  DRV8323_OCP_control_Vds_level_1_13V = 0b1011,
  DRV8323_OCP_control_Vds_level_1_3V = 0b1100,
  DRV8323_OCP_control_Vds_level_1_5V = 0b1101,
  DRV8323_OCP_control_Vds_level_1_7V = 0b1110,
  DRV8323_OCP_control_Vds_level_1_88V = 0b111,
};

enum DRV8323_CSA_control_FET {
  DRV8323_CSA_control_positive_input_SPx = 0, // default
  DRV8323_CSA_control_positive_input_SHx = 1 << 10
};

enum DRV8323_CSA_control_VREF_DIV {
  DRV8323_CSA_control_unidirectional_mode = 0,
  DRV8323_CSA_control_bidirectional_mode = 1 << 9 //default
};

enum DRV8323_CSA_control_LS_REF {
  DRV8323_CSA_control_VDS_OCP_across_SHx_to_SPx = 0, // default
  DRV8323_CSA_control_VDS_OCP_across_SHx_to_SNx = 1 << 8
};

enum DRV8323_CSA_control_gain {
  DRV8323_CSA_control_gain_5,
  DRV8323_CSA_control_gain_10 = 0b01 << 6,
  DRV8323_CSA_control_gain_20 = 0b10 << 6, // default
  DRV8323_CSA_control_gain_40 = 0b11 << 6,
};

enum DRV8323_CSA_control_sense_overcurrent {
  DRV8323_CSA_control_sense_overcurrent_enabled = 0, // default
  DRV8323_CSA_control_sense_overcurrent_disabled = 1 << 5
};

enum DRV8323_CSA_control_CAL_A {
  DRV8323_CSA_control_CAL_A_disabled = 0, // default
  DRV8323_CSA_control_CAL_A_enabled = 1 << 4
};

enum DRV8323_CSA_control_CAL_B {
  DRV8323_CSA_control_CAL_B_disabled = 0, // default
  DRV8323_CSA_control_CAL_B_enabled = 1 << 3
};

enum DRV8323_CSA_control_CAL_C {
  DRV8323_CSA_control_CAL_C_disabled = 0, // default
  DRV8323_CSA_control_CAL_C_enabled = 1 << 2
};

enum DRV8323_CSA_control_sense_overcurrent_level {
  DRV8323_CSA_control_sense_overcurrent_level_0_25V = 0,
  DRV8323_CSA_control_sense_overcurrent_level_0_5V = 0b01,
  DRV8323_CSA_control_sense_overcurrent_level_0_75V = 0b10,
  DRV8323_CSA_control_sense_overcurrent_level_1V = 0b11, // default
};


uint16_t DRV8323_Write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, uint16_t value);

uint16_t DRV8323_readRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, DRV8323_RegisterAddress address);

void DRV8323_writeRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, DRV8323_RegisterAddress address, uint16_t value);

#endif
