#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <stdint.h>
#include "device.h"
#include "driver_step_motor.h"

typedef struct {
	float x_dot;
	float theta1_dot;
	float theta2_dot;
	float x;
	float theta1;
	float theta2;
} lqr_state_t;

typedef enum {
	LQR_FAULT_NONE = 0,
	LQR_FAULT_NON_FINITE = 1u << 0,
	LQR_FAULT_STAGE_LIMIT = 1u << 1,
	LQR_FAULT_ANGLE_LIMIT = 1u << 2,
	LQR_FAULT_FORCE_LIMIT = 1u << 3,
	LQR_FAULT_SENSOR_TIMEOUT = 1u << 4
} lqr_fault_t;

typedef enum {
	LQR_MODE_DISABLED = 0,
	LQR_MODE_BALANCING,
	LQR_MODE_FAULT
} lqr_mode_t;

typedef enum {
	LQR_COMMAND_DISABLE = 0,
	LQR_COMMAND_ENABLE,
	LQR_COMMAND_RESET_FAULT
} lqr_command_t;

typedef struct {
	float target_stage_velocity_mps;
	float target_motor_rpm;
	uint32_t latched_faults;
	lqr_mode_t mode;
} lqr_controller_t;

typedef struct {
	lqr_state_t state;
	float force_n;
	float target_stage_velocity_mps;
	float target_motor_rpm;
	uint32_t faults;
	lqr_mode_t mode;
	uint8_t safe;
	uint8_t drive_enabled;
} lqr_controller_output_t;

void lqr_controller_init(lqr_controller_t* controller);
void lqr_controller_handle_command(lqr_controller_t* controller, lqr_command_t command);
lqr_state_t lqr_controller_convert_state(const sensor_state_t* sensor);
lqr_controller_output_t lqr_controller_update(lqr_controller_t* controller,
						const sensor_state_t* sensor,
						uint8_t sample_fresh);
void lqr_controller_apply_motor_output(const lqr_controller_output_t* output,
					step_motor_t* motor);

#endif
