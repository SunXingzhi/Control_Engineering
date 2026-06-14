#include "../include/lqr_controller.h"
#include "../include/lqr_controller_config.h"
#include <math.h>

static float clamp_float(float value, float minimum, float maximum)
{
	if (value < minimum) return minimum;
	if (value > maximum) return maximum;
	return value;
}

static uint8_t state_is_finite(const lqr_state_t* state)
{
	return isfinite(state->x_dot)
		&& isfinite(state->theta1_dot)
		&& isfinite(state->theta2_dot)
		&& isfinite(state->x)
		&& isfinite(state->theta1)
		&& isfinite(state->theta2);
}

static void reset_targets(lqr_controller_t* controller)
{
	controller->target_stage_velocity_mps = 0.0f;
	controller->target_motor_rpm = 0.0f;
}

static void latch_fault(lqr_controller_t* controller, uint32_t faults)
{
	controller->latched_faults |= faults;
	controller->mode = LQR_MODE_FAULT;
	reset_targets(controller);
}

void lqr_controller_init(lqr_controller_t* controller)
{
	if (controller == NULL) return;
	reset_targets(controller);
	controller->latched_faults = LQR_FAULT_NONE;
	controller->mode = LQR_MODE_DISABLED;
}

void lqr_controller_handle_command(lqr_controller_t* controller, lqr_command_t command)
{
	if (controller == NULL) return;

	reset_targets(controller);
	switch (command){
	case LQR_COMMAND_ENABLE:
		if (controller->latched_faults == LQR_FAULT_NONE){
			controller->mode = LQR_MODE_BALANCING;
		}
		break;
	case LQR_COMMAND_RESET_FAULT:
		controller->latched_faults = LQR_FAULT_NONE;
		controller->mode = LQR_MODE_DISABLED;
		break;
	case LQR_COMMAND_DISABLE:
	default:
		controller->mode = LQR_MODE_DISABLED;
		break;
	}
}

lqr_state_t lqr_controller_convert_state(const sensor_state_t* sensor)
{
	lqr_state_t state = {0};
	if (sensor == NULL) return state;

	const float travel_per_rad = LQR_PULLEY_TRAVEL_PER_REV_M / (2.0f * PI);

	state.x_dot = sensor->x_dot * LQR_PULLEY_TRAVEL_PER_REV_M / 60.0f;
	state.theta1_dot = sensor->theta1_dot * LQR_ANGLE_RAW_TO_RAD * LQR_THETA1_SIGN;
	state.theta2_dot = sensor->theta2_dot * LQR_ANGLE_RAW_TO_RAD * LQR_THETA2_SIGN;
	state.x = (sensor->x_cart - LQR_STAGE_CENTER_ENCODER_RAD) * travel_per_rad;
	state.theta1 = (sensor->theta1 - LQR_THETA1_ZERO_RAW)
			* LQR_ANGLE_RAW_TO_RAD * LQR_THETA1_SIGN;
	state.theta2 = (sensor->theta2 - LQR_THETA2_ZERO_RAW)
			* LQR_ANGLE_RAW_TO_RAD * LQR_THETA2_SIGN;

	return state;
}

lqr_controller_output_t lqr_controller_update(lqr_controller_t* controller,
						const sensor_state_t* sensor,
						uint8_t sample_fresh)
{
	lqr_controller_output_t output = {0};
	if (controller == NULL || sensor == NULL){
		output.faults = LQR_FAULT_NON_FINITE;
		return output;
	}

	if (!sample_fresh){
		latch_fault(controller, LQR_FAULT_SENSOR_TIMEOUT);
	}

	output.mode = controller->mode;
	output.faults = controller->latched_faults;
	if (controller->mode == LQR_MODE_FAULT){
		return output;
	}

	output.state = lqr_controller_convert_state(sensor);
	if (!state_is_finite(&output.state)){
		latch_fault(controller, LQR_FAULT_NON_FINITE);
		output.faults = controller->latched_faults;
		output.mode = controller->mode;
		return output;
	}

	/* 停用状态只监测传感器有效性，不对静止摆角进行越界锁存。 */
	if (controller->mode == LQR_MODE_DISABLED){
		output.safe = 1;
		return output;
	}

	const float state_vector[6] = {
		output.state.x_dot,
		output.state.theta1_dot,
		output.state.theta2_dot,
		output.state.x,
		output.state.theta1,
		output.state.theta2
	};

	for (uint8_t i = 0; i < 6; i++){
		output.force_n -= LQR_K[i] * state_vector[i];
	}

	if (self_fabs(output.state.x) > LQR_MAX_STAGE_POSITION_M){
		output.faults |= LQR_FAULT_STAGE_LIMIT;
	}
	if (self_fabs(output.state.theta1) > LQR_MAX_ABS_ANGLE_RAD
	    || self_fabs(output.state.theta2) > LQR_MAX_ABS_ANGLE_RAD){
		output.faults |= LQR_FAULT_ANGLE_LIMIT;
	}
	if (!isfinite(output.force_n) || self_fabs(output.force_n) > LQR_MAX_ABS_FORCE_N){
		output.faults |= LQR_FAULT_FORCE_LIMIT;
	}

	if (output.faults != LQR_FAULT_NONE){
		latch_fault(controller, output.faults);
		output.faults = controller->latched_faults;
		output.mode = controller->mode;
		return output;
	}

	output.safe = 1;
	if (controller->mode != LQR_MODE_BALANCING){
		return output;
	}

	const float acceleration = output.force_n / LQR_EFFECTIVE_STAGE_MASS_KG;
	controller->target_stage_velocity_mps += acceleration * LQR_CONTROL_DT_S;
	controller->target_stage_velocity_mps = clamp_float(
		controller->target_stage_velocity_mps,
		-LQR_MAX_STAGE_SPEED_MPS,
		LQR_MAX_STAGE_SPEED_MPS);

	output.target_stage_velocity_mps = controller->target_stage_velocity_mps;
	float desired_motor_rpm = clamp_float(
		output.target_stage_velocity_mps
			* 60.0f / LQR_PULLEY_TRAVEL_PER_REV_M,
		-LQR_MAX_MOTOR_RPM,
		LQR_MAX_MOTOR_RPM);
	/* 换向时必须先降到零，避免单周期直接跨零反转。 */
	if ((controller->target_motor_rpm > 0.0f && desired_motor_rpm < 0.0f)
	    || (controller->target_motor_rpm < 0.0f && desired_motor_rpm > 0.0f)){
		desired_motor_rpm = 0.0f;
	}
	const float rpm_delta = clamp_float(
		desired_motor_rpm - controller->target_motor_rpm,
		-LQR_MAX_MOTOR_RPM_STEP,
		LQR_MAX_MOTOR_RPM_STEP);
	controller->target_motor_rpm += rpm_delta;
	output.target_motor_rpm = controller->target_motor_rpm;
#if LQR_MOTOR_OUTPUT_ENABLED && LQR_PARAMETERS_VERIFIED
	output.drive_enabled = 1;
#endif
	return output;
}

void lqr_controller_apply_motor_output(const lqr_controller_output_t* output,
					step_motor_t* motor)
{
#if LQR_MOTOR_OUTPUT_ENABLED && LQR_PARAMETERS_VERIFIED
	if (output == NULL || motor == NULL) return;

	if (!output->drive_enabled || !output->safe){
		step_motor_stop(motor);
		return;
	}

	const motor_direction_t direction = output->target_motor_rpm >= 0.0f
			? POSITIVE_DIR
			: NEGATIVE_DIR;
	step_motor_set_speed(motor, self_fabs(output->target_motor_rpm), direction);
#else
	(void)output;
	(void)motor;
#endif
}
