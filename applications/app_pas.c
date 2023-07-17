/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include <math.h>

#include <unistd.h> // For usleep function (microsecond delay)

// Settings
#define PEDAL_INPUT_TIMEOUT 0.2
#define MAX_MS_WITHOUT_CADENCE 3000
#define MIN_MS_WITHOUT_POWER 500
#define FILTER_SAMPLES 5
#define RPM_FILTER_SAMPLES 8
#define MS_WITHOUT_CADENCE_COOLING_TIME 1000
// Define PID controller constants
#define KP 0.01 // Proportional gain
#define KI 0.01 // Integral gain
#define KD 0.1	// Derivative gain
// Define PID controller variables
static float integral_error = 0.0; // Integral error
static float prev_error = 0.0;	   // Previous error
#define MAX_MOTOR_RPM 15000

// Threads
static THD_FUNCTION(pas_thread, arg);
static THD_WORKING_AREA(pas_thread_wa, 1024);

// Private variables
static volatile pas_config config;
static volatile float sub_scaling = 1.0;
static volatile float output_current_rel = 0.0;
static volatile float ms_without_power = 0.0;
static volatile float max_pulse_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float pedal_rpm = 0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float torque_ratio = 0.0;
static volatile float min_start_torque = 0.5; // put this later in config
static volatile bool torque_on_adc1 = false;
static volatile bool adc_active = false;
static volatile float adc_throttle;
static volatile float ms_without_cadence = 0.0;
static volatile bool first_start_init = true;
static volatile float ms_without_cadence_cooling_time = 0.0;
static volatile bool min_start_torque_reached = false;

static float volatile max_speed = 25 / 3.6;
static float current_speed;
static float threshold_speed;
static float current_rpm;
static float current_rpm_last;
static float current_max_rpm = 7500;
static float current_rpm_goal = 7500;
static float current_rpm_goal_last;
static float abs_current;
static volatile uint16_t sample_time = 0;
static volatile float drift_percent = 0;
static volatile float torque_percent = 0;
static volatile uint16_t uptime = 0;
static volatile uint16_t downtime = 0;
uint16_t print_trigger = 0;
static volatile bool torque_started = false;
static volatile uint8_t torque_smoothing_trigger = 0; // 2ms loop = 10* 10 = 100ms
static volatile float error = MAX_MOTOR_RPM * 0.2;	  // 20% from max rpm is the start point
static volatile float output_pid;

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
void app_pas_configure(pas_config *conf)
{
	config = *conf;
	ms_without_power = 0.0;
	output_current_rel = 0.0;

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_rpm_end * 3.0 / 60.0));

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);
}

/**
 * Start PAS thread
 *
 * @param is_primary_output
 * True when PAS app takes direct control of the current target,
 * false when PAS app shares control with the ADC app for current command
 */
void app_pas_start(bool is_primary_output)
{
	stop_now = false;
	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);

	primary_output = is_primary_output;
}

bool app_pas_is_running(void)
{
	return is_running;
}

bool has_torque_on_adc1(void)
{
	return torque_on_adc1;
}

void app_pas_stop(void)
{
	stop_now = true;
	while (is_running)
	{
		chThdSleepMilliseconds(1);
	}

	if (primary_output == true)
	{
		mc_interface_set_current_rel(0.0);
	}
	else
	{
		output_current_rel = 0.0;
	}
}

void app_pas_set_current_sub_scaling(float current_sub_scaling)
{
	sub_scaling = current_sub_scaling;
}

float app_pas_get_current_target_rel(void)
{
	return output_current_rel;
}

float app_pas_get_pedal_rpm(void)
{
	return pedal_rpm;
}

void pas_event_handler(void)
{
#ifdef HW_HAS_3_WIRES_PAS_SENSOR
	if (config.sensor_type == PAS_SENSOR_TYPE_STANDARD)
	{
		uint8_t new_state;
		static uint8_t pulse_count = 0;
		static uint8_t old_state = 0;
		static float old_timestamp = 0;
		static float inactivity_time = 0;
		static float period_filtered = 0;

		uint8_t PAS_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
		// Maybe here Set the second port for external sensor
		new_state = PAS_level;
		if (new_state != old_state)
			pulse_count++;

		old_state = new_state;

		const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

		// sensors are poorly placed, so use only one rising edge as reference
		if (pulse_count > 1)
		{
			float period = (timestamp - old_timestamp) * (float)config.magnets;
			old_timestamp = timestamp;

			UTILS_LP_FAST(period_filtered, period, 0.5);

			if (period_filtered < min_pedal_period)
			{ // can't be that short, abort
				return;
			}
			pedal_rpm = 60.0 / period_filtered;
			inactivity_time = 0.0;
			pulse_count = 0;
		}
		else
		{
			inactivity_time += 1.0 / (float)config.update_rate_hz;

			// if no pedal activity, set RPM as zero + taking in account the current RPM speed to give a more natural stop feeling
			if (inactivity_time > 1.0 / ((((config.pedal_rpm_start / 2) + pedal_rpm) / 60.0) * config.magnets) * 2)
			{
				pedal_rpm = 0.0;
			}
		}
	}
#endif

#ifdef HW_HAS_4_WIRES_PAS_SENSOR
	const int8_t QEM[] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix
	int8_t direction_qem;
	uint8_t new_state;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static uint8_t correct_direction_counter = 0;
	static uint8_t correct_direction_counter_last = 100;

	uint8_t PAS1_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
	uint8_t PAS2_level = palReadPad(HW_PAS2_PORT, HW_PAS2_PIN);

	// Hall sensor based torque sensor data aquisition
	if (pedal_rpm != 0)
	{
		// If HALL TORQUE SENSOR
		if (PAS1_level == 1 && PAS2_level == 1)
		{
			uptime++;
		}
		else if (uptime != 0 && pedal_rpm != 0)
		{
			downtime++;
		}
		sample_time = uptime + downtime;
		if (sample_time > 100)
		{
			drift_percent = ((((uptime * 100) / sample_time) - 36) * -1) * 5; // 5 is a calibration factor to get percentage
			uptime = 0;
			downtime = 0;
		}

		// safety guards
		drift_percent = fmin(fmax(drift_percent, 0), 100);

		// reaching for smoother values
		torque_smoothing_trigger++;
		if (torque_smoothing_trigger > 8)
		{

			if (drift_percent > torque_percent)
			{
				torque_percent += (drift_percent - torque_percent) / 2;
			}
			else if (drift_percent < torque_percent)
			{
				torque_percent -= (torque_percent - drift_percent) / 2;
			}
			torque_smoothing_trigger = 0;
			// safety guards
			torque_percent = fmin(fmax(torque_percent, 0), 100);

		}
	}
	new_state = PAS2_level * 2 + PAS1_level;
	direction_qem = (float)QEM[old_state * 4 + new_state];
	old_state = new_state;

	const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	// Require several quadrature events in the right direction to prevent vibrations from
	// engging PAS
	int8_t direction = (direction_conf * direction_qem);

	switch (direction)
	{
	case 1:
		correct_direction_counter++;
		break;
	case -1:
		correct_direction_counter = 0;
		break;
	}

	// sensors are poorly placed, so use only one rising edge as reference
	if ((new_state == 3) && (correct_direction_counter >= 4) && correct_direction_counter_last != correct_direction_counter)
	{
		float period = (timestamp - old_timestamp) * (float)config.magnets;
		old_timestamp = timestamp;

		UTILS_LP_FAST(period_filtered, period, 1.0);

		if (period_filtered < min_pedal_period)
		{ // can't be that short, abort
			return;
		}
		if (correct_direction_counter >= 16)
		{
			correct_direction_counter = 4;
		}
		correct_direction_counter_last = correct_direction_counter;
		pedal_rpm = 60.0 / period_filtered;
		pedal_rpm *= (direction_conf * (float)direction_qem);
		inactivity_time = 0.0;
	}
	else
	{
		inactivity_time += 1.0 / (float)config.update_rate_hz;

		// if no pedal activity, set RPM as zero + taking in account the current RPM speed to give a more natural stop feeling
		if (inactivity_time > 1.0 / ((((config.pedal_rpm_start / 2) + pedal_rpm) / 60.0) * config.magnets) * 2)
		{
			pedal_rpm = 0.0;
		}
	}
#endif
}
static THD_FUNCTION(pas_thread, arg)
{
	(void)arg;

	float output = 0;
	chRegSetThreadName("APP_PAS");

#ifdef HW_HAS_PAS_SENSOR
	palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
#endif

#ifdef HW_HAS_4_WIRES_PAS_SENSOR
	palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
#endif

	is_running = true;

	for (;;)
	{
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0)
		{
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now)
		{
			is_running = false;
			return;
		}

		pas_event_handler(); // this could happen inside an ISR instead of being polled

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE)
		{
			ms_without_power = 0;
		}

		if (app_is_output_disabled())
		{
			continue;
		}

		switch (config.ctrl_type)
		{

		case PAS_CTRL_TYPE_NONE:
			output = 0.0;
			break;
		case PAS_CTRL_TYPE_BASIC: // PAS_CTRL_TYPE_HALL_TORQUE

			if (torque_percent > 20 && !torque_started) // 20 is a good value
			{
				torque_started = true;
			}
			if (pedal_rpm > (config.pedal_rpm_start) && torque_started)
			{

				output = (utils_throttle_curve((torque_percent / 100), -0.95, 0, 0)) + 0.005;

			}
			else
			{
				output = 0.0;
				torque_started = false;
			}

			break;

		case PAS_CTRL_TYPE_CADENCE:
			// Map pedal rpm to assist level
			// NOTE: If the limits are the same a numerical instability is approached, so in that case
			// just use on/off control (which is what setting the limits to the same value essentially means).
			if (pedal_rpm > (config.pedal_rpm_start))
			{

				output = 1.0;
			}
			else
			{
				output = 0.0;
			}

			break;
		case PAS_CTRL_TYPE_TORQUE: // Standard way a PAS/Torque sensor (analog) on an E-Bike works. if PAS over min RPM. it starts and gives the max what assist level is * torque ration (0->1) set at, else, it stops. It uses throttle as TS!
			// first start init
			if (first_start_init)
			{
				torque_on_adc1 = true;
				app_adc_detach_adc(1);
				first_start_init = false;
			}

			torque_ratio = app_adc_get_decoded_level();

			if (pedal_rpm > (config.pedal_rpm_start + 1.0))
			{
				output = config.current_scaling * torque_ratio * sub_scaling;
				utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
				ms_without_cadence = 0.0;
				ms_without_cadence_cooling_time = 0.0;
				min_start_torque_reached = false;
			}
			// start on pedal press available (1s delay) for 3s, if no PAS signal are detected it cools down for 1 second.
			else
			{
				if (torque_ratio > min_start_torque)
				{
					min_start_torque_reached = true;
				}
				if (min_start_torque_reached)
				{
					if (ms_without_cadence_cooling_time > MS_WITHOUT_CADENCE_COOLING_TIME)
					{
						ms_without_cadence += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
						if (ms_without_cadence < MAX_MS_WITHOUT_CADENCE)
						{
							output = config.current_scaling * torque_ratio * sub_scaling;
							utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
						}
						else
						{
							output = 0.0;
							ms_without_cadence_cooling_time = 0.0;
							min_start_torque_reached = false;
						}
					}
					else
					{
						output = 0.0;
						ms_without_cadence_cooling_time += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					}
				}
			}
			break;

// this had to be modified (CAN_TORQUE_SENSOR) because it's an Bafang M600 specific hardware definition (CANbus torque sensor) it may be included but in a other way.
#ifdef HW_HAS_CAN_TORQUE_SENSOR
		case PAS_CTRL_TYPE_CAN_TORQUE:
		{
			torque_ratio = hw_get_PAS_torque();
			output = torque_ratio * config.current_scaling * sub_scaling;
			utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
		}
		/* fall through */
		case PAS_CTRL_TYPE_CAN_TORQUE_WITH_CADENCE_TIMEOUT:
		{
			// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
			// motor overtemps when the rider is just resting on the pedals
			static float ms_without_cadence_or_torque = 0.0;
			if (output == 0.0 || pedal_rpm > 0)
			{
				ms_without_cadence_or_torque = 0.0;
			}
			else
			{
				ms_without_cadence_or_torque += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
				if (ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE)
				{
					output = 0.0;
				}
			}
		}
#endif
		default:
			break;
		}

		// APPLY SPEED LIMITING
		if (pedal_rpm > (config.pedal_rpm_start))
		{
			// get current speed and rpm
			current_speed = mc_interface_get_speed();
			current_rpm = mc_interface_get_rpm();
			abs_current = mc_interface_get_tot_current_filtered();

			// set the threshold speed for limiting speed limiting interference
			threshold_speed = max_speed * 0.69; // Last 0.05% of the maximum speed

			// calculate the gear ratio to know when to stop
			if (current_speed > threshold_speed && abs_current > 10 && current_rpm != 0 && current_speed < max_speed && current_rpm > current_rpm_last)
			{
				current_rpm_goal = (max_speed * current_rpm) / current_speed;
				// guard unuseful rpm target
				if (current_rpm_goal >= MAX_MOTOR_RPM)
					current_rpm_goal = MAX_MOTOR_RPM;
				// ignore small fluctuations
				if (fabsf(current_rpm_goal - current_rpm_goal_last) < 200)
				{
					current_rpm_goal = current_rpm_goal_last;
				}
				else
				{
					if (current_rpm_goal > current_rpm_goal_last)
						current_rpm_goal += (current_rpm_goal - current_rpm_goal_last) / 2;
					else
						current_rpm_goal -= (current_rpm_goal_last - current_rpm_goal) / 2;
				}

				current_rpm_goal_last = current_rpm_goal;
			}

			// set a point to know if rpm are growing or not
			current_rpm_last = current_rpm;

			// apply the reach at thread Hz.
			current_max_rpm = current_max_rpm + ((current_rpm_goal - current_max_rpm) / 2);

			// start PID control after knowing the max rpm
			if (current_speed > ((threshold_speed / 6.9) * 7))
			{
				// Calculate the error (difference between desired speed and current speed)
				error = ((current_max_rpm - current_rpm) * 100) / (current_max_rpm * 0.2);
				error = fmin(fmax(error, -100.0), 100.0);

				// Update the integral error and guard for overvalues
				integral_error += ((error / config.update_rate_hz)); // 500Hz = 0.002s
				integral_error = fmin(fmax(integral_error, -100.0), 100.0);

				// Calculate the PID output
				output_pid = KP * error + KI * integral_error + KD * (error - prev_error);
				output_pid = fmin(fmax(output_pid, 0.0), 1.0);

				// Update the previous error
				prev_error = error;

				// Apply the scaling factor to the output
				if (output > output_pid)
					output *= output_pid;

				//*************************** PRINT *************************
				// if (print_trigger < 200)
				// {
				// 	print_trigger++;
				// }
				// else
				// {
				// 	// commands_printf("output %d\n", (int)(output_pid * 100));
				// 	commands_printf(" current_speed = %d , current_max_rpm = %d , current_rpm = %d, error = %d , KP error = %d , KI error = %d , KD error = %d, output = %d, abs_current = %d\n", (int)((current_speed * 3.6)), (int)(current_max_rpm), (int)(current_rpm), (int)(error), (int)(KP * error) * 100, (int)(KI * integral_error) * 100, (int)(KD * (error - prev_error)) * 1000, (int)(output * 100), (int)(abs_current * 100));
				// 	print_trigger = 0;
				// }
			}
			if (current_speed < threshold_speed)
				current_max_rpm = MAX_MOTOR_RPM;
		}
		else
		{
		}

		// APPLY RAMPING
		static systime_t last_time = 0;
		static float output_ramp = 0.0;
		float ramp_time_pos = config.ramp_time_pos; // Config ramp time for positive ramping
		float ramp_time_neg = config.ramp_time_neg; // COnfig ramp time for negative ramping

		// Apply smooth ramping at start.
		if (pedal_rpm > 0.01)
		{
			// Adjust the ramp times to be inversely proportional to the pedal RPM
			ramp_time_pos /= (pedal_rpm / 100);
			ramp_time_neg /= (pedal_rpm / 100);
		}

		float ramp_time = fabsf(output) > fabsf(output_ramp) ? ramp_time_pos : ramp_time_neg;

		// slow things up when speed limiting and over speed limit threshold
		if (pedal_rpm > (config.pedal_rpm_start + 1.0) && current_speed > threshold_speed)
			ramp_time *= 4;
		// add here double ramp time when torque only is detected.
		if (ramp_time > 0.01)
		{
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);

			utils_step_towards(&output_ramp, output, ramp_step);

			// apply max power limiting
			output = utils_map(output_ramp, 0, 1.0, 0.0, config.current_scaling);

			last_time = chVTGetSystemTimeX();
		}

		if (output < 0.001)
		{
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}

		// Safe start is enabled if the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER)
		{
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before)
			{
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			output_current_rel = 0.0;
			continue;
		}

		// Reset timeout
		timeout_reset();

		// print guard function
		//  if (print_trigger < 200)
		//  {
		//  	print_trigger++;
		//  }
		//  else
		//  {
		//  	// commands_printf("output %d\n", (int)(output_pid * 100));
		//  	commands_printf("output= %d, output_pid= %d, output_ramp= %d\n", (int)(output * 100), (int)(output_pid * 100), (int)(output_ramp * 100));
		//  	print_trigger = 0;
		//  }

		if (primary_output == true)
		{
			mc_interface_set_current_rel(output);
		}
		else
		{
			output_current_rel = output;
		}
	}
}
