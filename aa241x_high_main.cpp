/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file aa241x_fw_control_main.cpp
 *
 * Wrapper for the flight controller to be designed for
 * Stanford's AA241X course.  Fixed wing controller based on
 * the fw_att_control file in the original PX4 source code.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/aa241x_mission_status.h>
#include <uORB/topics/aa241x_picture_result.h>
#include <uORB/topics/aa241x_water_drop_result.h>
#include <uORB/topics/aa241x_low_data.h>
#include <uORB/topics/aa241x_high_data.h>
#include <uORB/topics/aa241x_local_data.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <platforms/px4_defines.h>


#include "aa241x_high_params.h"
#include "aa241x_high_aux.h"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_high_main(int argc, char *argv[]);

class FixedwingControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingControl();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool	_task_should_exit;		/**< if true, attitude control task should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_loop_counter;			/**< count of iterations of main loop */

	// handles to the subscriptions needed
	int		_att_sub;				/**< vehicle attitude subscription */
	int		_accel_sub;				/**< accelerometer subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int		_sensor_combined_sub;	/**< sensor data subscription */
	int		_battery_status_sub;	/**< battery status subscription */

	int		_pic_result_sub;		/**< picture result subscription */
	int		_water_drop_result_sub;	/**< water drop result subscription NOT SURE NEEDED HERE */
	int		_mission_status_sub;	/**< aa241x mission status subscription */
	int		_low_data_sub;			/**< low priority loop data subscription */

	// the data that will be published from this controller
	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_high_data_pub;			/**< data fields to be shared with low priority module */
	orb_advert_t	_local_data_pub;		/**< custom calculated data fields */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure


	// structures of data that comes in from the uORB subscriptions
	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct accel_report					_accel;				/**< body frame accelerations */
	struct vehicle_rates_setpoint_s		_rates_sp;			/**< attitude rates setpoint */
	struct vehicle_attitude_setpoint_s	_att_sp;			/**< attitude setpoint (for debugging help */
	struct manual_control_setpoint_s	_manual;			/**< r/c channel data */
	struct airspeed_s					_airspeed;			/**< airspeed */
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator control inputs */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct sensor_combined_s			_sensor_combined;	/**< raw / minimal filtered sensor data (for some accelerations) */
	struct battery_status_s				_battery_status;	/**< battery status */

	struct picture_result_s				_pic_result;		/**< picture result MAY JUST USER AUX FILE ONE */
	struct water_drop_result_s			_water_drop_result;	/**< water drop result MAY NOT BE NEEDED HERE */
	struct aa241x_mission_status_s		_mis_status;		/**< current mission status */
	// low data struct is in attached aux header file

	// some flags
	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;					/**< if set to true, print debug output */

	map_projection_reference_s		_lake_lag_proj_ref;		/**< projection reference given by the center of the lake */

	float		_initial_baro_offset;	/**< the initial baro alt difference from adjusted gps in [m] */
	bool		_initial_offset_valid;	/**< boolean to flag whether or not the above offset is valid */


	// general RC parameters
	struct {
		float trim_roll;
		float trim_pitch;
		float trim_yaw;
	}		_parameters;			/**< local copies of interesting parameters */

	// handles for general RC parameters
	struct {
		/* rc parameters */
		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;

		/* mission parameters */
		param_t min_alt;
		param_t max_alt;
		param_t auto_alt;
		param_t cell_width;
		param_t duration;
		param_t max_radius;
		param_t timestep;
		param_t std;
		param_t t_pic;
		param_t min_fov;
		param_t max_fov;
		param_t index;
		param_t water_weight;
		param_t weight_per_drop;
		param_t ctr_lat;
		param_t ctr_lon;
		param_t ctr_alt;
	}		_parameter_handles;		/**< handles for interesting parameters */


	// handles for custom parameters
	// NOTE: the struct for the parameters themselves can be found in the aa241x_fw_aux file
	struct aah_param_handles		_aah_parameter_handles;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void	control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void	vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void	vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	void	vehicle_airspeed_poll();

	/**
	 * Check for accel updates.
	 */
	void	vehicle_accel_poll();

	/**
	 * Check for global position updates.
	 */
	void	global_pos_poll();

	/**
	 * Check for local position updates.
	 */
	void	local_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void	vehicle_status_poll();

	/**
	 * Check for combined sensor data updates.
	 */
	void	sensor_combined_poll();

	/**
	 * Check for battery status updates.
	 */
	void	battery_status_poll();

	/**
	 * Check for a picture result.
	 */
	void	picture_result_poll();

	/**
	 * Check for a water drop result.
	 */
	void	water_drop_result_poll();

	/**
	 * Check for a mission status update.
	 */
	void	mission_status_poll();

	/**
	 * Check for an update of the low priority loop data.
	 */
	void	low_data_poll();

	/**
	 * Publish the data fields from this module (high priority thread).
	 */
	void	publish_high_data();

	/**
	 * Publish the custom calculated data fields.
	 */
	void	publish_local_data();

	/**
	 * Set the aux variables that need to be constantly logged.
	 */
	void	set_local_data();

	/**
	 * Set all the aux variables needed for control law.
	 */
	void 	set_aux_values();

	/**
	 * Set the actuator output values from the control law.
	 */
	void	set_actuators();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	void	sim_testing();

	void	testing_helper();

	/**
	 * Main attitude controller collection task.
	 */
	void	task_main();

};

/* define namespace to hold the controller */
namespace att_control
{

// oddly, ERROR is not defined for c++
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingControl	*g_control = nullptr;
}


FixedwingControl::FixedwingControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_sensor_combined_sub(-1),
	_battery_status_sub(-1),
	_pic_result_sub(-1),
	_water_drop_result_sub(-1),
	_mission_status_sub(-1),
	_low_data_sub(-1),

/* publications */
	_rate_sp_pub(-1),
	_attitude_sp_pub(-1),
	_actuators_0_pub(-1),
	_high_data_pub(-1),
	_local_data_pub(-1),

	_rates_sp_id(0),
	_actuators_id(0),

/* states */
	_setpoint_valid(false),
	_debug(false),
	_initial_baro_offset(NAN),
	_initial_offset_valid(false)
{
	/* safely initialize structs */
	_att = {};
	_accel = {};
	_rates_sp = {};
	_att_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_sensor_combined = {};
	_battery_status = {};

	_pic_result = {};
	_water_drop_result = {};
	_mis_status = {};

	_lake_lag_proj_ref = {};

	// initialize the global remote parameters
	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");

	_parameter_handles.min_alt = param_find("AAMIS_ALT_MIN");
	_parameter_handles.max_alt = param_find("AAMIS_ALT_MAX");
	_parameter_handles.auto_alt = param_find("AAMIS_ALT_AUTO");
	_parameter_handles.cell_width = param_find("AAMIS_CELL_W");
	_parameter_handles.duration = param_find("AAMIS_DURATION");
	_parameter_handles.max_radius = param_find("AAMIS_RAD_MAX");
	_parameter_handles.timestep = param_find("AAMIS_TSTEP");
	_parameter_handles.std = param_find("AAMIS_STD");
	_parameter_handles.t_pic = param_find("AAMIS_TPIC");
	_parameter_handles.min_fov = param_find("AAMIS_FOV_MIN");
	_parameter_handles.max_fov = param_find("AAMIS_FOV_MAX");
	_parameter_handles.index = param_find("AA_MIS_INDEX");
	_parameter_handles.weight_per_drop = param_find("AAMIS_WGHT_DROP");
	_parameter_handles.water_weight = param_find("AA_WATER_WGHT");
	_parameter_handles.ctr_lat = param_find("AAMIS_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("AAMIS_CTR_LON");
	_parameter_handles.ctr_alt = param_find("AAMIS_CTR_ALT");

	// initialize the aa241x control parameters
	aah_parameters_init(&_aah_parameter_handles);


	// fetch initial remote parameters
	parameters_update();

	// fetch initial aa241x control parameters
	aah_parameters_update(&_aah_parameter_handles, &aah_parameters);
}

FixedwingControl::~FixedwingControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	att_control::g_control = nullptr;
}

int
FixedwingControl::parameters_update()
{

	// update the remote control parameters
	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));

	// update the mission parameters
	param_get(_parameter_handles.min_alt, &(mission_parameters.min_alt));
	param_get(_parameter_handles.max_alt, &(mission_parameters.max_alt));
	param_get(_parameter_handles.auto_alt, &(mission_parameters.auto_alt));
	param_get(_parameter_handles.cell_width, &(mission_parameters.cell_width));
	param_get(_parameter_handles.duration, &(mission_parameters.duration));
	param_get(_parameter_handles.max_radius, &(mission_parameters.max_radius));
	param_get(_parameter_handles.timestep, &(mission_parameters.timestep));
	param_get(_parameter_handles.std, &(mission_parameters.std));
	param_get(_parameter_handles.t_pic, &(mission_parameters.t_pic));
	param_get(_parameter_handles.index, &(mission_parameters.index));
	param_get(_parameter_handles.weight_per_drop, &(mission_parameters.weight_per_drop));
	param_get(_parameter_handles.water_weight, &(mission_parameters.water_weight));
	param_get(_parameter_handles.ctr_lat, &(mission_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(mission_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(mission_parameters.ctr_alt));

	// update the aa241x control parameters
	aah_parameters_update(&_aah_parameter_handles, &aah_parameters);

	return OK;
}

void
FixedwingControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedwingControl::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

void
FixedwingControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingControl::local_pos_poll()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
FixedwingControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
			_actuators_id = ORB_ID(actuator_controls_0);
		}
	}
}

void
FixedwingControl::sensor_combined_poll()
{
	/* check if there is new sensor combined data */
	bool sensor_combined_updated;
	orb_check(_sensor_combined_sub, &sensor_combined_updated);

	if (sensor_combined_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}


void
FixedwingControl::battery_status_poll()
{
	/* check if there is a new battery status */
	bool battery_status_updated;
	orb_check(_battery_status_sub, &battery_status_updated);

	if (battery_status_updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
FixedwingControl::picture_result_poll()
{
	/* check if there is a new picture result */
	bool pic_result_updated;
	orb_check(_pic_result_sub, &pic_result_updated);

	if (pic_result_updated) {
		orb_copy(ORB_ID(aa241x_picture_result), _pic_result_sub, &_pic_result);

		/* set the data to be used by students */
		new_pic = true;
		pic_result = _pic_result;
	}
}

void
FixedwingControl::water_drop_result_poll()
{
	/* check if there is a new water drop result */
	bool water_drop_result_updated;
	orb_check(_water_drop_result_sub, &water_drop_result_updated);

	if (water_drop_result_updated) {
		orb_copy(ORB_ID(battery_status), _water_drop_result_sub, &_water_drop_result);
	}
}

void
FixedwingControl::mission_status_poll()
{
	/* check if there is a new mission status */
	bool mission_status_updated;
	orb_check(_mission_status_sub, &mission_status_updated);

	if (mission_status_updated) {
		orb_copy(ORB_ID(battery_status), _mission_status_sub, &_mis_status);
	}
}

void
FixedwingControl::low_data_poll()
{
	/* check if there is a new low priority loop data */
	bool low_data_updated;
	orb_check(_low_data_sub, &low_data_updated);

	if (low_data_updated) {
		orb_copy(ORB_ID(aa241x_low_data), _low_data_sub, &low_data);
	}
}

void
FixedwingControl::publish_high_data()
{
	/* publish the high priority loop data */
	if (_high_data_pub > 0) {
		orb_publish(ORB_ID(aa241x_high_data), _high_data_pub, &high_data);
	} else {
		_high_data_pub = orb_advertise(ORB_ID(aa241x_high_data), &high_data);
	}
}

void
FixedwingControl::publish_local_data()
{
	aa241x_local_data_s local_data;
	local_data.N = position_N;
	local_data.E = position_E;
	local_data.D_baro = position_D_baro;
	local_data.D_gps = position_D_gps;
	local_data.body_u = speed_body_u;
	local_data.body_v = speed_body_v;
	local_data.body_w = speed_body_w;
	local_data.ground_speed = ground_speed;

	/* publish the high priority loop data */
	if (_local_data_pub > 0) {
		orb_publish(ORB_ID(aa241x_local_data), _local_data_pub, &local_data);
	} else {
		_local_data_pub = orb_advertise(ORB_ID(aa241x_local_data), &local_data);
	}
}

void
FixedwingControl::set_local_data()
{
	// calculate the body velocities
	speed_body_u = 0.0f;		// set them to 0 just in case unable to do calculation
	speed_body_v = 0.0f;
	speed_body_w = 0.0f;
	if(_att.R_valid) 	{
		speed_body_u = PX4_R(_att.R, 0, 0) * _global_pos.vel_n + PX4_R(_att.R, 1, 0) * _global_pos.vel_e + PX4_R(_att.R, 2, 0) * _global_pos.vel_d;
		speed_body_v = PX4_R(_att.R, 0, 1) * _global_pos.vel_n + PX4_R(_att.R, 1, 1) * _global_pos.vel_e + PX4_R(_att.R, 2, 1) * _global_pos.vel_d;
		speed_body_w = PX4_R(_att.R, 0, 2) * _global_pos.vel_n + PX4_R(_att.R, 1, 2) * _global_pos.vel_e + PX4_R(_att.R, 2, 2) * _global_pos.vel_d;
	} else	{
		if (_debug && _loop_counter % 10 == 0) {
			warnx("Did not get a valid R\n");
		}
	}

	// local position in NED frame [m] from center of lake lag
	position_N = 0.0f;
	position_E = 0.0f;
	position_D_baro = 0.0f;
	position_D_gps = -_global_pos.alt + mission_parameters.ctr_alt;
	map_projection_project(&_lake_lag_proj_ref, _global_pos.lat, _global_pos.lon, &position_N, &position_E);
	if (_local_pos.z_valid) {
		position_D_baro = _local_pos.z;

		if (_initial_offset_valid) {
			position_D_baro += _initial_baro_offset;
		}
	}
	local_pos_ne_valid = _local_pos.xy_valid;
	local_pos_d_valid = _local_pos.z_valid;

	if (gps_ok && !_initial_offset_valid && local_pos_d_valid) {
		_initial_baro_offset = position_D_baro - position_D_gps;
		_initial_offset_valid = true;
	}

	// ground course and speed
	// TODO: maybe use local position....
	ground_speed = sqrtf(_global_pos.vel_n * _global_pos.vel_n + _global_pos.vel_e * _global_pos.vel_e);		// speed relative to ground in [m/s]
	ground_course = _global_pos.yaw; 	// this is course over ground (direction of velocity relative to North in [rad])
}


void
FixedwingControl::set_aux_values()
{

	// set the euler angles and rates
	roll = _att.roll;
	pitch = _att.pitch;
	yaw = _att.yaw;

	// set the angular rates
	roll_rate = _att.rollspeed;
	pitch_rate = _att.pitchspeed;
	yaw_rate = _att.yawspeed;

	// body accelerations [m/s^2]
	accel_body_x = _sensor_combined.accelerometer_m_s2[0];
	accel_body_y = _sensor_combined.accelerometer_m_s2[1];
	accel_body_z = _sensor_combined.accelerometer_m_s2[2];

	// velocities in the NED frame [m/s]
	// TODO: maybe use local position...
	vel_N = _global_pos.vel_n;
	vel_E = _global_pos.vel_e;
	vel_D = _global_pos.vel_d;

	// airspeed [m/s]
	air_speed = _airspeed.true_airspeed_m_s;	// speed relative to air in [m/s] (measured by pitot tube)

	// status check
	gps_ok = _vehicle_status.gps_failure; 		// boolean as to whether or not the gps data coming in is valid

	// battery info
	battery_voltage = _battery_status.voltage_filtered_v;
	battery_current = _battery_status.current_a;

	// manual control inputs
	// input for each of the controls from the remote control
	man_roll_in = _manual.y;
	man_pitch_in = _manual.x;
	man_yaw_in = _manual.r;
	man_throttle_in = _manual.z;

	man_flaps_in = _manual.flaps;
	man_aux1_in = _manual.aux1;
	man_aux2_in = _manual.aux2;

	// trim conditions (from remote control)
	roll_trim = _parameters.trim_roll;
	pitch_trim = _parameters.trim_pitch;
	yaw_trim = _parameters.trim_yaw;

	// time information
	timestamp = hrt_absolute_time();
	utc_timestamp = _global_pos.time_utc_usec;

}

void
FixedwingControl::set_actuators()
{
	// do some safety checking to ensure that all the values are within the required bounds of -1..1 or 0..1
	// check roll
	if (roll_servo_out > 1) {
		roll_servo_out = 1.0f;
	}
	if (roll_servo_out < -1) {
		roll_servo_out = -1.0f;
	}

	// check pitch
	if (pitch_servo_out > 1) {
		pitch_servo_out = 1.0f;
	}
	if (pitch_servo_out < -1) {
		pitch_servo_out = -1.0f;
	}

	// check yaw
	if (yaw_servo_out > 1) {
		yaw_servo_out = 1.0f;
	}
	if (yaw_servo_out < -1) {
		yaw_servo_out = -1.0f;
	}

	// check throttle
	if (throttle_servo_out > 1) {
		throttle_servo_out = 1.0f;
	}
	if (throttle_servo_out < 0) {
		throttle_servo_out = 0.0f;
	}


	// set the actuators
	_actuators.control[0] = (isfinite(roll_servo_out)) ? roll_servo_out : roll_trim;
	_actuators.control[1] = (isfinite(roll_servo_out)) ? pitch_servo_out : pitch_trim;
	_actuators.control[2] = (isfinite(roll_servo_out)) ? yaw_servo_out : yaw_trim;
	_actuators.control[3] = (isfinite(roll_servo_out)) ? throttle_servo_out : 0.0f;
	_actuators.control[4] = _manual.flaps;
}


void
FixedwingControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
	// att_control::g_control->sim_testing(); 	// DEBUG
}

void
FixedwingControl::sim_testing()
{
	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_pic_result_sub = orb_subscribe(ORB_ID(aa241x_picture_result));
	_water_drop_result_sub = orb_subscribe(ORB_ID(aa241x_water_drop_result));
	_mission_status_sub = orb_subscribe(ORB_ID(aa241x_mission_status));
	_low_data_sub = orb_subscribe(ORB_ID(aa241x_low_data));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* rate limit attitude control to 50 Hz (with some margin, so 17 ms) */
	orb_set_interval(_att_sub, 17);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_airspeed_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	global_pos_poll();
	local_pos_poll();
	vehicle_status_poll();
	sensor_combined_poll();
	battery_status_poll();

	/* initialize projection reference */
	map_projection_init(&_lake_lag_proj_ref, (double) mission_parameters.ctr_lat, (double) mission_parameters.ctr_lon);

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		_loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);


			// update vehicle information structs as needed
			vehicle_airspeed_poll();
			vehicle_accel_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			global_pos_poll();
			local_pos_poll();
			vehicle_status_poll();
			sensor_combined_poll();
			battery_status_poll();

			// update aa241x data structs as needed
			picture_result_poll();
			water_drop_result_poll();
			mission_status_poll();
			low_data_poll();

			// set all the variables needed for the control law
			set_aux_values();

			/* run the custom control law */
			flight_control();
			// testing_helper();

			/* publish the shared data */
			publish_high_data();

			/* publish the custom calculated data */
			publish_local_data();

			// set the user desired servo positions (that were set in the flight control function)
			set_actuators();

			/* update previous loop timestamp */
			previous_loop_timestamp = timestamp;

			// set the attitude setpoint values
			_att_sp.roll_body = (isfinite(roll_desired)) ? roll_desired : 0.0f;
			_att_sp.pitch_body = (isfinite(pitch_desired)) ? pitch_desired : 0.0f;
			_att_sp.yaw_body = (isfinite(yaw_desired)) ? yaw_desired : 0.0f;
			_att_sp.thrust = (isfinite(throttle_desired)) ? throttle_desired : 0.0f;


			// TODO: maybe remove these?? (they aren't needed)
			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;


			/* publish the actuator controls */
			if (_actuators_0_pub > 0) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
			} else if (_actuators_id) {
				_actuators_0_pub= orb_advertise(_actuators_id, &_actuators);
			}

			/* publish the attitude setpoint (the targeted roll, pitch and yaw angles) */
			if (_attitude_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);
			} else {
				_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}

		}

		_loop_counter++;
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}


void
FixedwingControl::testing_helper()
{
	printf("loop deltaT = %fms\n", (double) (timestamp - previous_loop_timestamp)/1000.0);

	if (new_pic) {
		printf("New pic received:\n");

		for (int i = 0; i < pic_result.num_cells; i++) {
			printf("(%i,%i) -> %i\n", pic_result.i[i], pic_result.j[i], pic_result.state[i]);
		}

		new_pic = false;
	}

	// take the picture
	take_picture();

}


void
FixedwingControl::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_pic_result_sub = orb_subscribe(ORB_ID(aa241x_picture_result));
	_water_drop_result_sub = orb_subscribe(ORB_ID(aa241x_water_drop_result));
	_mission_status_sub = orb_subscribe(ORB_ID(aa241x_mission_status));
	_low_data_sub = orb_subscribe(ORB_ID(aa241x_low_data));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* rate limit attitude control to 50 Hz (with some margin, so 17 ms) */
	orb_set_interval(_att_sub, 17);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_airspeed_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	global_pos_poll();
	local_pos_poll();
	vehicle_status_poll();
	sensor_combined_poll();
	battery_status_poll();

	// 241x poll
	mission_status_poll();
	low_data_poll();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		_loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);


			// update vehicle information structs as needed
			vehicle_airspeed_poll();
			vehicle_accel_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			global_pos_poll();
			local_pos_poll();
			vehicle_status_poll();
			sensor_combined_poll();
			battery_status_poll();

			// update aa241x data structs as needed
			picture_result_poll();
			water_drop_result_poll();
			mission_status_poll();
			low_data_poll();

			/* set the data that needs to be logged regardless of control mode */
			set_local_data();

			/* publish the custom calculated data */
			publish_local_data();


			if (_vcontrol_mode.flag_control_auto_enabled) {

				// set all the variables needed for the control law
				set_aux_values();

				// TODO: potentially add stabilize and other modes back in....
				/* run the custom control law */
				flight_control();

				/* publish the shared data */
				publish_high_data();

				// set the user desired servo positions (that were set in the flight control function)
				set_actuators();

				/* update previous loop timestamp */
				previous_loop_timestamp = timestamp;

				// set the attitude setpoint values
				_att_sp.roll_body = (isfinite(roll_desired)) ? roll_desired : 0.0f;
				_att_sp.pitch_body = (isfinite(pitch_desired)) ? pitch_desired : 0.0f;
				_att_sp.yaw_body = (isfinite(yaw_desired)) ? yaw_desired : 0.0f;
				_att_sp.thrust = (isfinite(throttle_desired)) ? throttle_desired : 0.0f;


			} else { // have manual control of the plane

				/* manual/direct control */
				_actuators.control[0] = _manual.y;
				_actuators.control[1] = -_manual.x;
				_actuators.control[2] = _manual.r;
				_actuators.control[3] = _manual.z;
				_actuators.control[4] = _manual.flaps;

			}

			// TODO: maybe remove these?? (they aren't needed)
			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;


			/* publish the actuator controls */
			if (_actuators_0_pub > 0) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
			} else if (_actuators_id) {
				_actuators_0_pub= orb_advertise(_actuators_id, &_actuators);
			}

			/* publish the attitude setpoint (the targeted roll, pitch and yaw angles) */
			if (_attitude_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);
			} else {
				_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}

		}

		_loop_counter++;
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
FixedwingControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("aa241x_high",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&FixedwingControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int aa241x_high_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: aa241x_high {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new FixedwingControl;

		if (att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr)
			errx(1, "not running");

		delete att_control::g_control;
		att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
