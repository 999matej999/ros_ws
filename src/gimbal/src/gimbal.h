/*
 *  Copyright (C) 2021 Jozef Török torokjozef9@gmail.com
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#define MOTOR_YAW           0
#define MOTOR_PITCH         1
#define DEFAULT_SPEED       90      // [number of impulses/s]
#define MAX_SPEED           140     // [number of impulses/s]
#define MIN_SPEED           70      // [number of impulses/s]
#define TIME_OUT            0.1     // PID loop elapsed time limit [s]

#define YAW_OFFSET          180     // YAW offset for home position
#define PITCH_OFFSET        90      // PITCH offset for home possition

#include "geometry_msgs/PoseStamped.h"

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles toDegrees(EulerAngles angles);
EulerAngles toEulerAngles(geometry_msgs::Quaternion q);

class Gimbal
{

public:
	Gimbal();
	~Gimbal();

	void init();
	void home();
	void endstopsControl();
	void resetAngle(int16_t motor);
	void setAngle(double angle_req, int16_t motor, int64_t time_sec = -1, int64_t time_nsec = -1);
	void run();

private:
	
	// timing -------------------------------------------------------------------
	long double actual_time_pitch, actual_time_yaw, prev_time_pitch, prev_time_yaw;
	double elapsed_time;
	long double time_pom = 0;

	// yaw axis -------------------------------------------------------------------------------------------------
	int16_t speed_yaw = 0;                  // 4096 [number of impulses] = 15.625 [rot/s] = 100 000 [usteps/s]
	double distance_rot_yaw = 0;            // traveled rotations pitch axis
	double distance_usteps_yaw = 0;         // traveled microsteps pitch axis (resolution = 6400 usteps/rot)
	double angle_yaw = 0;                   // actual angle of rotation pitch axis
	int endstop_yaw = 0;                    // endstop yaw axis
	int8_t speed_limit_yaw = 0;             // on/off speed limit
	// PID constants
	double kp_yaw = 12; //2
	double ki_yaw = 0.00001; //0.00001
	double kd_yaw = 0.001; //0.001
	// PID variables
	double PID_p_yaw = 0;
	double PID_i_yaw = 0;
	double PID_d_yaw = 0;
	double e_yaw = 0;
	double e_prev_yaw = 0;

	// pitch axis ----------------------------------------------------------------------------------------------
	int16_t speed_pitch = 0;                // 4096 [number of impulses/s] = 15.625 [rot/s] = 100 000 [usteps/s]
	double distance_rot_pitch = 0;          // traveled rotations pitch axis
	double distance_usteps_pitch = 0;       // traveled microsteps pitch axis (resolution = 6400 usteps/rot)
	double angle_pitch = 0;                 // actual angle of rotation pitch axis
	int endstop_pitch = 0;                  // endstop pitch axis
	int8_t speed_limit_pitch = 0;           // on/off speed limit
	// PID constants
	double kp_pitch = 12; //2;
	double ki_pitch = 0.00001; //0.00001;
	double kd_pitch = 0.001; //0.001;
	// PID variables
	double PID_p_pitch = 0;
	double PID_i_pitch = 0;
	double PID_d_pitch = 0;
	double e_pitch = 0;
	double e_prev_pitch = 0;

	double hysteresis = 0.01; // hysteresis +- [°]

	// others
	int fd; // for i2c init

};

#endif // __GIMBAL_H__ //
