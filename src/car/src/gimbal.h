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

// default I2C address of module
#define ADDR_KM2_DEFAULT      0x71

// KM2 registers
#define KM2_SPEED             0x00   // RW 2x uint16_t
#define KM2_ODOMETRY          0x01   // R- 2x uint32_t
#define KM2_STATUS            0x01   // RW 1x uint16_t

#define KM2_CMDRESET          0xE0   // -W no data
#define KM2_CMDLOAD           0xE1   // -W no data
#define KM2_CMDSTORE          0xE2   // -W no data

#define KM2_CFGMAXSPD         0xF0    // RW 1x int16_t
#define KM2_CFGTIMEOUT        0xF1    // RW 1x uint16_t
#define KM2_CFGTIMEOUTPWOFF   0xF2    // RW 1x uint16_t
#define KM2_CFGADDR           0xFF    // RW 1x uint8_t (or 2x uint8_t)

// KM2 register values

// KM2_CFGADDR -----------------------------------------------------------------

#define KM2_CFGADDR_ADDR          (0x7F << 1)  
#define KM2_CFGADDR_ADDR_(v)      ((v) << 1)
#define KM2_CFGADDR_BCASTEN       (0x01)      // if set, device will respond on addr 0 too


#define MOTOR_YAW           0
#define MOTOR_PITCH         1
#define DEFAULT_SPEED       30      // [number of impulses/s]
#define MAX_SPEED           140     // [number of impulses/s]
#define MIN_SPEED           70      // [number of impulses/s]
#define TIME_OUT            0.1     // PID loop elapsed time limit [s]

#define YAW_OFFSET          180     // YAW offset for home position
#define PITCH_OFFSET        90      // PITCH offset for home possition

#define SAMPLES				5

#define HALL_SENSOR_THRESHOLD_YAW 2650
#define HALL_SENSOR_THRESHOLD_PITCH 2850

#include "geometry_msgs/PoseStamped.h"
#include <roboutils/I2C.h>
#include <roboutils/ADC.h>

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles toDegrees(EulerAngles angles);
EulerAngles toEulerAngles(geometry_msgs::Quaternion q);

class Gimbal
{

public:
	Gimbal(RoboUtils::I2C *i2c, uint8_t addr = ADDR_KM2_DEFAULT);
	~Gimbal();

	void home();
	void set(geometry_msgs::Quaternion quat, ros::Time stamp);
	void endstopsControl();
	void resetAngle(int16_t motor);
	void setAngle(double angle_req, int16_t motor, int64_t time_sec = -1, int64_t time_nsec = -1);
	void run();
	void drive(int16_t left, int16_t right);

private:
	RoboUtils::I2C *i2c;
	RoboUtils::ADC adc;
	uint8_t addr;

	uint16_t samples_yaw[SAMPLES]= {};
	uint16_t samples_pitch[SAMPLES] = {};
	size_t count = 0;
	
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

};

#endif // __GIMBAL_H__ //
