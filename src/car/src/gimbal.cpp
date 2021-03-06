#include "gimbal.h"
#include <iostream>
#include <math.h>
#include <roboutils/utils.h>

Gimbal::Gimbal(RoboUtils::I2C *i2c, uint8_t addr): adc(i2c)
{
	this->i2c = i2c;
	this->addr = addr;

	resetAngle(MOTOR_YAW); // reset angle yaw
	resetAngle(MOTOR_PITCH); // reset angle pitch
}

Gimbal::~Gimbal()
{

}

void Gimbal::run()
{
	//endstopsControl();
	drive(speed_yaw, speed_pitch);
}

void Gimbal::drive(int16_t left, int16_t right)
{
	int16_t values[2] = { left, right };
	
	try
	{
		i2c->write16bitArray(addr, KM2_SPEED, values, 2);
	}
	catch(const char *error)
	{
		std::cout << "Motors: " << error << std::endl;
	}

}

void Gimbal::set(geometry_msgs::Quaternion quat, ros::Time stamp)
{
	EulerAngles ang_req = toEulerAngles(quat);
	//EulerAngles ang_req = ToDegrees(ang_req); // only for second variant of calculations angles

	setAngle(ang_req.pitch, MOTOR_PITCH, stamp.sec, stamp.nsec);
	setAngle(ang_req.yaw, MOTOR_YAW, stamp.sec, stamp.nsec);

	/*std::cout.precision(2);
	std::cout << "YAW =\t" << std::fixed << ang_req.yaw << "\t\t";
	std::cout << "PITCH =\t" << std::fixed << ang_req.pitch << "\t\t";
	std::cout << "ROLL =\t" << std::fixed << ang_req.roll << std::endl;*/
}

void Gimbal::setAngle(double angle_req, int16_t motor, int64_t time_sec, int64_t time_nsec)
{
	double speed; // only for conversion speed to double
	time_pom = time_nsec;
	time_pom /= 1000000000;

	switch (motor)
	{
		case 0: // yaw axis ===========================================================================================
			if (!endstop_yaw) {
				e_yaw = angle_req - angle_yaw; // error calculating
				if (e_yaw <= hysteresis && e_yaw >= -hysteresis) e_yaw = 0; // hysteresis

			// timing -----------------------------------------------------
				prev_time_yaw = actual_time_yaw; // store previous actual time
				// read actual time
				if (time_sec == -1 && time_nsec == -1)
				{
					actual_time_yaw = RoboUtils::millis();
					actual_time_yaw /= 1000;
				}
				else actual_time_yaw = time_sec+time_pom;

				elapsed_time = actual_time_yaw - prev_time_yaw;
				if (elapsed_time > TIME_OUT || elapsed_time < 0) // time out protection [s]
				{
					elapsed_time = 0;
					PID_i_yaw = 0;
					PID_d_yaw = 0;
				}
				if (elapsed_time == 0) break;

			// D
				PID_d_yaw = kd_yaw * ((e_yaw - e_prev_yaw) / elapsed_time); // D parameter calculating
				if (isnan(PID_d_yaw) || isinf(PID_d_yaw))
					PID_d_yaw = 0; // protection against division by zero or infinite
				e_prev_yaw = e_yaw; // store actual error

			// I
				PID_i_yaw += (ki_yaw * e_yaw); // I parameter calculating
			// P
				PID_p_yaw = kp_yaw * e_yaw; // P parameter calculating

			// PID
				speed_yaw = PID_p_yaw + PID_i_yaw + PID_d_yaw;

			// Limit speed
				if (speed_limit_yaw)
				{
					if (speed_yaw > MAX_SPEED) speed_yaw = MAX_SPEED;    // [number of impulses/s]
					if (speed_yaw < -MAX_SPEED) speed_yaw = -MAX_SPEED;  // [number of impulses/s]
					if (speed_yaw < MIN_SPEED && speed_yaw > 0) speed_yaw = MIN_SPEED; // [number of impulses/s]
					if (speed_yaw > -MIN_SPEED && speed_yaw < 0) speed_yaw = -MIN_SPEED; // [number of impulses/s]
				}

			// measurements calculating
				speed = speed_yaw;
				distance_usteps_yaw = distance_usteps_yaw + (speed * 100000 / 4096) * elapsed_time;
				distance_rot_yaw = distance_rot_yaw + (speed * 100000 / 4096) / 6400 * elapsed_time;
				angle_yaw = distance_usteps_yaw * 360 / 6400;
			}
			break;
		case 1: // pitch axis =========================================================================================
			if (!endstop_pitch)
			{
				e_pitch = angle_req - angle_pitch; // error calculating
				if (e_pitch <= hysteresis && e_pitch >= -hysteresis) e_pitch = 0; // hysteresis

			// timing -----------------------------------------------------
				prev_time_pitch = actual_time_pitch; // store previous actual time
				// read actual time
				if (time_sec == -1 && time_nsec == -1) 
				{
					actual_time_pitch = RoboUtils::millis();
					actual_time_pitch /= 1000;
				}
				else actual_time_pitch = time_sec+time_pom;

				elapsed_time = actual_time_pitch - prev_time_pitch;
				if (elapsed_time > TIME_OUT || elapsed_time < 0) // time out protection [s]
				{
					elapsed_time = 0;
					PID_i_pitch = 0;
					PID_d_pitch = 0;
				}
				if (elapsed_time == 0) break;

			// D
				PID_d_pitch = kd_pitch * ((e_pitch - e_prev_pitch) / elapsed_time); // D parameter calculating
				if (isnan(PID_d_pitch) || isinf(PID_d_pitch))
					PID_d_pitch = 0; // protection against division by zero or infinite
				e_prev_pitch = e_pitch; // store actual error
			// I
				PID_i_pitch += (ki_pitch * e_pitch); // I parameter calculating
			// P
				PID_p_pitch = kp_pitch * e_pitch; // P parameter calculating

			// PID
				speed_pitch = PID_p_pitch + PID_i_pitch + PID_d_pitch;

			// Limit speed
				if (speed_limit_pitch)
				{
					if (speed_pitch > MAX_SPEED) speed_pitch = MAX_SPEED;    // [number of impulses/s]
					if (speed_pitch < -MAX_SPEED) speed_pitch = -MAX_SPEED;  // [number of impulses/s]
					if (speed_pitch < MIN_SPEED && speed_pitch > 0) speed_pitch = MIN_SPEED; // [number of impulses/s]
					if (speed_pitch > -MIN_SPEED && speed_pitch < 0) speed_pitch = -MIN_SPEED; // [number of impulses/s]
				}

			// measurements calculating
				speed = speed_pitch;
				distance_usteps_pitch = distance_usteps_pitch + (speed * 100000 / 4096) * elapsed_time;
				distance_rot_pitch = distance_rot_pitch + (speed * 100000 / 4096) / 6400 * elapsed_time;
				angle_pitch = distance_usteps_pitch * 360 / 6400;
			}
			break;
	}
	// print
	//std::cout << distance_rot_pitch << "\t" << angle_pitch << "\t" << distance_usteps_pitch << std::endl;
	//std::cout << angle_yaw << "\t" << angle_pitch << std::endl;
}

void Gimbal::home()
{
	speed_pitch = -DEFAULT_SPEED;
	speed_yaw = -DEFAULT_SPEED;
	while(1) // set gimbal to endstop position ------------------
	{
		endstopsControl();
		drive(speed_yaw, speed_pitch);
		if (endstop_pitch && endstop_yaw) break;
	}
	endstop_yaw = endstop_pitch = 0; // reset endstops
	resetAngle(MOTOR_YAW); // reset angle yaw
	resetAngle(MOTOR_PITCH); // reset angle pitch
	RoboUtils::delay(500); // only a little delay after reaching endstop position
	
	speed_limit_yaw = speed_limit_pitch = 1; // turn on speed limit
	while(1) // set gimbal to home position ------------------------
	{
		setAngle(YAW_OFFSET, MOTOR_YAW);
		setAngle(PITCH_OFFSET, MOTOR_PITCH);
		drive(speed_yaw, speed_pitch);
		// break when reach position
		if (speed_yaw == 0 && angle_yaw > YAW_OFFSET/2 && speed_pitch == 0 && angle_pitch > PITCH_OFFSET/2) break;
	}
	resetAngle(MOTOR_YAW);
	resetAngle(MOTOR_PITCH);
	// turn off speed limit
	speed_limit_yaw = 0;
	speed_limit_pitch = 0; 
}

void Gimbal::endstopsControl()
{
	auto raw0 = adc.readChannel(0);
	auto raw1 = adc.readChannel(1);

	samples_yaw[count] = adc.readChannel(0);
	samples_pitch[count] = adc.readChannel(1);

	if(++count >= SAMPLES) count = 0;

	uint32_t sum_yaw = 0;
	uint32_t sum_pitch = 0;

	for(size_t i = 0; i < SAMPLES; ++i)
	{
		sum_yaw += samples_yaw[i];
		sum_pitch += samples_pitch[i];
	}

	uint16_t avg_yaw = sum_yaw / SAMPLES;
	uint16_t avg_pitch = sum_pitch / SAMPLES;

	endstop_yaw = avg_yaw > HALL_SENSOR_THRESHOLD_YAW; // yaw axis endstop
	endstop_pitch = avg_pitch > HALL_SENSOR_THRESHOLD_PITCH; // pitch axis endstop

	//std::cout << raw0 << ", " << raw1 << ", " << avg_yaw << ", " << avg_pitch << std::endl;

	if(endstop_yaw) speed_yaw = 0;
	if(endstop_pitch) speed_pitch = 0;
}

void Gimbal::resetAngle(int16_t motor)
{
	switch (motor)
	{
		case 0: // yaw axis ==========
			distance_usteps_yaw = 0;
			distance_rot_yaw = 0;
			angle_yaw = 0;
			break;
		case 1: // pitch axis ========
			distance_usteps_pitch = 0;
			distance_rot_pitch = 0;
			angle_pitch = 0;
			break;
	}
}


EulerAngles toEulerAngles(geometry_msgs::Quaternion q)
{
	EulerAngles a;

	a.pitch = (q.x * 90.0);
	a.yaw = (q.y * -90.0);

	a.pitch = std::max(-90.0, std::min(a.pitch, 90.0)); // limit (-90; 90) [??]

	a.yaw = std::max(-90.0, std::min(a.yaw, 90.0)); // limit (-90; 90) [??]

	return a;
}

EulerAngles toDegrees(EulerAngles a)
{
  double pi = 3.14159;

  a.yaw *= (180/pi);
  a.pitch *= (180/pi);
  a.roll *= (180/pi);

  return a;
}