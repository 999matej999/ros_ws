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

#ifndef GIMBAL_GIMBAL_H
#define GIMBAL_GIMBAL_H

#include <iostream>
#include <wiringPi.h>
#include <sys/utsname.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <math.h>

#include "i2c.h"
#include "km2.h"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

#define MOTOR_YAW           0
#define MOTOR_PITCH         1
#define DEFAULT_SPEED       90      // [number of impulses/s]
#define MAX_SPEED           140     // [number of impulses/s]
#define MIN_SPEED           70      // [number of impulses/s]
#define TIME_OUT            0.1     // PID loop elapsed time limit [s]

#define YAW_OFFSET          180     // YAW offset for home position
#define PITCH_OFFSET        90      // PITCH offset for home possition

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

// Variables ===============================================================================================
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

// Function prototypes ================================
void GimbalInit();
void EndstopsControl();
void GimbalHome();
void ResetAngle(int16_t motor);
EulerAngles ToDegrees(EulerAngles angles);
EulerAngles ToEulerAngles(Quaternion q);

void SetAngle(double angle_req, int16_t motor, int64_t time_sec = -1, int64_t time_nsec = -1)
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
                    actual_time_yaw = millis();
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
                    actual_time_pitch = millis();
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

void GimbalInit()
{
    pinMode(24, INPUT); // yaw axis endstop
    pinMode(25, INPUT); // pitch axis endstop
    ResetAngle(MOTOR_YAW); // reset angle yaw
    ResetAngle(MOTOR_PITCH); // reset angle pitch
}

void GimbalHome()
{
    speed_pitch = -DEFAULT_SPEED;
    speed_yaw = -DEFAULT_SPEED;
    while(1) // set gimbal to endstop position ------------------
    {
        EndstopsControl();
        km2_drive(fd, 0x71, speed_yaw, speed_pitch);
        if (endstop_pitch && endstop_yaw) break;
    }
    endstop_yaw = endstop_pitch = 0; // reset endstops
    ResetAngle(MOTOR_YAW); // reset angle yaw
    ResetAngle(MOTOR_PITCH); // reset angle pitch
    delay(500); // only a little delay after reaching endstop position
    
    speed_limit_yaw = speed_limit_pitch = 1; // turn on speed limit
    while(1) // set gimbal to home position ------------------------
    {
        SetAngle(YAW_OFFSET, MOTOR_YAW);
        SetAngle(PITCH_OFFSET, MOTOR_PITCH);
        km2_drive(fd, 0x71, speed_yaw, speed_pitch);
        // break when reach position
        if (speed_yaw == 0 && angle_yaw > YAW_OFFSET/2 && speed_pitch == 0 && angle_pitch > PITCH_OFFSET/2) break;
    }
    ResetAngle(MOTOR_YAW);
    ResetAngle(MOTOR_PITCH);
    // turn off speed limit
    speed_limit_yaw = 0;
    speed_limit_pitch = 0; 
}

void EndstopsControl()
{
    endstop_yaw = digitalRead(24);
    endstop_pitch = digitalRead(25);

    if (endstop_yaw) speed_yaw = 0;
    if (endstop_pitch) speed_pitch = 0;
}

void ResetAngle(int16_t motor)
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

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles a;

    a.pitch = (q.x * 90.0);
    a.yaw = (q.y * -90.0);

    a.pitch = std::max(-90.0, std::min(a.pitch, 90.0)); // limit (-90; 90) [°]

    a.yaw = std::max(-90.0, std::min(a.yaw, 90.0)); // limit (-90; 90) [°]

    #if 0
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1) angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    #endif

    return a;
}

EulerAngles ToDegrees(EulerAngles a)
{
  double pi = 3.14159;

  a.yaw *= (180/pi);
  a.pitch *= (180/pi);
  a.roll *= (180/pi);

  return a;
}

#endif //GIMBAL_GIMBAL_H
