//
// Created by Martin Mušinka on 15/04/2021.
//

#ifndef HELLOWORLD_PCA9685SERVO_H
#define HELLOWORLD_PCA9685SERVO_H

#include "PCA9685.h"

#define SERVO_LEFT_DEFAULT_MS	0.8
#define SERVO_RIGHT_DEFAULT_MS 	2.2

class PCA9685Servo : PCA9685{
public:
    PCA9685Servo(RoboUtils::I2C *i2c, uint8_t nChannel);
    ~PCA9685Servo();

    void SetDirection(float nDir);

private:
    RoboUtils::I2C *i2c;
    uint8_t nChannel;

};


#endif //HELLOWORLD_PCA9685SERVO_H
