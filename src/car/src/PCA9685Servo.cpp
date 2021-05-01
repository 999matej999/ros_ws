//
// Created by Martin Mušinka on 15/04/2021.
//

#include "roboutils/I2C.h"

#include "PCA9685Servo.h"

float map(float x, float in_min, float in_max, float out_min, float out_max) {     //funkcia map na zmenu rozsahu
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

PCA9685Servo::PCA9685Servo(RoboUtils::I2C *i2c): PCA9685(i2c) {

    this->i2c = i2c;
    reset();
    setPWMFreq(50);

}

PCA9685Servo::~PCA9685Servo() {
}

void PCA9685Servo::reset() {

    i2c->write8bitLEValue(0x40,MODE1, 0x00); //Normal mode
    i2c->write8bitLEValue(0x40,MODE2, 0x04); //totem pole

}

void PCA9685Servo::SetDirection(uint8_t nChannel, float nDir) {

    nDir = map(nDir,-1,1,SERVO_LEFT_DEFAULT_MS,SERVO_RIGHT_DEFAULT_MS);
    setPWM(nChannel, 4095-(nDir*4095)/20 );

}