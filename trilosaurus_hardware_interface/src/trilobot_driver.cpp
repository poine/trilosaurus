#include <trilosaurus_hardware_interface/trilobot_driver.h>

#include <iostream>
#include <wiringPi.h> 
#include <softPwm.h>

struct TrilobotDriverStruct {
  int i2c_fd;
  int32_t enc1;
  int32_t enc2;
};

struct TrilobotDriverStruct _driver;

int trilobot_driver_init() {
  if (wiringPiSetup() == -1) 	          /* initialize wiringPi setup */
    return -1;
  motors_init();
  encoders_init();
  return 0;
}

// Teensy with Quadencoders
#include <wiringPiI2C.h>

#define ENC_ADDR 0x2D
#define REG_RESET 0x00
#define REG_ENC1 0x06
#define REG_ENC2 0x0A

int encoders_init() {
  _driver.i2c_fd = wiringPiI2CSetup(ENC_ADDR);
    if (_driver.i2c_fd == -1) {
        std::cout << "Failed to init I2C communication.\n";
        return -1;
    }
    std::cout << "I2C communication successfully setup.\n";
    return 0;
}

int encoders_read(int32_t* e1, int32_t* e2) {
  _driver.enc1 = wiringPiI2CReadReg16(_driver.i2c_fd, REG_ENC1);
  _driver.enc1 += (wiringPiI2CReadReg16(_driver.i2c_fd, REG_ENC1+2)<<16);
  _driver.enc2 = wiringPiI2CReadReg16(_driver.i2c_fd, REG_ENC2);
  _driver.enc2 += (wiringPiI2CReadReg16(_driver.i2c_fd, REG_ENC2+2)<<16);
  if (e1) *e1 = _driver.enc1;
  if (e2) *e2 = _driver.enc2;
  return 0;
}

int encoders_reset() {
  wiringPiI2CWriteReg8(_driver.i2c_fd, REG_RESET, 1);
  return 0;
}

// Motor driver pins, via DRV8833PWP Dual H-Bridge
#define MOTOR_EN_PIN       26 // BCM
#define MOTOR_EN_PIN_WPI   25 // Wiring pi library

#define MOTOR_LEFT_N        8
#define MOTOR_LEFT_N_WPI   10

#define MOTOR_LEFT_P       11
#define MOTOR_LEFT_P_WPI   14

#define MOTOR_RIGHT_P      10
#define MOTOR_RIGHT_P_WPI  12

#define MOTOR_RIGHT_N       9
#define MOTOR_RIGHT_N_WPI  13

int MOTOR_MAPPING[][NUM_MOTORS] = {{MOTOR_LEFT_P_WPI, MOTOR_LEFT_N_WPI}, {MOTOR_RIGHT_P_WPI,MOTOR_RIGHT_N_WPI}};


int motors_init() {
  pinMode(MOTOR_EN_PIN_WPI, PWM_OUTPUT);  /* set GPIO as output */
  digitalWrite(MOTOR_EN_PIN_WPI, 0);
  pinMode(MOTOR_LEFT_P_WPI, PWM_OUTPUT);
  pinMode(MOTOR_LEFT_N_WPI, PWM_OUTPUT);
  pinMode(MOTOR_RIGHT_P_WPI, PWM_OUTPUT);
  pinMode(MOTOR_RIGHT_N_WPI, PWM_OUTPUT);

  softPwmCreate(MOTOR_LEFT_P_WPI,0,100);
  softPwmCreate(MOTOR_LEFT_N_WPI,0,100);
  softPwmCreate(MOTOR_RIGHT_P_WPI,0,100);
  softPwmCreate(MOTOR_RIGHT_N_WPI,0,100);
  return 0;
}

void motors_enable() {
  trilobot_driver_motors_set_speed(0, 0);
  trilobot_driver_motors_set_speed(1, 0);
  digitalWrite(MOTOR_EN_PIN_WPI, 0);
}

// s is in -100:100 range
void trilobot_driver_motors_set_speed(uint8_t motor, int8_t s) {
  if (s>100) s=100;
  if (s<-100) s=-100;
  digitalWrite(MOTOR_EN_PIN_WPI, 1);
  if (s > 0) {
      softPwmWrite (MOTOR_MAPPING[motor][0], 100);
      softPwmWrite (MOTOR_MAPPING[motor][1], 100-s);
    }
  else {
      softPwmWrite (MOTOR_MAPPING[motor][0], 100+s);
      softPwmWrite (MOTOR_MAPPING[motor][1], 100);
  }
}

void trilobot_driver_motors_disable() {
  digitalWrite(MOTOR_EN_PIN_WPI, 0);
  softPwmWrite (MOTOR_LEFT_P_WPI, 0);
  softPwmWrite (MOTOR_LEFT_N_WPI, 0);
  softPwmWrite (MOTOR_RIGHT_P_WPI, 0);
  softPwmWrite (MOTOR_RIGHT_N_WPI, 0);
}

