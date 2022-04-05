#include <trilosaurus_hardware_interface/trilobot_driver.h>

#include <wiringPi.h> 
#include <softPwm.h>

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

int trilobot_driver_setup_hardware() {
  if (wiringPiSetup() == -1) 	          /* initialize wiringPi setup */
    return -1;
  pinMode(MOTOR_EN_PIN_WPI, PWM_OUTPUT);  /* set GPIO as output */
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

