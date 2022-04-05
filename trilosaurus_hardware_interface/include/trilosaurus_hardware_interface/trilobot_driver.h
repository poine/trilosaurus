#ifndef TRILOSAURUS_HARDWARE_INTERFACE__TRILOBOT_DRIVER_HPP_
#define TRILOSAURUS_HARDWARE_INTERFACE__TRILOBOT_DRIVER_HPP_

// Motor names
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1
#define NUM_MOTORS  2

#include <stdint.h>

int trilobot_driver_setup_hardware();
void trilobot_driver_motors_set_speed(uint8_t motor, int8_t s);
void trilobot_driver_motors_disable();

#endif // TRILOSAURUS_HARDWARE_INTERFACE__TRILOBOT_DRIVER_HPP_
