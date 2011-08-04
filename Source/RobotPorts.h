#ifndef ROBOT_PORTS_H
#define ROBOT_PORTS_H

#include <types/vxTypes.h>

//Constants defined for various motor, analog, digital, solenoid,
//and compressor port values.

//MOTOR PORTS
static const uint32_t SPOOL_MOTOR_A_PWM_PORT = 1;
static const uint32_t SPOOL_MOTOR_B_PWM_PORT = 2;

static const uint32_t RIGHT_MOTOR_A_PWM_PORT = 3;
static const uint32_t RIGHT_MOTOR_B_PWM_PORT = 4;
static const uint32_t LEFT_MOTOR_A_PWM_PORT = 5;
static const uint32_t LEFT_MOTOR_B_PWM_PORT = 6;

static const uint32_t ARM_MOTOR_PWM_PORT = 7;
static const uint32_t TOP_ROLLER_MOTOR_PWM_PORT = 8;
static const uint32_t BOTTOM_ROLLER_MOTOR_PWM_PORT = 9;

//ANALOG PORTS
static const uint32_t ARM_POT_PORT = 7;

//DIGITAL PORTS
static const uint32_t LEFT_WHEEL_ENCODER_A_PWM_PORT = 3;
static const uint32_t LEFT_WHEEL_ENCODER_B_PWM_PORT = 4;
static const uint32_t RIGHT_WHEEL_ENCODER_A_PWM_PORT = 1;
static const uint32_t RIGHT_WHEEL_ENCODER_B_PWM_PORT = 2;

static const uint32_t ELEVATOR_ENCODER_A_PWM_PORT = 5;
static const uint32_t ELEVATOR_ENCODER_B_PWM_PORT = 6;
static const uint32_t ELEVATOR_SECOND_STAGE_LIMIT_SWITCH = 10;

static const uint32_t TOP_ROLLER_ENCODER_A_PWM_PORT = 7;
static const uint32_t TOP_ROLLER_ENCODER_B_PWM_PORT = 8;

static const uint32_t BOTTOM_ROLLER_ENCODER_A_PWM_PORT = 9;
static const uint32_t BOTTOM_ROLLER_ENCODER_B_PWM_PORT = 10;

static const uint32_t GYRO_PORT = 1;

static const uint32_t CARRIAGE_LIMIT_SWITCH = 12;
static const uint32_t ROLLER_LIMIT_SWITCH=13;

// SOLENOID AND COMPORESSOR PORTS
static const uint32_t ROLLER_HIGH_SOLENOID_CHAN=1;
static const uint32_t ROLLER_LOW_SOLENOID_CHAN=2;
static const uint32_t GEAR_SHIFT_SOLENOID_CHAN=3;
static const uint32_t MINIBOT_RELEASE_SOLENOID = 4;
static const uint32_t MINIBOT_DEPLOY_SOLENOID = 5;
static const uint32_t COMPRESSOR_PRESSURE_SWITCH_CHAN=14;
static const uint32_t COMPRESSOR_RELAY_CHAN=1;


#endif
