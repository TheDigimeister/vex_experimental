#pragma once
#include "pros/motor_group.hpp"

// Motor ports for V5 brain
#define LEFT_MOTOR_PORT_1 1
#define LEFT_MOTOR_PORT_2 2
#define LEFT_MOTOR_PORT_3 3
#define RIGHT_MOTOR_PORT_1 4
#define RIGHT_MOTOR_PORT_2 5
#define RIGHT_MOTOR_PORT_3 6

// Extern declarations for motor groups
extern pros::v5::MotorGroup left_mg;
extern pros::v5::MotorGroup right_mg;
