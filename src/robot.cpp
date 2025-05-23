#include "pros/motor_group.hpp"
#include "robot.h"

// Initialize left motor group with three VEX 5.5W motors on ports 1, 2, 3
pros::v5::MotorGroup left_mg({1, 2, 3});

// Initialize right motor group with three VEX 5.5W motors on ports 4, 5, 6
pros::v5::MotorGroup right_mg({4, 5, 6});
