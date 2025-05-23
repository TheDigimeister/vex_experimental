#include "autons.h"
#include <iostream>
#include "pros/llemu.hpp"

int auton_flag = 0; // 0 = one, 1 = two, 2 = skills
const char* auton_names[] = {"Routine One", "Routine Two", "Skills"};
const int num_autons = 3;

void auton_selector_ui() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Select Auton:");
    pros::lcd::set_text(1, auton_names[auton_flag]);
    pros::lcd::register_btn0_cb([]() {
        auton_flag = (auton_flag - 1 + num_autons) % num_autons;
        pros::lcd::set_text(1, auton_names[auton_flag]);
    });
    pros::lcd::register_btn2_cb([]() {
        auton_flag = (auton_flag + 1) % num_autons;
        pros::lcd::set_text(1, auton_names[auton_flag]);
    });
}

void auton_routine_one() {
    std::cout << "Running Auton Routine One" << std::endl;
    // Add robot actions here
}

void auton_routine_two() {
    std::cout << "Running Auton Routine Two" << std::endl;
    // Add robot actions here
}

void auton_routine_skills() {
    std::cout << "Running Skills Auton Routine" << std::endl;
    // Add robot actions here
}
