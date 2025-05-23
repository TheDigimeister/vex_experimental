#pragma once

// Example autonomous routines
typedef void (*AutonRoutine)();

void auton_routine_one();
void auton_routine_two();
void auton_routine_skills();

// UI for selecting auton
void auton_selector_ui();
extern int auton_flag; // 0 = one, 1 = two, 2 = skills
