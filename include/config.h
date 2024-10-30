

#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#define LEFT_FRONT_MOTOR_PORT 20
#define LEFT_MIDDLE_MOTOR_PORT 19
#define LEFT_BACK_MOTOR_PORT 8
#define RIGHT_FRONT_MOTOR_PORT -4
#define RIGHT_MIDDLE_MOTOR_PORT 20
#define RIGHT_BACK_MOTOR_PORT -20

#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

inline pros::Motor left_front_motor(LEFT_FRONT_MOTOR_PORT);
inline pros::Motor left_middle_motor(LEFT_MIDDLE_MOTOR_PORT);
inline pros::Motor left_back_motor(LEFT_BACK_MOTOR_PORT);
inline pros::Motor right_front_motor(RIGHT_FRONT_MOTOR_PORT);
inline pros::Motor right_middle_motor(RIGHT_MIDDLE_MOTOR_PORT);
inline pros::Motor right_back_motor(RIGHT_BACK_MOTOR_PORT);

inline pros::MotorGroup left_mg({LEFT_FRONT_MOTOR_PORT,LEFT_MIDDLE_MOTOR_PORT,LEFT_BACK_MOTOR_PORT},pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_MOTOR_PORT,RIGHT_MIDDLE_MOTOR_PORT, RIGHT_BACK_MOTOR_PORT},pros::MotorGearset::blue);

inline pros::Controller master(pros::E_CONTROLLER_MASTER);