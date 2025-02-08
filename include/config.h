

#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// drive motors
#define LEFT_FRONT_DRIVE -21 //c
#define LEFT_MIDDLE_DRIVE -18 //c
#define LEFT_BACK_DRIVE -19 //c

#define RIGHT_FRONT_DRIVE 14 //c
#define RIGHT_MIDDLE_DRIVE 15 //c
#define RIGHT_BACK_DRIVE 16 //c

// drive config
#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

// intake motors
#define INTAKE_PREROLLER 17 //c
#define INTAKE_HOOKS -20 //c

// wall stake motors
#define WALL_MOTOR 3 //c

// pneumatics
#define DOINKER 'G'
#define CLAMP 'C'
#define RUSH 'A'
#define LIFT 'E'

// sensors
#define WALL_ROTATION 15
#define IMU 10             // c
#define VERTICAL_ODOM 13   // c
#define HORIZONTAL_ODOM 12 // c

// ring hold
#define RING_DISTANCE 5
//**IN MILLIMETERS** the value that the ring must be closer than to be detected
#define RING_DISTANCE_THRESHOLD 100

// color sort
#define RING_COLOR 6
//*IN MILLISECONDS* the time that it takes for the ring to reach the top of the hooks from when the color sensor detects it*/
#define COLOR_TIME 400
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::Motor left_front_drive(LEFT_FRONT_DRIVE);
inline pros::Motor left_middle_drive(LEFT_MIDDLE_DRIVE);
inline pros::Motor left_back_drive(LEFT_BACK_DRIVE);
inline pros::Motor right_front_drive(RIGHT_FRONT_DRIVE);
inline pros::Motor right_middle_drive(RIGHT_MIDDLE_DRIVE);
inline pros::Motor right_back_drive(RIGHT_BACK_DRIVE);

inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);

inline pros::Rotation wall_rotation(WALL_ROTATION);
inline pros::Motor wall_motor(WALL_MOTOR);

inline pros::Motor intake_hooks(INTAKE_HOOKS);
inline pros::Motor intake_preroller(INTAKE_PREROLLER);

inline pros::Imu imu(IMU);

inline pros::adi::Pneumatics clamp(CLAMP, false);
inline pros::adi::Pneumatics doinker(DOINKER, false);
inline pros::adi::Pneumatics rush(RUSH, false);
inline pros::adi::Pneumatics lift(LIFT, false);

inline pros::Distance ring_distance(RING_DISTANCE);
inline pros::Optical ring_color(RING_COLOR);

inline pros::Rotation vertical_odom(VERTICAL_ODOM);
inline pros::Rotation horizontal_odom(HORIZONTAL_ODOM);
