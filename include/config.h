

#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

//drive motors
#define LEFT_FRONT_DRIVE -3
#define LEFT_MIDDLE_DRIVE -2
#define LEFT_BACK_DRIVE -1

#define RIGHT_FRONT_DRIVE 8
#define RIGHT_MIDDLE_DRIVE 9
#define RIGHT_BACK_DRIVE 10

//drive config
#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

//intake motors
#define INTAKE_LEFT -6
#define INTAKE_RIGHT -7

//pneumatics
#define DOINKER 'G'
#define CLAMP 'H'

//sensors
#define WALL_ROTATION 20
#define IMU 19
#define VERTICAL_ODOM -12
#define HORIZONTAL_ODOM 18

//ring hold
#define RING_DISTANCE 5
//**IN MILLIMETERS** the value that the ring must be closer than to be detected
#define RING_DISTANCE_THRESHOLD 100

//color sort
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

inline pros::Motor intake_right(INTAKE_RIGHT);
inline pros::Motor intake_left(INTAKE_LEFT);

inline pros::Imu imu(IMU);

inline pros::adi::Pneumatics clamp(CLAMP, false);
inline pros::adi::Pneumatics doinker(DOINKER, false);


inline pros::Distance ring_distance(RING_DISTANCE);
inline pros::Optical ring_color(RING_COLOR);

inline pros::Rotation vertical_odom(VERTICAL_ODOM);
inline pros::Rotation horizontal_odom(HORIZONTAL_ODOM);

