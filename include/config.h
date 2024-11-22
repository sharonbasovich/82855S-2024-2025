

#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#define LEFT_FRONT_MOTOR_PORT -3
#define LEFT_MIDDLE_MOTOR_PORT -2
#define LEFT_BACK_MOTOR_PORT -1
#define RIGHT_FRONT_MOTOR_PORT 8
#define RIGHT_MIDDLE_MOTOR_PORT 9
#define RIGHT_BACK_MOTOR_PORT 10
#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

#define INTAKE_MOTOR -6
#define INTAKE_HALF_MOTOR -7

#define DOINKER_PORT 'G'
#define CLAMP_PORT 'H'

#define WALL_STAKE_ROTATION_PORT 20

//#define LOAD_ANGLE 20
//#define SCORE_ANGLE 140

#define IMU 19
#define ODOM_VERTICAL 12
#define ODOM_HORIZONTAL 18





inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::Motor left_front_motor(LEFT_FRONT_MOTOR_PORT);
inline pros::Motor left_middle_motor(LEFT_MIDDLE_MOTOR_PORT);
inline pros::Motor left_back_motor(LEFT_BACK_MOTOR_PORT);
inline pros::Motor right_front_motor(RIGHT_FRONT_MOTOR_PORT);
inline pros::Motor right_middle_motor(RIGHT_MIDDLE_MOTOR_PORT);
inline pros::Motor right_back_motor(RIGHT_BACK_MOTOR_PORT);
//inline pros::Motor wall_stake_motor(WALL_STAKE_PORT);
inline pros::MotorGroup left_mg({LEFT_FRONT_MOTOR_PORT,LEFT_MIDDLE_MOTOR_PORT,LEFT_BACK_MOTOR_PORT},pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_MOTOR_PORT,RIGHT_MIDDLE_MOTOR_PORT, RIGHT_BACK_MOTOR_PORT},pros::MotorGearset::blue);


inline pros::Rotation wall_stake_rotation(WALL_STAKE_ROTATION_PORT);
inline pros::Motor intake_half_motor(INTAKE_HALF_MOTOR);

inline pros::Motor intake_motor(INTAKE_MOTOR);

inline pros::Motor wall_stake_motor(20);

inline pros::Imu imu_sensor(IMU);

inline pros::adi::Pneumatics clamp_piston(CLAMP_PORT, false);
inline pros::adi::Pneumatics doinker_piston(DOINKER_PORT, false);

inline pros::Optical Optic(0);
inline pros::Distance Dist1(0);
