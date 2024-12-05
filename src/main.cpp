#include "main.h"
#include "json.hpp"
#include "logging.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include <iostream>
// #include <sys/_intsup.h>
#include "config.h"

bool intake = 0;
bool outake = 0;

bool clamp = 0;

bool doinkPosition = 0;
bool doinker = 0;

bool teamColour = 0; // 0 for red

// Ladybrown

int wallState = 0;
int checkState = -1;
int wallStakeAngle = 0;
double wallStakePos = 0;
double currentPosition = 0;
double lastPosition = -1000;
bool intakeSlowdown = false;
bool resetWallStake = false;
bool extendFull = false;
bool PPosition = false;
int primedPosition = 0;
bool stationary = false;

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,                   // left motor group
                              &right_mg,                  // right motor group
                              12,                         // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450,                        // drivetrain rpm is 450
                              2                           // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, lemlib::Omniwheel::NEW_275, 1);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, lemlib::Omniwheel::NEW_275, -1.5);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,   // vertical tracking wheel 1, set to null
    nullptr,                    // vertical tracking wheel 2, set to nullptr as we have none
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr,                    // horizontal tracking wheel 2, set to nullptr as we don't have a
                                // second one
    &imu                        // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(8,  // proportional gain (kP)
                                              0,  // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0,  // anti windup
                                              0,  // small error range, in inches
                                              0,  // small error range timeout, in milliseconds
                                              0,  // large error range, in inches
                                              0,  // large error range timeout, in milliseconds
                                              100 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(6,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              18,  // derivative gain (kD)
                                              3,   // anti windup
                                              1,   // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
                                            // &throttle_curve, &steer_curve
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors

    std::cout << "foxglove" << std::endl; //flag to indicate new session

    pros::Task controller_task(updateController); //prints to controller, comment out to get back default ui
    updateScreen(); //prints to brain screen
}

pros::Task screen_task;

void updateScreen() {
    screen_task = pros::Task([]()
                             {
                                 while (true)
                                 {
                                    //up to 8 lines (0-7)
                                    //use %d for integer and boolean
                                    //use %f for floating point
                                    //if the wrong one is used could have 0 or huge random output
                                    //  pros::lcd::print(<0-7>, "var: <%d or %f>", <valueToPrint>);
                                     pros::delay(20);
                                 } });
}

void updateController()
{
    while (true)
    {
        // prints to controller up to 3 rows (this is the max possible)
        master.clear();

        // up to 3 lines (0-2)
        // use %d for integer and boolean
        // use %f for floating point
        // if the wrong one is used could have 0 or huge random output
        master.print(0, 0, "check: %d", checkState);
        master.print(1, 0, "wall %d", wallState);
        master.print(2, 0, "current %d", ring_distance_sensor.get());

        pros::delay(50);
    }
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void getCurrentHeading()
{
    while (true)
    {
        pros::lcd::print(5, "%f", fmod(imu.get_rotation(), 360.0));
    }
}

void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

bool hold = false;

void holdRing()
{
    while (hold)
    {
        if (ring_distance_sensor.get() < 100)
        {
            intake_motor.move(0);
            pros::delay(10);
            intake_half_motor.move(0);
            pros::delay(10);
            break;
        }
    }
}

void intakeForward()
{
    intake_motor.move(127);
    pros::delay(20);
    intake_half_motor.move(127);
    pros::delay(20);
}

void intakeBackward()
{
    intake_motor.move(-127);
    pros::delay(20);
    intake_half_motor.move(-127);
    pros::delay(20);
}

void intakeStop()
{
    intake_motor.move(0);
    pros::delay(20);
    intake_half_motor.move(0);
    pros::delay(20);
}

void autonomous()
{


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void ladyBrown()
{

    bool full = true;
    while (true)
    {
 
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        {
            if (wallState == 0)
            {
                wall_stake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                wall_stake_motor.move(30);
                pros::delay(225);
                wall_stake_motor.move(0);
                wallState += 1;
            }
            else if (wallState == 1)
            {

                wall_stake_motor.move(127);

                wallState += 1;
            }
            else if (wallState == 2)
            {
                wall_stake_motor.move(-127);
                wallState = 0;
                pros::delay(1500);
                wall_stake_motor.move(0);
            }
        }

        pros::delay(25);
    }
}



void opcontrol()
{

    int x = 0;
    while (true)
    {
        ExampleStruct payload{x};
        Message msg{"my_topic_name", payload};
        std::cout << static_cast<json>(msg) << std::flush;

        pros::delay(10);
        x++;
    }
    /*
    while (true)
    {
        // get left y and right x positions

        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade((leftY), rightX);

        // buttons

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake_motor.move(-50);
            intake_half_motor.move(-50);
            pros::delay(50);
            intake_motor.move(0);
            intake_half_motor.move(0);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            intake = !intake;
            outake = 0;
        } // activate intake
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            outake = !outake;
            intake = 0;
        } // activate outake

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {

            clamp = !clamp;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
        {
            doinker = !doinker;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        {
        }

        // // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        // // {

        //     intakeSlowdown = !intakeSlowdown;
        //

        if (doinker)
        {
            doinker_piston.extend();
        }
        else
        {
            doinker_piston.retract();
        }

        if (clamp)
        {
            clamp_piston.extend();
        }
        else
        {
            clamp_piston.retract();
        }

        int intakePower = intake - outake; // this should return 1 for forwards, -1 for backwards
        intakePower = intakePower * 127 / (1 + 5 * intakeSlowdown);
        intake_motor.move(intakePower);
        intake_half_motor.move(intakePower);

        pros::delay(20);
    }
    */
}

/*
bool ejectRing()
{
    // optical sensor
    double hue = Optic.get_hue();
    bool redOrBlu = 0; // 0 for red, 1 for blue

    if (std::min(hue, 360 - hue) > abs(hue - 180))
    {
        // distance from 0 or 360     //distance from 180
        redOrBlu = 1;
        // if the distance from red areas is larger than distance from blue areas, it must be blue, default is red
    }

    double saturation = Optic.get_saturation(); // these probably arent necessary
    double brightness = Optic.get_brightness(); // these probably arent necessary

    // distance sensor
    int ringIntakeDist = ring_distance_sensor.get();
    // int ringPresenceConfidence = Dist1.get_confidence();

    return (ringIntakeDist <= 20 && (teamColour != redOrBlu));
    // returns true if the ring is the wrong colour
}
*/