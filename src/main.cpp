#include "main.h"
#include "json.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include <iostream>
//#include <sys/_intsup.h>
#include "config.h"

bool intake = 0;
bool outake = 0;
int wallStakePos = 0;
bool doinkPosition = 0;
bool clamp = 0;
bool doinker = 0;
int wallStakeAngle = 0;

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,                   // left motor group
                              &right_mg,                  // right motor group
                              12,                         // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450,                        // drivetrain rpm is 450
                              2                           // horizontal drift is 2 (for now)
);

// lemlib::TrackingWheel horizontal_tracking_wheel(&adi_encoder, lemlib::Omniwheel::NEW_275, -6.5);

// lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_275, 0);

/*lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we have none
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);*/

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu_sensor);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              3,   // derivative gain (kD)
                                              3,   // anti windup
                                              1,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              10,  // derivative gain (kD)
                                              3,   // anti windup
                                              1,   // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

// // create the chassis
// lemlib::Chassis chassis(drivetrain,         // drivetrain settings
//                         lateral_controller, // lateral PID settings
//                         angular_controller, // angular PID settings
//                         sensors             // odometry sensors
// );

// lemlib::ExpoDriveCurve
//     steer_curve(3,    // joystick deadband out of 127
//                 10,   // minimum output where drivetrain will move out of 127
//                 1.019 // expo curve gain
//     );

// lemlib::ExpoDriveCurve
//     throttle_curve(3,    // joystick deadband out of 127
//                    10,   // minimum output where drivetrain will move out of 127
//                    1.019 // expo curve gain
//     );

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
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    // wall_stake_rotation.reset_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
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
void autonomous()
{
    // int delta = 1000;
    // chassis.moveToPoint(0, 43, 4000);
    // pros::delay(delta);
    // chassis.turnToHeading(30, 4000);
    // pros::delay(delta);
    // clamp_piston.extend();
    // pros::delay(100);
    // chassis.moveToPoint(0, 10, 4000, {.forwards = false});
    // pros::delay(delta);
    // clamp_piston.retract();
    // pros::delay(100);
    // chassis.turnToHeading(225, 4000);
    // pros::delay(delta);
    // chassis.moveToPoint(0, 15, 4000, {.forwards = false});
    // pros::delay(delta);
    // doinker_piston.extend();
    // pros::delay(100);
    // intake_motor.move(127);
    pros::delay(100);
    chassis.moveToPoint(0, -30, 4000, {.forwards = false, .maxSpeed=60});
    pros::delay(2000);
    doinker_piston.extend();
    pros::delay(300);
    intake_motor.move(-127);
    pros::delay(3000);
    // intake_motor.move(0);
    // pros::delay(300);
    // chassis.turnToHeading(290, 4000);
    // pros::delay(300);
    // chassis.moveToPoint(20, 20, 4000, {.maxSpeed=60});
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

void opcontrol()
{
    // button guide:
    /*
    joysticks: drivetrain
    L2: doinker
    L1: goal clamp
    R2: home position lady brown
    R1: move lady brown
    */
    // loop forever
    while (true)
    {

        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade((leftY), rightX);

        // buttons

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

//ladybrown
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            wallStakePos = 0;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            if(wallStakePos==0||wallStakePos==255)wallStakePos = 128;
            else wallStakePos = 255;

        }

        wall_stake_motor.move_absolute(wallStakePos, 100);



        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        // {
        //     wallStakeIdle = 1;
        //     wallStakeGrab = 0;
        //     wallStakeSwing = 0;
        // } // home position wall stake

        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        // {

        //     if (wallStakeIdle)
        //     {
        //         wallStakeGrab = 1;
        //         wallStakeSwing = 0;
        //     }
        //     else if (wallStakeGrab)
        //     {
        //         wallStakeGrab = 0;
        //         wallStakeSwing = 1;
        //     }
        //     else if (wallStakeSwing)
        //     {
        //         wallStakeGrab = 1;
        //         wallStakeSwing = 0;
        //     }
        //     wallStakeIdle = 0;
        // }

        // // calculate required angle
        // wallStakeAngle = (wallStakeGrab * LOAD_ANGLE) + (wallStakeSwing * SCORE_ANGLE);
        // int low = (wall_stake_rotation.get_position() / 100) - 10;
        // int high = (wall_stake_rotation.get_position() / 100) + 10;
        // bool onTarget = low <= wallStakeAngle && wallStakeAngle <= high;

        // if (onTarget)
        // {
        //     wall_stake_motor.move(0);
        // } else {
        //     if (wallStakeAngle > low)
        //     {
        //         wall_stake_motor.move(127);
        //         // go backwards
        //     }
        //     else
        //     {
        //         wall_stake_motor.move(127);
        //         // go forwards
        //     }
        // }

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
        intake_motor.move(127 * intakePower);
        intake_half_motor.move(127 * intakePower);

        // int distToGoal = Dist2.get();
        //  int confidenceToGoal = Dist2.get_confidence();

        // bool mechDetach = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        // bool mechClose = (distToGoal <= 30 && !mechDetach); // actiate mech if it detects a goal

        // bool hanglock = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        /*
                if (mechClose)
                {
                    MogoPiston1.extend();
                    MogoPiston2.extend();
                }
                else
                {
                    MogoPiston1.retract();
                    MogoPiston2.retract();
                }
                */
        // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && doinkerCooldown == 0)
        // {
        //     doinkPosition = !doinkPosition;
        //     doinkerCooldown = 10;
        //     // if(doinkPosition) doinkerArm.extend();
        //     // else doinkerArm.retract();
        // }

        /*
         if (wallStakeIdle || wallStakeGrab || wallStakeSwing)
         {
             int position = wallStakeGrab*(1) + wallStakeIdle*(1) + wallStakeSwing*(1);
             wall_stake_arm.move_absolute(position, 100);
         }*/

        // if (intake || outake)
        //{

        /*uncomment if were using this
        if(ejectRing()){
            Eject.extend();
            pros::delay(10);
            Eject.retract();
        }
        */
        //}

        // // delay to save resources
        // intakeCooldown = std::max(intakeCooldown - 1, 0);
        // outakeCooldown = std::max(outakeCooldown - 1, 0);
        // doinkerCooldown = std::max(doinkerCooldown - 1, 0);

        pros::Task screen_task([&]()
                               {
                                   //                            // print robot location to the brain screen
                                   //                            //   pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
                                   //                            //   pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
                                   //                            //   pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
                                   //                            pros::lcd::print(0, "Readout: %d", wall_stake_rotation.get_position());
                                   //                            pros::lcd::print(1, "Low: %d", low);
                                //    pros::lcd::print(0, "TEST");
                                      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
                                      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
                                      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
                                      pros::lcd::print(3, "leftY: %f", leftY);
                                      //                            pros::lcd::print(2, "High: %d", high);
                                      //                            pros::lcd::print(3, "On target: %d", onTarget);
                               });

        pros::delay(25);
    }
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
    int ringIntakeDist = Dist1.get();
    // int ringPresenceConfidence = Dist1.get_confidence();

    return (ringIntakeDist <= 20 && (teamColour != redOrBlu));
    // returns true if the ring is the wrong colour
}*/