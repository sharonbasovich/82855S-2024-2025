#include "main.h"
#include "json.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "logging.hpp"
#include <iostream>
#include <sys/_intsup.h>
#include "config.h"

// create an imu on port 17
//pros::Imu imu(17);

//assign ports
pros::Optical Optic(0);
pros::Distance Dist1(0);
pros::Distance Dist2(0);
bool teamColour = 0;//0 for red, 1 for blue
pros::Motor wallStakeArm(0);
pros::adi::Pneumatics clampPistonL('a', false);
pros::adi::Pneumatics clampPistonR('a', false);
pros::adi::Pneumatics Eject('a', false); //are we even implementing this
bool outake = 0; bool intake = 0;



// create an optical shaft encoder connected to ports 'A' and 'B'
//pros::adi::Encoder adi_encoder('A', 'B');

// create a v5 rotation sensor on port 7
//pros::Rotation rotation_sensor(7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,               // left motor group
                              &right_mg,              // right motor group
                              12,                         // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2    // horizontal drift is 2 (for now)
);

//lemlib::TrackingWheel horizontal_tracking_wheel(&adi_encoder, lemlib::Omniwheel::NEW_275, -6.5);

//lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_275, 0);

/*lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we have none
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);*/

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
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
/*
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,            // odometry sensors
                        &throttle_curve, &steer_curve);
*/
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  /*pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });*/
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
void autonomous() {}

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

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        //buttons
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            intake = (intake^1); outake = 0;
        }
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            outake = (outake^1); intake = 0;
        }
        
        int distToGoal = Dist2.get();
        //int confidenceToGoal = Dist2.get_confidence();

        bool mechDetach = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool mechClose = (distToGoal <= 10/* && confidenceToGoal > 50 */&& !mechDetach); //actiate mech if it detects a goal

        bool wallStakeFor = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool wallStakeBack = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        //bool hanglock = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        if(mechClose){
            clampPistonL.extend();
            clampPistonR.extend();
        }
        else{
            clampPistonL.retract();
            clampPistonR.retract();
        }

        if(wallStakeFor||wallStakeBack){
            int wallStakePower = wallStakeFor - wallStakeBack; //this should return 1 for forwards, -1 for backwards
            wallStakeArm.move(127*wallStakePower);

        }

        if(intake||outake){
            int intakePower = intake - outake;  //this should return 1 for forwards, -1 for backwards
            intake_motor.move(127 * intakePower);
            
            /*uncomment if were using this
            if(ejectRing()){
                Eject.extend();
                pros::delay(10);
                Eject.retract();
            }
            */
        }


        // delay to save resources
        pros::delay(25);
    }
}

bool ejectRing() {
        //optical sensor
    double hue = Optic.get_hue(); 
    bool redOrBlu = 0; //0 for red, 1 for blue
        
    if( std::min(hue, 360-hue)       >    abs(hue-180)){
        //distance from 0 or 360     //distance from 180
        redOrBlu = 1;
        //if the distance from red areas is larger than distance from blue areas, it must be blue, default is red
    }
          
    double saturation = Optic.get_saturation(); //these probably arent necessary
    double brightness = Optic.get_brightness(); //these probably arent necessary

        //distance sensor
    int ringIntakeDist = Dist1.get();
    //int ringPresenceConfidence = Dist1.get_confidence();

    return (ringIntakeDist<=20/*&&ringPresenceConfidence>=32)*/&&(teamColour!=redOrBlu));
    //returns true if the ring is the wrong colour
}