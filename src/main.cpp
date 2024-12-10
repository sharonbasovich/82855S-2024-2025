#include "main.h"
#include "json.hpp"
#include "logging.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include <iostream>
// #include <sys/_intsup.h>
#include "config.h"
#include "liblvgl/lvgl.h"
#include "selector.hpp"

// Structure to hold button label and color information
typedef struct
{
    const char *label; // Button name
    bool is_red;       // Flag to determine button color (true for red, false for blue)
} button_info_t;

// Array of button labels and their colors
// True makes the button red, and false makes it blue
button_info_t button_info[] = {
    {PN1, PC1},
    {PN2, PC2},
    {PN3, PC3},
    {PN4, PC4},
    {PN5, PC5},
    {PN6, PC6}};

// the number of buttons to be created based on the array above
#define NUM_BUTTONS (sizeof(button_info) / sizeof(button_info[0]))

static lv_obj_t *readout_label;

// The brain screen is 480 by 272 pixels
// Size of the button
int button_x = BUTTON_X;
int button_y = BUTTON_Y;

lv_coord_t x_start = X_START;            // Starting x-coordinate for buttons
lv_coord_t y_start = Y_START;            // Starting y-coordinate for buttons
lv_coord_t x_offset = button_x + OFFSET; // Horizontal offset between buttons
lv_coord_t y_offset = button_y + OFFSET; // Vertical offset between buttons
lv_coord_t num_columns = NUM_COLUMNS;    // Number of columns in the grid

// Button color definitions
#define BLUE_COLOR lv_color_hex(0x0000FF)
#define RED_COLOR lv_color_hex(0xFF0000)
lv_style_t blue_style;
lv_style_t red_style;

// Callback function for button events
static void btn_event_cb(lv_event_t *e)
{
    // Check if the button was pressed
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        // Retrieve the value associated with the button
        int value = (intptr_t)lv_obj_get_user_data(lv_event_get_target(e));

        // Determine the color of the button from the array
        const char *color = button_info[value - 1].is_red ? "Red" : "Blue";

        // Update the readout with the new value and color
        lv_label_set_text_fmt(readout_label, "Auto: %s     Color: %s", button_info[value - 1].label, color);
    }
}

// Function to abstract the creation of the button
lv_obj_t *create_button(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y, int value, bool is_red)
{
    lv_obj_t *btn = lv_btn_create(parent);      // Create the button
    lv_obj_set_size(btn, button_x, button_y);   // Set button size
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y); // Position the button

    // Create a label for the button text
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);

    // Set the label alignment to left
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    // Remove any padding on the left of the button to optimize space
    lv_obj_set_style_pad_left(btn, 3, 0);

    // Set the button color based on the `is_red` flag from the array
    if (is_red)
    {
        lv_obj_add_style(btn, &red_style, 0);
    }
    else
    {
        lv_obj_add_style(btn, &blue_style, 0);
    }

    // Set the value of the button
    lv_obj_set_user_data(btn, (void *)(intptr_t)value);

    // Attach the event callback
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
    return btn;
}

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



void intakeForward()
{
    intake_left.move(127);
    pros::delay(10);
    intake_right.move(127);
    pros::delay(10);
}

void intakeBackward()
{
    intake_left.move(-127);
    pros::delay(10);
    intake_right.move(-127);
    pros::delay(10);
}

void intakeStop()
{
    intake_left.move(0);
    pros::delay(10);
    intake_right.move(0);
    pros::delay(10);
}

bool hold = false;
bool hasRing = false;

void holdRing()
{
    // detects if ring is held or not
    // hasRing is constantly updated for if a ring is detected or not
    // set hold to true if intake should stop to hold ring when it is detected or false otherwise
    while (true)
    {
        if (ring_distance.get() < RING_DISTANCE_THRESHOLD)
        {
            if (hold)
            {
                intakeStop();
            }
            hasRing = true;
        }
        else
        {
            hasRing = false;
        }
        pros::delay(10);
    }
}

bool isRed = true;
float hue = -1;
bool sort = false;

void colorSort()
{
    // set sort to true when you want the ring sort activated, and set it to false when it should stop
    ring_color.set_led_pwm(100);
    pros::delay(10);
    while (true)
    {
        hue = ring_color.get_hue();
        if (sort && isRed * (240) > (hue - 30) && isRed * (240) < (hue + 30) && ring_distance.get() < RING_DISTANCE_THRESHOLD)
        {
            pros::delay(COLOR_TIME);
            intakeBackward();
            pros::delay(200);
            intakeForward();
        }
        pros::delay(10);
    }
}

void foxglove()
{
    // float a = 0;
    // float b = 0;
    // float c = 0;
    // int x = 0;

    std::cout << "foxglove" << std::endl; // flag to indicate new session
    pros::delay(50);
    while (true)
    {
        lemlib::Pose pose = chassis.getPose();
        Odometry odom = {std::ceil((double)pose.x * 100.0) / 100.0, std::ceil((double)pose.y * 100.0) / 100.0, std::ceil((double)pose.theta * 100.0) / 100.0};
        // Odometry odom = {std::ceil((double)a * 100.0) / 100.0, std::ceil((double)b * 100.0) / 100.0, std::ceil((double)0 * 100.0) / 100.0};

        Message msg{"odometry", odom};
        std::cout << static_cast<json>(msg) << std::flush;
        pros::delay(50);
        // a++;
        // b++;
        // c++;
    }
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
        master.print(2, 0, "current %d", ring_distance.get());

        pros::delay(50);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
    // Initialize button styles for blue
    lv_style_init(&blue_style);
    lv_style_set_bg_color(&blue_style, BLUE_COLOR);

    // Initialize button styles for red
    lv_style_init(&red_style);
    lv_style_set_bg_color(&red_style, RED_COLOR);

    // Create the readout that displays which program is selected
    readout_label = lv_label_create(lv_scr_act());

    // Indicate that no program has been selected yet
    lv_label_set_text(readout_label, "Auto: NONE");

    // Position the readout
    lv_obj_align(readout_label, LV_ALIGN_TOP_LEFT, 10, 10);

    // Create buttons in a grid pattern
    int button_count = 0;
    for (int row = 0; row < 5; row++)
    {
        for (int col = 0; col < num_columns; col++)
        {
            // Stop if all the buttons are created
            if (button_count >= NUM_BUTTONS)
            {
                break;
            }

            lv_coord_t x = x_start + col * x_offset; // Calculate x position
            lv_coord_t y = y_start + row * y_offset; // Calculate y position

            // Get label text and color info from the 2D array
            const char *label_text = button_info[button_count].label;
            bool is_red = button_info[button_count].is_red;
            int button_value = (button_count + 1);

            // Use the previously created button creator with the values for the current button
            create_button(lv_scr_act(), label_text, x, y, button_value, is_red);

            // Track the number of buttons created
            button_count++;
        }
    }
    
    chassis.calibrate();     // calibrate sensors

    pros::Task controller_task(updateController); // prints to controller, comment out to get back default ui
    // updateScreen();                               // prints to brain screen
    
    //cannot use if using auton selector
    /*
    pros::lcd::initialize(); // initialize brain screen
    pros::Task screen_task([&]()
                           {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "hue: %f", hue);
            pros::lcd::print(1, "hasring?: %d", hasRing);
            pros::lcd::print(2, "Theta: %d", 8); // heading
            // delay to save resources
            pros::delay(20);
        } });
    */
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

// void ladyBrown()
// {

//     bool full = true;
//     while (true)
//     {

//         if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
//         {
//             if (wallState == 0)
//             {
//                 wall_stake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//                 wall_stake_motor.move(30);
//                 pros::delay(225);
//                 wall_stake_motor.move(0);
//                 wallState += 1;
//             }
//             else if (wallState == 1)
//             {

//                 wall_stake_motor.move(127);

//                 wallState += 1;
//             }
//             else if (wallState == 2)
//             {
//                 wall_stake_motor.move(-127);
//                 wallState = 0;
//                 pros::delay(1500);
//                 wall_stake_motor.move(0);
//             }
//         }

//         pros::delay(25);
//     }
// }

void opcontrol()
{
    lv_timer_handler(); // Process LVGL tasks
    // pros::Task foxglove_task(foxglove);

    // pros::Task holdring_trak(holdRing);

    pros::Task colorsort_task(colorSort);

    sort = true;
    // while (true)
    // {
    //     ExampleStruct payload{x};
    //     Message msg{"my_topic_name", payload};
    //     std::cout << static_cast<json>(msg) << std::flush;

    //     pros::delay(50);
    //     x++;
    // }

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
            intake_left.move(-50);
            intake_right.move(-50);
            pros::delay(50);
            intake_left.move(0);
            intake_right.move(0);
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
        intake_left.move(intakePower);
        intake_right.move(intakePower);

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