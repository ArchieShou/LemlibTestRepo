#include "main.h"
#include "lemlib/api.hpp"

// controller

// drive motors
pros::Motor lF(3, pros::E_MOTOR_GEARSET_06, true); // left front motor. port 12, reversed
pros::Motor lM(12, pros::E_MOTOR_GEARSET_06, false); // left middle motor. port 11, reversed
pros::Motor lB(13, pros::E_MOTOR_GEARSET_06, true); // left back motor. port 1, reversed
pros::Motor rF(19, pros::E_MOTOR_GEARSET_06, false); // right front motor. port 2
pros::Motor rM(20, pros::E_MOTOR_GEARSET_06, true); // right middle motor. port 11
pros::Motor rB(21, pros::E_MOTOR_GEARSET_06, false); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

//DT Set Up
lemlib::Drivetrain_t drivetrain {
    &leftMotors, // left drivetrain motors
    &rightMotors, // right drivetrain motors
    11, // track width
    3.25, // wheel diameter
    400 // wheel rpm
};
pros::Controller master(pros::E_CONTROLLER_MASTER);
// Inertial Sensor on port 2
pros::Imu imu1(7);

// tracking wheels
// Encoder
pros::Rotation l_enc(16, true); // horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
pros::Rotation r_enc(15, false);
//pros::Rotation h_enc(15, true);


// horizontal tracking wheel. 2.00" diameter, 3.7" offset, back of the robot (negative)
lemlib::TrackingWheel l_tracking_wheel(&l_enc, 2, -4, 1);
lemlib::TrackingWheel r_tracking_wheel(&r_enc, 2, 4, 1);
// //lemlib::TrackingWheel h_tracking_wheel(&h_enc, 2.75, 0, 1);

// odometry struct
lemlib::OdomSensors_t sensors {
    &l_tracking_wheel, //&l_tracking_wheel, // vertical tracking wheel 1
    &r_tracking_wheel, // vertical tracking wheel 2
    nullptr, //&h_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu1, // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// Set up screen
void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
    pros::Task screenTask(screen); // create a task to print the position to the screen
	//chassis.setPose(5.2, 10.333, 87); // X: 5.2, Y: 10.333, Heading: 87
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0

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


// Auton Lists

void auton_Skills() {
	// chassis.setPose(5.2, 10.333, 45); // X: 5.2, Y: 10.333, Heading: 87

}

void far_side_auton() {}

void close_side_auton() {}

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

void autonomous() {
	chassis.follow("straightPath.txt", 10000, 5);
    // chassis.follow("paht2.txt", 10000, 5, true);
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
void opcontrol() {
	
	pros::lcd::print(3, "op");
	if (master.get_digital_new_press(DIGITAL_A)){
        pros::lcd::print(4, "pressed");
        chassis.turnTo(10,10,1000);
        pros::lcd::print(5, "Turned");

    }
}

