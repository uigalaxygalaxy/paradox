#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// create the controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);



// Initialize the Drivetrain:

// Creates a Motor Group with PROS. the number is the port. Negative numbers = reverse
//front, middle, back
pros::MotorGroup left_motors({4, -3, -1}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-9, 8, 10}, pros::MotorGearset::blue);

//IMU = inertial sensor, number = port
pros::Imu imu(2);

//our rotation sensors for odom

pros::Rotation horizontal(11);
pros::Rotation vertical(20);

//intake motor
pros::Motor firstStage(-7);
pros::Motor secondStage(-19);

// define our odom wheels (sensor, wheel, tracking offset)
// how far away is the odom wheel from the center of the bot? for ex. if a horizontal sensor is 4 inches below the center, it would be -4.
lemlib::TrackingWheel horizontal_odom(&horizontal, lemlib::Omniwheel::NEW_275, 0);
lemlib::TrackingWheel vertical_odom(&vertical, lemlib::Omniwheel::NEW_275, 0);

// init odomsensors to feed the chassis class
lemlib::OdomSensors sensors(&vertical_odom, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            &horizontal_odom, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

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


// Create the Drivetrain with LemLib.
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // The track width (From the front of the bot, how far apart are the wheels?)
                              lemlib::Omniwheel::NEW_275, // what wheel?
                              450, // Drivetrain RPM
                              2 // horizontal drift
);

// DRIVING STUFF:

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// finally create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve, 
                        &steer_curve
);



void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); //initialize the screen
	chassis.calibrate(); //calibrate sensors yippee


	pros::lcd::register_btn1_cb(on_center_button);

	pros::Task spitInfo([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(20);
			//spits info onto the screen so you can perfect PID!
			//this utilizes an Lambda: you name, define, and use the task all in one
        }
    });
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


//makeshift intake code
 void intake() {
   if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    firstStage.move(100);
    secondStage.move(100);
  }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    firstStage.move(-100);
    secondStage.move(-100);
  }else {
    firstStage.move(0);
    secondStage.move(0);
  }
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
while (true) {
    intake();
	    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Curvature Drive
        chassis.curvature(leftY, rightX);

		// Arcade Drive (disable the curve exponent stuff, higher numbers = prioritize throttle)
		//chassis.arcade(leftY, leftX, false, 0.75);

        // delay to save resources
        pros::delay(25); 
}
}