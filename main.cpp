#include "main.h"
#include "lemlib/api.hpp"

// left motor group
pros::MotorGroup left_motor_group({-4, -5, -6}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({1, 2, 3}, pros::MotorGears::blue);
pros::adi::DigitalOut piston('A');

// Intake motors
pros::Motor intake_lower(7);
pros::Motor intake_upper(8);
pros::Motor arm(9);


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(10);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
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

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
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
void autonomous() 
{
 
        chassis.setPose(0,0,0);   
        chassis.turnToHeading(45, 4000);
        chassis.moveToPoint(10,  10, 4000); 
        piston.set_value(true);

        
        
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	
    
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
       // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.75;
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.75;
		int lowerIntakePower = 127;
        int upperIntakePower = lowerIntakePower * 0.7 ;

        // move the robot
        chassis.arcade(leftY, rightX);



		//auton
		 //Pneumatics
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            piston.set_value(true); // Extend piston
        }

        // If Button B is pressed, retract the piston
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            piston.set_value(false); // Retract piston

        }

                // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_lower.move(lowerIntakePower);   // Move intake forward
            intake_upper.move(upperIntakePower); // Move intake forward
         }
         else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) 
        {
            intake_lower.move(-1*lowerIntakePower);   // Reverse intake
            intake_upper.move(-1*upperIntakePower); // Reverse intake
        }
        else
        {
            intake_lower.move(0);   // Stop intake
            intake_upper.move(0); // Stop intake
        }

        //Arm Control P.S: WORK ON ARM CODE 
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
        {
            arm.move(10);

        }
        // delay to save resources
        pros::delay(25);
	
	}
}
