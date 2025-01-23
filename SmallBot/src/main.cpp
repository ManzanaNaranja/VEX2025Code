#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


// pros::Controller master(pros::E_CONTROLLER_MASTER);

// void a() {

// 	master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
// }

int SM1_Positions[3] = {0,30,180};
int lbindex = 0;
bool mogoClampVal = true;

pros::Imu imu(5);
// drivetrain, chassis and PID controllers definitions===================================
lemlib::ControllerSettings lateralPIDController(7, // proportional gain (kP)
                                                .1, // integral gain (kI)
                                                40, // derivative gain (kD)
                                                3, // anti windup
                                                0.25, // small error range, in inches
                                                100, // small error range timeout, in milliseconds
                                                1, // large error range, in inches
                                                750, // large error range timeout, in milliseconds
                                                16 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularPIDController(2, // proportional gain (kP)
                                                .1, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in inches
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in inches
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
);
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.5, lemlib::Omniwheel::NEW_275, 600, 2);
lemlib::Chassis chassis(drivetrain, lateralPIDController, angularPIDController,sensors);

pros::ADIDigitalOut mogoclamp ('A');

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// void nextLB() {
// 	lbindex++;
// 	if(lbindex > 2) lbindex = 0;
// }

enum LB_States {Default, Load, ReadyScore, Scoring};
LB_States currentState = Default;
int positions[] = {0,50,150,250};


void TickLB(int L1, int L2) {

	if(L1 && L2) {
		currentState = Scoring;
	} else if(L1 && !L2) {
		currentState = ReadyScore;
	} else if(L2 && !L1) {
		currentState = Load;
	} else {
		currentState = Default;
	}

	LadyBrown.move_absolute(positions[currentState],80);
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 24, 4000);
	// chassis.turnToHeading(90, 2500,{.maxSpeed = 100});



    //  chassis.moveToPose(20,20, 0, 3000, {.lead = 0.3});

	// auto heading = std::to_string(imu.get_heading());
	// pros::lcd::set_text(1,heading);
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
	LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	LadyBrown.tare_position();
	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	// while(true) {
	// 	if(master.get_digital_new_press(DIGITAL_A)) {
	// 		autonomous();
	// 	}
	// 	auto heading = std::to_string(imu.get_heading());
	// 	pros::lcd::set_text(1,heading);
	// 	pros::delay(20);
	// }
	// return;


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

	
		int L = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int R = master.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick

		int Intakeup = master.get_digital(DIGITAL_R1);
		int Intakedown = master.get_digital(DIGITAL_R2);

		int YButtonJustPressed = master.get_digital_new_press(DIGITAL_Y);

		int aButtonJustPressed = master.get_digital_new_press(DIGITAL_A);

		int LBup = master.get_digital(DIGITAL_UP);
		int LBdown = master.get_digital(DIGITAL_DOWN);
		int LBright = master.get_digital(DIGITAL_RIGHT);

		int L1Button = master.get_digital(DIGITAL_L1);
		int L2Button = master.get_digital(DIGITAL_L2);

		TickLB(L1Button, L2Button);

		if(YButtonJustPressed) {
			mogoClampVal = !mogoClampVal;
			mogoclamp.set_value(mogoClampVal);
		}

		if(Intakeup) {
			intake.move(127);
		} else if(Intakedown) {
			intake.move(-127);
		} else {
			intake.brake();
		}

		// if(LBup) {
		// 	// LadyBrown.move_absolute(0,100);
		// 	LadyBrown.move(70);
		// } else if(LBdown) {

		// 	LadyBrown.move(-70);
		// 	// LadyBrown.move(-80);
		// } else {
		// 	LadyBrown.brake();
		// 	// LadyBrown.move(0);
		// 	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		// 	// LadyBrown.move(0);
		// }

		// if(aButtonJustPressed) {
		// 	nextLB();
		// 	LadyBrown.move_absolute(SM1_Positions[lbindex], 100);
		// } else {
		// 	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		// }


		// left_mg.move(L);                      // Sets left motor voltage
		// right_mg.move(R);                     // Sets right motor voltage

		chassis.tank(L,R);
        // int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // // move the robot
        // chassis.arcade(leftY, rightX);

   ;
		pros::delay(20);                               // Run for 20 ms then update
	}
}