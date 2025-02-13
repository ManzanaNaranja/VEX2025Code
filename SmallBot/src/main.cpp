#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCDpr text between
 * "I was pressed!" and nothing.
 */


// pros::Controller master(pros::E_CONTROLLER_MASTER);

// void a() {

// 	master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
// }
int GameTimer = 0;
bool l1;
int SM1_Positions[3] = {0,30,180};
int lbindex = 0;
bool mogoClampVal = true;
pros::Task* color_sort = nullptr;
pros::Imu imu(5);
// drivetrain, chassis and PID controllers def	ions===================================
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
  	pros::Optical optical_sensor({13});
	optical_sensor.set_integration_time(10);
	optical_sensor.set_led_pwm(50);

	
}
// 	pros::Task task{[=] {
// 		while (true) {

// 			pros::delay(1000)

// 		};
// 	}
// 	}
// }

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
bool activated = true;
int clamptimer1 = 0;
int clamptimer2 = 1;
bool open =true;


inline void updateClamp()
{	
	int L1Button = master.get_digital(DIGITAL_L1);
	if (L1Button) {
		clamp.set_value(1);
	}
	else {
		clamp.set_value(0);

	}


	// bool newL = false;  // Declare newY outside the if block
	// clamptimer2 = GameTimer;


	// if(clamp.get_value() && lswitch.get_value() == 1 && clamptimer2-clamptimer1>=1000)
	// {clamp.set_value(true);
	// 	open = !open;
	// }
	// else if(clamp.get_value() == false && R>95)
	// {clamp.set_value(false);
	// 	clamptimer1=GameTimer;
	// 	open=!open;
	// }

	// if (activated == false && lswitch.get_value() == 1) {
	// l1 = !l1;
	// clamp.set_value(l1);
    // activated = true;  // Only deactivate if R <= 90 and loop exits naturally
	// }



// 	while (activated == true && lswitch.get_value() == 1) {
//     if (R > 90) {   // Check first before setting activated to false
//         newL = true;
//         break;
//     }
//     activated = false;  // Only deactivate if R <= 90 and loop exits naturally
// }

//MANUAL CODE FOR CLAMP
 // Gets amount forward/backward from left joystick
	
/////////////////////////////////////////////////////////
// 	bool newL = false;  // Declare newY outside the if block
// 	int R = master.get_analog(ANALOG_RIGHT_X);  // Gets amount forward/backward from left joystick

// 	if (R > 90) {
// 		newL = true;  // Update its value if condition is met
// 	}

// 	if (newL == true)
// 	{
// 		if (activated == false)
// 		{
// 			l1 = !l1;
// 			clamp.set_value(l1);
// 			activated = true;
// 		}
// 	}
// 	else
// 	{
// 		activated = false;
// 	}
}







bool l2 = false;
bool activatedDoinker = false;
inline void updateDoinker()
{
	
int L2Button = master.get_digital(DIGITAL_L2);
	if (L2Button) {
		doinker.set_value(1);
	}
	else {
		doinker.set_value(0);

	}


	// //OLD CODE/////////////////////////////////
	// int L = master.get_analog(ANALOG_LEFT_X);  // Gets amount forward/backward from left joystick
	// bool newY = true;  // Declare newY outside the if block

	// if (L > -90) {
	// 	newY = true;  // Update its value if condition is met
	// }

	// if (newY == true)
	// {
	// 	if (activatedDoinker == false)
	// 	{
	// 		l2 = !l2;
	// 		doinker.set_value(l2);
	// 		activatedDoinker = true;
	// 	}
	// }
	// else
	// {
	// 	activatedDoinker = false;
	// }
	// //OLD CODE/////////////////////////////////

}

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
//////////////////////////AUTON FUNCTIONS////////////////////////////
void inTakeRing(double color) {
	int counter = 0;
	while (counter < 100) {
		if (color > 130) {
			pros::delay(100);
			intake.move(-127);
			pros::delay(1000);			
			intake.move(127);
		}
		pros::delay(100);
		counter +=1;
	} 

}

void moveTilesFast(double tiles) {
	right_mg.move_velocity(127);
  	left_mg.move_velocity(127);
	pros::delay(280 * tiles);
	right_mg.move_velocity(0);
  	left_mg.move_velocity(0);

}

void moveTiles(double tiles) {
	right_mg.move_velocity(70);
  	left_mg.move_velocity(70);
	pros::delay(600 * tiles);
	right_mg.move_velocity(0);
  	left_mg.move_velocity(0);

}

void moveBackTiles(double tiles) {
	right_mg.move_velocity(-70);
  	left_mg.move_velocity(-70);
	pros::delay(600 * tiles);
	right_mg.move_velocity(0);
  	left_mg.move_velocity(0);
}

void doink(int seconds) {
    doinker.set_value(1);  // Activate doinker
    pros::delay(1000*seconds);       // Keep it active for 0.5 seconds
    doinker.set_value(0);  // Reset doinker
}

void rush() {
	doinker.set_value(1);
	moveTilesFast(1.4);
	pros::delay(800);
	moveBackTiles(1.3);
	pros::delay(500);
	doinker.set_value(0);
}

void turnleft90(double degrees) {
	right_mg.move_velocity(50);  // Move right motor forward
	left_mg.move_velocity(-50);  // Move left motor backward
	pros::delay(450*degrees);            // Adjust this time for a 45-degree turn
	right_mg.move_velocity(0);
	left_mg.move_velocity(0);

}

void turnright90(double degrees) {
	right_mg.move_velocity(-50);  // Move right motor forward
	left_mg.move_velocity(50);  // Move left motor backward
	pros::delay(450*degrees);            // Adjust this time for a 45-degree turn
	right_mg.move_velocity(0);
	left_mg.move_velocity(0);

}

void intakeRank(int color) {

	intake.move(-100);
	bool blueDetected = false;
	while(!blueDetected) {
		double h = optical_sensor.get_hue();
		if(h > 150) {
			blueDetected = true;
		}
		pros::delay(10);


	}

	pros::delay(200);
	intake.move(-127);
	pros::delay(200);
	intake.move(0);
}

void intakeRank2() {
	intake.move(-100);
	while(true) {
		double h = optical_sensor.get_hue();
		if(h > 150) {
			intake.move(0);
			pros::delay(300);
			intake.move(-127);
			pros::delay(200);
			intake.move(0);
			break;
		}
		else if (h < 30)
		{

			pros::delay(400);
			intake.move(0);
			break;
		}
		
		pros::delay(10);


	}
}

void halfintake(int color) {
	//BLUE ONLY
	intake.move(-100);
	bool blueDetected = false;
	while(!blueDetected) {
		double h = optical_sensor.get_hue();
		if(h > 150) {
			blueDetected = true;
		}
		pros::delay(10);
		// if(blueDetected) {
		// 	pros::lcd::set_text(3, "ring detected");
		// } else {
		// 	pros::lcd::set_text(3, "not detect");
		// }

	}
	intake.move(0);
}

//////////////////////////AUTON FUNCTIONS////////////////////////////

void autonomous() {
	//AUTON////////////////////////////////////////////////
	intakeRank2();

	// rush();
	// pros::delay(400);
	// turnleft90(1.3);
	// pros::delay(300);
	// moveTiles(0.4);
	// // //NEED TO CHANGE TO RED SIDE
	// halfintake(1);
	// turnright90(0.37);
	// clamp.set_value(HIGH);
	// moveBackTiles(2.8);
	// clamp.set_value(LOW);
	// pros::delay(300);
	// intake.move(-100);
	// pros::delay(1000);
	// intake.move(0);
	// turnright90(0.25);
	// moveTiles(3);
	// intakeRank(1);
	// pros::delay(200);
	// moveTiles(0.1);
	// pros::delay(200);
	// intakeRank(1);
	// turnright90(0.5);
	// moveTiles(0.5);
	// intakeRank(1);
	// intakeRank(1);
	// moveBackTiles(2);
	// turnleft90(1.3);
	// moveTiles(1.5);
	// intakeRank(1);
	// intakeRank(1);






	 //AUTON////////////////////////////////////////////////



// ////MANUAL TURN CODE/////
// // right_mg.move_velocity(50);  // Move right motor forward
// // left_mg.move_velocity(-50);  // Move left motor backward
// // pros::delay(450);            // Adjust this time for a 45-degree turn
// // right_mg.move_velocity(0);
// // left_mg.move_velocity(0);

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

bool autonActivated=false;
void opcontrol() {
	LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	LadyBrown.tare_position();
	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	//AUTON/////////////////////////////////////////////////
	while (!autonActivated) {
    if (master.get_digital_new_press(DIGITAL_A)) {
		autonActivated=true;
        autonomous();  // Run autonomous function
        break;  // Exit the loop after autonomous() finishes
    }
    auto heading = std::to_string(imu.get_heading());
    pros::lcd::set_text(1, heading);
    pros::delay(20);
}
	//AUTON/////////////////////////////////////////////////

	while (true) {
		// double h = optical_sensor.get_hue();	
		// double s = optical_sensor.get_saturation();
		// double v = optical_sensor.get_brightness();
		// std::int32_t set_led_pwm(50);
		// pros::lcd::set_text(2,"Pose: " + std::to_string(h) + " "+ std::to_string(s) + " " + std::to_string(v));

		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs


		updateDoinker();
		updateClamp();

		int L = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int R = master.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick

		int Intakeup = master.get_digital(DIGITAL_R2);
		int Intakedown = master.get_digital(DIGITAL_R1);

		int YButtonJustPressed = master.get_digital_new_press(DIGITAL_Y);

		int aButtonJustPressed = master.get_digital_new_press(DIGITAL_A);

		int LBup = master.get_digital(DIGITAL_UP);
		int LBdown = master.get_digital(DIGITAL_DOWN);
		int LBright = master.get_digital(DIGITAL_RIGHT);

		// int L1Button = master.get_digital(DIGITAL_L1);
		int L2Button = master.get_digital(DIGITAL_L2);

		// TickLB(L1Button, L2Button);




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

		if(LBup) {
			LadyBrown.move(70);
		} else if(LBdown) {
			LadyBrown.move(-70);
		} else {
			LadyBrown.brake();
			LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

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

		if (GameTimer = 10000) {
		GameTimer = 0;
		}

		GameTimer += 20;   
		pros::delay(20);
                           // Run for 20 ms then update
	}
}
