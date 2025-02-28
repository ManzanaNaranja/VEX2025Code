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



// void TickLB(int xbtn, int up, int down) {

// 	if(up || down) {
// 		currentState = ManualMode;
// 	}
// 	else if(xbtn && currentState == Default) {
// 		currentState = Load;
// 		pros::lcd::set_text(1, "Load");
// 	} else if(xbtn && currentState == Load) {
// 		currentState = Scoring;
// 		pros::lcd::set_text(1, "scoring");
// 	} else if(xbtn && currentState == Scoring) {
// 		currentState = Default;
// 		pros::lcd::set_text(1, "default");
// 	} else if(xbtn) {
// 		currentState = Default;
//       pros::lcd::set_text(1, "default");
// 	} 
	
// 	if(currentState != ManualMode) {
// 		LadyBrown.move_absolute(positions[currentState],100);
// 	} else {
// 		if(up) {
// 			// LadyBrown.move_absolute(0,100);
// 			LadyBrown.move(70);
// 		} else if(down) {

// 			LadyBrown.move(-70);
// 			// LadyBrown.move(-80);
// 		} else {
// 			LadyBrown.brake();
// 			// LadyBrown.move(0);
// 			LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
// 			// LadyBrown.move(0);
// 		}
// 	}
	
// }


enum LB_States {Default, Load, ReadyScore, Scoring, ManualMode};
LB_States currentState = Default;
int positions[] = {0,55,150,250};


void TickLB(int L1, int L2, int up, int down, int xbtn) {

	if(up || down) {
		currentState = ManualMode;
	}
	else if(L1 && L2) {
		currentState = Scoring;
	} else if(L1 && !L2) {
		currentState = ReadyScore;
	} else if(L2 && !L1) {
		currentState = Load;
	} else if(currentState != ManualMode) {
		currentState = Default;
	} else if(xbtn && currentState == ManualMode) {
		currentState = Default;
	}
	
	if(currentState != ManualMode) {
		LadyBrownL.move_absolute(positions[currentState],80);
		LadyBrownR.move_absolute(positions[currentState],80);

	} else {
		if(up) {
			// LadyBrown.move_absolute(0,100);
			LadyBrownR.move(70);
			LadyBrownL.move(70);

		} else if(down) {

			LadyBrownR.move(-70);
			LadyBrownL.move(-70);

			// LadyBrown.move(-80);
		} else {
			LadyBrownL.brake();
			LadyBrownR.brake();

			// LadyBrown.move(0);
			LadyBrownR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			LadyBrownL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

			// LadyBrown.move(0);
		}
	}
	
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
	LadyBrownL.tare_position();
	LadyBrownR.tare_position();



	//////////////////////////////////COLOR SENSOR

            // double hue = optical_sensor.get_hue();

            // // Modify the conditions based on color needs
            // if (hue > 150) {  // Example: Detects blue
            //     intake.move(-127);
            //     pros::delay(1000);  // Adjust time as needed
            //     intake.move(127);
            // } 
            // else if (hue < 30) {  // Example: Detects red
            //     intake.move(127);
            //     pros::delay(500);
            //     intake.move(0);
            // }

            // pros::delay(100);  // Prevent CPU overload
	


	//////////////////////////////////////
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
bool activated = true;
int clamptimer1 = 0;
int clamptimer2 = 1;
bool open =true;
int lastLimVal = 0;
int ccc = 0;



inline void updateClamp()
{	
	int limVal = lswitch.get_value();
	int L1Button = master.get_digital(DIGITAL_L2);
	int yyy = master.get_digital(DIGITAL_B);
	if(lastLimVal == 0 && limVal == 1 || yyy == 1) {
		ccc = 0;
	} else if (L1Button) {
		ccc = 1;
	} 
	clamp.set_value(ccc);
	lastLimVal = limVal;
}


bool l2 = false;
bool activatedDoinker = false;
inline void updateDoinker()
{
	
int L2Button = master.get_digital(DIGITAL_L1);
	if (L2Button) {
		doinker.set_value(1);
	}
	else {
		doinker.set_value(0);

	}


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
//////////////////////////AUTON FUNCTIONS///////////////////////////
void intakeRank2(int color) {
	////BLUE = 1, RED = 0, INT COLOR IS WHAT YOU WANT TO SCORE
	intake.move(-100);
	int timeout = pros::millis() + 2000;
	while(pros::millis() < timeout) {
		double h = optical_sensor.get_hue();
		if((color == 0 && h > 150) || (color == 1 && h < 30)) {
			pros::delay(200);
			intake.move(0);
			pros::delay(200);
			intake.move(127);
			pros::delay(200);
			intake.move(-127);
			pros::delay(200);
			intake.move(0);
			break;
		}
		else if ((color == 0 && h <30) || (color == 1 && h > 150))
		{

			intake.move(-100);
			pros::delay(400);
			intake.move(0);

			break;
		}
		
		pros::delay(10);


	}
}

void halfintake(int color) {
	////BLUE = 1, RED = 0
	///COLOR 	IS WHAT YOU WANT
    intake.move(-100);
    int timeout = pros::millis() + 2000; // 4-second timeout

    while (pros::millis() < timeout) {  // Ensures loop exits after timeout
        double h = optical_sensor.get_hue();

        if ((color == 1 && h > 150) || (color == 0 && h < 30)) { 
            break; // Stop if the hue condition is met
        }

        pros::delay(10); // Small delay to prevent CPU overuse
    }

    intake.move(0); // Stop the intake
}


//////////////////////////AUTON FUNCTIONS////////////////////////////

void autonomous() {
	
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

int clampButtonPrev = 0; // Stores previous button state

bool autonActivated=false;
void opcontrol() {
	LadyBrownL.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	LadyBrownR.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	LadyBrownL.tare_position();
	LadyBrownR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	LadyBrownL.tare_position();
	LadyBrownR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	//PRACTICE//AUTON/////////////////////////////////////////////////
// 	while (!autonActivated) {
//     if (master.get_digital_new_press(DIGITAL_A)) {
// 		autonActivated=true;
//         autonomous();  // Run autonomous function
//         break;  // Exit the loop after autonomous() finishes
//     }
//     auto heading = std::to_string(imu.get_heading());
//     pros::lcd::set_text(1, heading);
//     pros::delay(20);
// }
	//PRACTICE//AUTON/////////////////////////////////////////////////

	while (true) {

		updateDoinker();
		updateClamp();

		int L = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int R = master.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick

		int Intakeup = master.get_digital(DIGITAL_R2);
		int Intakedown = master.get_digital(DIGITAL_R1);

		int YButtonJustPressed = master.get_digital_new_press(DIGITAL_Y);
		int XButtonJustPressed = master.get_digital_new_press(DIGITAL_X);
		int BButtonJustPressed = master.get_digital_new_press(DIGITAL_B);
		int AButtonJustPressed = master.get_digital_new_press(DIGITAL_A);

		// int aButtonJustPressed = master.get_digital_new_press(DIGITAL_B);

		int LBup = master.get_digital(DIGITAL_B);
		int LBdown = master.get_digital(DIGITAL_DOWN);
		int LBright = master.get_digital(DIGITAL_RIGHT);

		int L1Button = master.get_digital(DIGITAL_L1);
		int L2Button = master.get_digital(DIGITAL_L2);

		// TickLB(L1Button, L2Button);

		/////////MANUAL CLAMP CODE//////////////////////////////
			// int clampButton = master.get_digital(DIGITAL_L2);

			// if (clampButton) {
			// 	clamp.set_value(HIGH);
			// } else {
			// 	clamp.set_value(LOW);
			// }
		/////////MANUAL CLAMP CODE///////////////////////////



		
		if(Intakeup) {
			intake.move(120);
		} else if(Intakedown) {
			intake.move(-120);
		} else {
			intake.brake();
		}

		TickLB(L1Button,L2Button, LBup, LBdown,XButtonJustPressed);

		// double h = optical_sensor.get_hue();
		// if(h > 150) {
		// 	pros::delay(200);
		// 	intake.move(0);
		// 	pros::delay(200);
		// 	intake.move(127);
		// 	pros::delay(200);
		// 	intake.move(-127);
		// 	pros::delay(200);
		// 	intake.move(0);
		// 	break;
		// }



		// left_mg.move(L);                      // Sets left motor voltage
		// right_mg.move(R);                     // Sets right motor voltage

		chassis.tank(L,R);
        // int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // // move the robot
    //    chassis.arcade(leftY, rightX);

		if (GameTimer = 10000) {
		GameTimer = 0;
		}

		GameTimer += 20;   
		pros::delay(20);
                           // Run for 20 ms then update
	}
}