#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include <thread>
#include "pros/rtos.hpp"

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

pros::adi::DigitalOut mogoclamp ('A');


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


bool color_sorting = false;
void colorSort() {
    // int colorvalue = 150;///(REJECTING BLUE)
    int colorvalue = 30;///(REJECTING RED))


    while (true) {
        // Read the hue value from the optical sensor
        double h = optical_sensor.get_hue();

        //pros::lcd::print(2, "Hue: %.2f", h);

        // Check the hue value and control the intake motor
        if (h > colorvalue) {    
            color_sorting = true;
            intake.move(0); // Stop the intake motor
            pros::delay(30);
            intake.move(127);
            pros::delay(200);
        } else {
            color_sorting=false;
            // pros::lcd::print(3, "Intake running!");
            // std::cout << "Intake running!" << std::endl;
        }
        // Small delay to avoid flooding the screen/terminal
        // pros::delay(20);
    }
}

enum LB_States {Default, Load, ReadyScore, Scoring, ManualMode};
LB_States currentState = Default;
const int positions[] = {0, 60, 150, 290};  // Added const for safety
const int numPositions = sizeof(positions)/sizeof(positions[0]);  // Calculate array size

void TickLB(int L1, int L2, int up, int down, int xbtn) {
    // State transition logic
    if (currentState == ManualMode) {
        if (xbtn) {
            currentState = Default;
        }
    } else {
        if (up || down) {
            currentState = ManualMode;
        } else if (L1 && L2) {
            currentState = Scoring;
        } else if (L1) {
            currentState = ReadyScore;
        } else if (L2) {
            currentState = Load;
        } else {
            currentState = Default;
        }
    }

    // State execution logic
    if (currentState != ManualMode) {
        // Ensure we don't access out of bounds array positions
        int targetPos = 0;
        if (currentState >= 0 && currentState < numPositions) {
            targetPos = positions[currentState];
        }
        
        LadyBrownL.move_absolute(targetPos, 80);
        LadyBrownR.move_absolute(targetPos, 80);
    } else {
        // Manual control
        if (up) {
            LadyBrownL.move(70);
            LadyBrownR.move(70);
        } else if (down) {
            LadyBrownL.move(-70);
            LadyBrownR.move(-70);
        } else {
            LadyBrownL.brake();
            LadyBrownR.brake();
        }
    }
}

void intakeRank(void* color_ptr) {
	// 	////BLUE = 1, RED = 0, INT COLOR IS WHAT YOU WANT TO SCORE
    int color = *(int*)color_ptr; // Cast the void pointer back to an int
    while (true) {
        double h = optical_sensor.get_hue();
        if ((color == 0 && h > 150) || (color == 1 && h < 30)) {
			intake.move(100);
            intake.move(0);
            pros::delay(200);
            intake.move(127);
            pros::delay(200);
            intake.move(-127);
            pros::delay(200);
            intake.move(0);
        } else if ((color == 0 && h < 30) || (color == 1 && h > 150)) {
            intake.move(-100);
            pros::delay(400);
            intake.move(0);
        }
        pros::delay(10);
    }
	delete (int*)color_ptr;
}


void startIntakeRankThread(int color) {
    // Dynamically allocate memory for the color parameter to pass to the thread
    int* color_ptr = new int(color);

    // Create a new thread to run intakeRank
    pros::Task intakeRankTask(intakeRank, color_ptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Rank Task");
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
	pros::lcd::register_btn1_cb(on_center_button);
  	pros::Optical optical_sensor({13});
	optical_sensor.set_integration_time(10);
	LadyBrownL.tare_position();
	LadyBrownR.tare_position();



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
int lastLimVal = 0; // Stores the previous state of the limit switch
int clampState = 0; // 0 = clamp open, 1 = clamp closed
bool lastL1State = 0; // Stores the previous state of the L1 button

inline void updateClamp() {
  // Read inputs
int limVal = lswitch.get_value(); // Current limit switch state
int L1Button = master.get_digital(DIGITAL_L1); // Current L1 button state

// If the limit switch is triggered (active)
if (limVal) {
    // Clamp should be disabled (set to 0)
    clamp.set_value(0);
    
    // If the L1 button is pressed, enable the clamp (set to 1)
    if (L1Button) {
        clamp.set_value(1);
    }
}
// If the limit switch is not triggered and L1 button is not pressed
else if (!limVal && !L1Button) {
    // Clamp should be enabled (set to 1)
    clamp.set_value(1);
}
// If the limit switch is not triggered but the L1 button is pressed
else {
    // Enable the clamp (set to 1)
    clamp.set_value(1);
}


    
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
// void intakeRank(int color) {
// 	////BLUE = 1, RED = 0, INT COLOR IS WHAT YOU WANT TO SCORE
// 	intake.move(-100);
// 	while(true) {
// 		double h = optical_sensor.get_hue();
// 		if((color == 0 && h > 150) || (color == 1 && h < 30)) {
// 			pros::delay(200);
// 			intake.move(0);
// 			pros::delay(200);
// 			intake.move(127);
// 			pros::delay(200);
// 			intake.move(-127);
// 			pros::delay(200);
// 			intake.move(0);
// 			break;
// 		}
// 		else if ((color == 0 && h <30) || (color == 1 && h > 150))
// 		{

// 			intake.move(-100);
// 			pros::delay(400);
// 			intake.move(0);

// 			break;
// 		}
		
// 		pros::delay(10);


// 	}
// }


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
    LadyBrownL.tare_position();
    LadyBrownR.tare_position();
	LadyBrownL.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	LadyBrownR.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	// LadyBrownL.tare_position();
	// LadyBrownR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// LadyBrownL.tare_position();
	// LadyBrownR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// pros::Task colorSortThread(colorSort);

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
   
    pros::adi::DigitalOut LBButton ('G');
    

	while (true) {
        LadyBrownL.tare_position();
        LadyBrownR.tare_position();

		updateDoinker();
		updateClamp();
        optical_sensor.set_led_pwm(100);

		int L = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int R = master.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick

		int Intakeup = master.get_digital(DIGITAL_R1);
		int Intakedown = master.get_digital(DIGITAL_R2);

		int YButtonJustPressed = master.get_digital(DIGITAL_Y);
		int XButtonJustPressed = master.get_digital_new_press(DIGITAL_X);
		int BButtonJustPressed = master.get_digital_new_press(DIGITAL_B);
		int AButtonJustPressed = master.get_digital_new_press(DIGITAL_A);
  

		// int aButtonJustPressed = master.get_digital_new_press(DIGITAL_B);

		int LBup = master.get_digital(DIGITAL_UP);
		int LBdown = master.get_digital(DIGITAL_DOWN);
		int LBright = master.get_digital(DIGITAL_RIGHT);
		int Left = master.get_digital(DIGITAL_LEFT);

		int L1Button = master.get_digital(DIGITAL_L1);
		int L2Button = master.get_digital(DIGITAL_L2);

		// TickLB(L1Button, L2Button);

		/////////MANUAL CLAMP CODE//////////////////////////////
			// int clampButton = master.get_digital(DIGITAL_L1);

			// if (clampButton) {
			// 	clamp.set_value(HIGH);
			// } else {
			// 	clamp.set_value(LOW);
			// }
		/////////MANUAL CLAMP CODE///////////////////////////

		
		if(Intakeup &&!color_sorting) {
			intake.move(110);
		} else if(Intakedown &&!color_sorting) {
			intake.move(-110);
		} else {
			intake.brake();
		}

		TickLB(LBright,YButtonJustPressed, LBup, LBdown,XButtonJustPressed);
        
        // MANUAL LB MOVEMENT
        // if (LBup) {
        //     LadyBrownL.move(50);
        //     LadyBrownR.move(50);
        // }
        // else if (LBdown) {
        //     LadyBrownL.move(-50);
        //     LadyBrownR.move(-50);
        // }
        // else {
        //     LadyBrownL.move(0);
        //     LadyBrownR.move(0);
        // }

        // if (Left) {
        //     LadyBrownL.move_absolute(100, 100);
        //     LadyBrownR.move_absolute(100, 100);
        // }


		// left_mg.move(L);                      // Sets left motor voltage
		// right_mg.move(R);                     // Sets right motor voltage

		chassis.tank(L,R);
        // int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // // move the robot
    //    chassis.arcade(leftY, rightX);

		
		pros::delay(10);    
                           // Run for 20 ms then update
	}
}