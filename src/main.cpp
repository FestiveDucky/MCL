#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/misc/lv_area.h"
#include "screen.h"
#include "autons.h"
#include "intake.h"


int turn_divider = 1;
int right_voltage = 0;
int left_voltage = 0;
int bottom_intake_voltage = 0;
int top_intake_voltage = 0;
bool just_lowered_hood = false;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	vertical_rotation.reset();
	pros::lcd::initialize();
    chassis.calibrate();
	controller.clear();
	controller.set_text(0, 0, "Blub Blub");
	// sc.initialize();
	intake.initialize();

    static pros::Task screen_task([&]() {
        while (true) {
			lemlib::Pose poseMCL = chassis.getPose();
			lemlib::Pose poseOLD = lemlib::getOldPose();
			std::uint32_t calculationTime = lemlib::getCalculationTime();
			std::int32_t confidenceFront = lemlib::getConfidence();
			pros::lcd::print(0, "MCL X: %f Y: %f", poseMCL.x, poseMCL.y);
			pros::lcd::print(1, "Theta: %f", poseMCL.theta);
			pros::lcd::print(2, "OLD X: %f Y: %f", poseOLD.x, poseOLD.y);
			pros::lcd::print(3, "Calculation Time: %d", calculationTime);
			pros::lcd::print(4, "Front Sensor Confidence: %d", confidenceFront);
            // sc.showInfoLabel(position.c_str());
            pros::delay(100); 
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
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
void autonomous() {
    printf("starti9gn auto\n");
	// sc.state = RobotState::AUTONOMOUS;
	// switch(sc.selectedAuton) {
    //     case 0: left(); break;
    //     case 1: right(); break;
    //     case 2: skills(); break;
    //     case 3: /* do nothing */ break;
    //     default: break;
    // }
	// left();

	skills();
	// test();
	// right9ball();
	// fiveInch();
	// left();
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
	// sc.state = RobotState::DRIVER;
	//sc.hideSelector();

	// right9ball();	
	// test();
	skills();
	// chassis.setPose(-48, -48, 0);
	

	//right();
	descore.set_value(true);
	
	
	while (true) {
		// Potentiometer test 
		// int at = potentiometer.get_value();
		// pros::lcd::print(3, "Potentiometer Value: %d", at);


		// if (controller.get_digital_new_release(DIGITAL_R1) || controller.get_digital_new_release(DIGITAL_R2) || controller.get_digital_new_release(DIGITAL_L1) || controller.get_digital_new_release(DIGITAL_L2)) {
		if (controller.get_digital_new_release(DIGITAL_L1) || controller.get_digital_new_release(DIGITAL_L2) || controller.get_digital_new_release(DIGITAL_Y) || controller.get_digital_new_release(DIGITAL_R2)) {
			bottom_intake_voltage = 0;
			top_intake_voltage = 0;
			top_score.set_value(true);
			middlescore_piston.set_value(false);
		}
		
		// release descore
		if (controller.get_digital_new_release(DIGITAL_R1)) {
			descore.set_value(true);
		}

		// Intake (Both)
		if (controller.get_digital_new_press(DIGITAL_L2)) {
			top_score.set_value(false);
			bottom_intake_voltage = 127;
			top_intake_voltage = 127;
			printf("running intake\n");
		}

		// Intake (Bottom)
		if (controller.get_digital_new_press(DIGITAL_L1)) {
			bottom_intake_voltage = 127;
			printf("storing\n");
		}

		// Outtake
		if (controller.get_digital_new_press(DIGITAL_Y)) {
			bottom_intake_voltage = -127;
			top_intake_voltage = -127;
			printf("outtaking\n");
		}

		// Scraper (toggle)
		if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
			scraper_piston.toggle();
			printf("scraper\n");
		}

		// Descore (toggle)
		if (controller.get_digital_new_press(DIGITAL_R1)) {
			descore.set_value(false);
		}

		if (controller.get_digital_new_press(DIGITAL_R2)) {
			middlescore_piston.set_value(true);
			bottom_intake_voltage = 127;
			top_intake_voltage = 127;
		}


        std::string currents = "Voltage Left: " + std::to_string(left_voltage) + "\n";
		currents += "Voltage Right: " + std::to_string(right_voltage) + "\n";

		

		// int dir = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X) * 0.90;
		int volt = controller.get_analog(ANALOG_LEFT_Y);
		// turn = turn < 15 ? 0 : turn; // Deadzone for turning

		currents += "Turn: " + std::to_string(turn) + "\n";
		currents += "Volt: " + std::to_string(volt) + "\n";
		// sc.showInfoLabel(currents.c_str());

		// chassis.arcade(volt, turn, false, 0.75);
		chassis.arcade(volt, turn, false, 0.75);
		// left_motor_group.move(volt + turn);
		// right_motor_group.move(volt - turn);

		
		bottom_intake.move(bottom_intake_voltage);
		top_intake.move(top_intake_voltage);

		pros::delay(20);
	}
}