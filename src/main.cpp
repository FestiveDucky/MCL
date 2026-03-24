#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/misc/lv_area.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "screen.h"
#include "autons.h"
#include "intake.h"
#include <array>
#include <atomic>
#include <cstdio>
#include <string>
#include <vector>

namespace {
struct AutonOption {
    const char* name;
    void (*run)();
};

// Change this table to change auton count and mapping for the potentiometer selector.
const std::array<AutonOption, 6> AUTONS = {{
    {"Left", left},
    {"Right", right},
    {"Skills", skills96},
    {"Five Inch", fiveInch},
	{"Test", test},
	{"Solo AWP", soloAWP}
}};

std::vector<std::string> autonNamesFromTable() {
    std::vector<std::string> names;
    names.reserve(AUTONS.size());
    for (const auto& auton : AUTONS) names.emplace_back(auton.name);
    return names;
}

std::atomic_bool mclPaused{false};

void setMCLPaused(bool pause) {
    if (pause == mclPaused.load()) return;
    lemlib::toggleMCL();
    mclPaused.store(pause);
}
} // namespace


int turn_divider = 1;
int right_voltage = 0;
int left_voltage = 0;
int bottom_intake_voltage = 0;
int top_intake_voltage = 0;
bool just_lowered_hood = false;

bool removerActivated = false;
bool wingActivated = false;
bool hoodActivated = false;
bool scraperActivated = false;

// flingBlue = false;
bool removerPressedLast = false;
bool hoodPressedLast = false;
bool scraperPressedLast = false;
bool wingPressedLast = false;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	vertical_rotation.reset();
	horizontal_rotation.reset();
	// pros::lcd::initialize();
    chassis.calibrate();
    // Keep MCL initialized/running, but paused until autonomous starts.
    setMCLPaused(true);
	controller.clear();
	// controller.set_text(0, 0, "Blub Blub");
	// sc.setAutonNames(autonNamesFromTable());
	sc.initialize();
	intake.initialize();

    static pros::Task screen_task([&]() {
        while (true) {
            if (mclPaused.load()) {
                sc.showInfoLabel("MCL paused (enabled in autonomous)");
                pros::delay(100);
                continue;
            }

			lemlib::Pose poseMCL = chassis.getPose();
			lemlib::Pose poseOLD = lemlib::getOldPose();
			std::uint32_t calculationTime = lemlib::getCalculationTime();
			std::int32_t confidenceFront = lemlib::getConfidence();
            char info[192];
            std::snprintf(
                info,
                sizeof(info),
                "MCL X: %.2f Y: %.2f\nTh: %.2f OldX: %.2f OldY: %.2f\nCalc Time: %u  FrontConf: %d",
                poseMCL.x,
                poseMCL.y,
                poseMCL.theta,
                poseOLD.x,
                poseOLD.y,
                static_cast<unsigned>(calculationTime),
                static_cast<int>(confidenceFront));
            sc.showInfoLabel(info);
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
    sc.state = RobotState::AUTONOMOUS;
		left_motor_group.set_brake_mode(MOTOR_BRAKE_HOLD);
		right_motor_group.set_brake_mode(MOTOR_BRAKE_HOLD);
    setMCLPaused(false);
	soloAWP();
    // int idx = sc.selectedAuton;
    // if (idx < 0 || idx >= static_cast<int>(AUTONS.size())) idx = 0;
    // if (AUTONS[idx].run != nullptr) AUTONS[idx].run();
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
	// sc.hideSelector();

	// right9ball();	
	// test();
	// skills();
	// chassis.setPose(-48, -48, 0);
	

	//right();
	wing.set_value(true);
	while (true) {

		// if (controller.get_digital_new_release(DIGITAL_R1) || controller.get_digital_new_release(DIGITAL_R2) || controller.get_digital_new_release(DIGITAL_L1) || controller.get_digital_new_release(DIGITAL_L2)) {
		if (controller.get_digital_new_release(DIGITAL_L1) || controller.get_digital_new_release(DIGITAL_L2) || controller.get_digital_new_release(DIGITAL_R1) || controller.get_digital_new_release(DIGITAL_R2)) {
			bottom_intake_voltage = 0;
			top_intake_voltage = 0;
			flappier.set_value(true);
			flappy.set_value(false);
		}

		// Intake (top)
		if (controller.get_digital_new_press(DIGITAL_L1)) {
			flappier.set_value(false);
			flappy.set_value(false);
			bottom_intake_voltage = 127;
			top_intake_voltage = 127;
		}

		if (controller.get_digital_new_press(DIGITAL_L1)) {
        flappy.set_value(true);
        flappier.set_value(true);
				bottom_intake_voltage = 127;
				top_intake_voltage = 127;
		}

		//storage in
		if (controller.get_digital_new_press(DIGITAL_R1)) {
			flappier.set_value(true);
			flappy.set_value(false);
			bottom_intake_voltage = 127;
			top_intake_voltage = 127;
		}

		// Outtake (Bottom)
		if (controller.get_digital_new_press(DIGITAL_R2)) {
			bottom_intake_voltage = -127;
			top_intake_voltage = -127;
			flappy.set_value(false);
    	flappier.set_value(true);
		}

		// Scraper (toggle)
		bool scraperPressedNow =
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);

		if (scraperPressedNow && !scraperPressedLast) {
		// Toggle scraper
		scraperActivated = !scraperActivated;
		scraper.set_value(scraperActivated);
		}

		scraperPressedLast = scraperPressedNow;

		//descore
		bool removerPressedNow =
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

		if (removerPressedNow && !removerPressedLast) {
		// Toggle remover
		removerActivated = !removerActivated;
		wing.set_value(removerActivated);
		}

		//middle goal
		if (controller.get_digital_new_press(DIGITAL_L2)) {
			intake.score(127, true);
			// flappy.set_value(true);
			// flappier.set_value(true);
			// bottom_intake_voltage = 127;
			// top_intake_voltage = 127;
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
