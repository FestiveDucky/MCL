#include "main.h"
#include "devices.h"
#include "intake.h"
#include <cmath>

namespace {
constexpr int kUpdatePeriodMs = 10;
constexpr int kDirectionSwapCooldownMs = 120;
constexpr int kJamCurrentThresholdMa = 1200;
constexpr double kJamVelocityThresholdRpm = 18.0;
constexpr int kJamDetectTimeMs = 120;
constexpr int kReverseDurationMs = 180;
constexpr int kReversePower = 40;

int reverseForPower(int commandedPower, int reversePower) {
    if (commandedPower > 0) return -reversePower;
    if (commandedPower < 0) return reversePower;
    return 0;
}
}

void Intake::initialize() {
    update_task = std::make_unique<pros::Task>([this](){
        while(true) {
            update();
            pros::delay(kUpdatePeriodMs);
        }
    });

}

void Intake::store(int power) {
    flappy.set_value(false);
    flappier.set_value(true);
    cooldown = 200;
    jam_timer_ms = 0;
    reverse_timer_ms = 0;
    reversed = false;
    Intake::power = power;
    state = IntakeState::STORING;
    bottom_intake.move(Intake::power);
    top_intake.move(Intake::power);
}

void Intake::score(int power, bool middle) {
    if (middle) {
        flappy.set_value(true);
        flappier.set_value(true);
    } else {
        flappier.set_value(false);
        flappy.set_value(false);
    }
    cooldown = 200;
    jam_timer_ms = 0;
    reverse_timer_ms = 0;
    reversed = false;
    Intake::power = power;
    state = IntakeState::SCORING;
    bottom_intake.move(Intake::power);
    top_intake.move(Intake::power);
}

void Intake::outtake(int power) {
    cooldown = 200;
    jam_timer_ms = 0;
    reverse_timer_ms = 0;
    reversed = false;
    Intake::power = power;
    state = IntakeState::OUTTAKING;
    bottom_intake.move(-Intake::power);
    top_intake.move(-Intake::power);
}

void Intake::stop() {
    state = IntakeState::STOPPED;
    jam_timer_ms = 0;
    reverse_timer_ms = 0;
    reversed = false;
    bottom_intake.move(0);
    top_intake.move(0);
    // flappy.set_value(false);
    // flappier.set_value(true);
}

void Intake::update() {
    int bottom_command = 0;
    int top_command = 0;
    switch(state) {
        case STORING:
            bottom_command = power;
            top_command = 0;
            break;
        case SCORING:
            bottom_command = power;
            top_command = power;
            break;
        case OUTTAKING:
            bottom_command = -power;
            top_command = -power;
            break;
        case STOPPED:
            jam_timer_ms = 0;
            reverse_timer_ms = 0;
            return;
    }

    if (cooldown != 0) {
        cooldown -= kUpdatePeriodMs;
        if (cooldown < 0) cooldown = 0;
        return;
    }

    if (reversed) {
        reverse_timer_ms -= kUpdatePeriodMs;
        if (reverse_timer_ms > 0) {
            return;
        }

        reversed = false;
        cooldown = kDirectionSwapCooldownMs;
        bottom_intake.move(bottom_command);
        top_intake.move(top_command);
        return;
    }

    // Anti-jam disabled
    (void)bottom_command;
    (void)top_command;
}

// void stopIntake() {
//     top_intake.brake();
//     bottom_intake.brake();
// }

// void scoreTop() {
//     flappier.set_value(false);
//     flappy.set_value(false);
//     top_intake.move(127);
//     bottom_intake.move(127);
// }

// void scoreMiddle() {
//     //intaking = true;
//     flappy.set_value(true);
//     flappier.set_value(true);
//     top_intake.move(127); 
//     bottom_intake.move(127);
// }

// void scoreBottom()
// {   
//     flappy.set_value(false);
//     flappier.set_value(true);
//     top_intake.move(-127);
//     bottom_intake.move(-127);
// }