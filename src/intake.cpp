#include "main.h"
#include "devices.h"
#include "intake.h"
#include <cmath>

namespace {
constexpr bool kAntiJamEnabled = false;
constexpr int kUpdatePeriodMs = 10;
constexpr int kDirectionSwapCooldownMs = 120;
constexpr int kJamCurrentThresholdMa = 1200;
constexpr double kJamVelocityThresholdRpm = 18.0;
constexpr int kJamDetectTimeMs = 120;
constexpr int kReverseDurationMs = 150;
constexpr int kReversePower = 65;

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

void Intake::store(int power, bool includeTop) {
    cooldown = 200;
    jam_timer_ms = 0;
    reverse_timer_ms = 0;
    reversed = false;
    Intake::power = power;
    state = IntakeState::STORING;
    bottom_intake.move(Intake::power);
    if (includeTop) {
        top_intake.move(30);
    } else {
        top_intake.move(0);
    }
    
}

void Intake::score(int power, bool middle) {
    if (middle) {
        middlescore_piston.set_value(false);
    } else {
        top_score.set_value(true);
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
    middlescore_piston.set_value(false);
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
    middlescore_piston.set_value(true);
    top_score.set_value(false);
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

    if (!kAntiJamEnabled) {
        jam_timer_ms = 0;
        reverse_timer_ms = 0;
        reversed = false;
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

    const bool command_active = std::abs(bottom_command) > 0;
    const int current_ma = std::abs(bottom_intake.get_current_draw());
    const double velocity_rpm = std::abs(bottom_intake.get_actual_velocity());
    const bool jam_candidate =
        command_active &&
        current_ma > kJamCurrentThresholdMa &&
        velocity_rpm < kJamVelocityThresholdRpm;

    if (!jam_candidate) {
        jam_timer_ms = 0;
        return;
    }

    jam_timer_ms += kUpdatePeriodMs;
    if (jam_timer_ms < kJamDetectTimeMs) {
        return;
    }

    jam_timer_ms = 0;
    reversed = true;
    reverse_timer_ms = kReverseDurationMs;
    bottom_intake.move(reverseForPower(bottom_command, kReversePower));
    top_intake.move(reverseForPower(top_command, kReversePower));
}
