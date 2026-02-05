#include "main.h"
#include "devices.h"
#include "intake.h"
#include "utils.h"
#include "math.h"

void Intake::initialize() {
    update_task = std::make_unique<pros::Task>([this](){
        while(true) {
            update();
            pros::delay(10);
        }
    });

}

void Intake::store(int power) {
    cooldown = 200;
    Intake::power = power;
    state = IntakeState::STORING;
    bottom_intake.move(Intake::power);
    top_intake.move(0);
}

void Intake::score(int power, bool middle) {
    if (middle) {
        middlescore_piston.set_value(true);
    } else {
        top_score.set_value(false);
    }
    cooldown = 200;
    Intake::power = power;
    state = IntakeState::SCORING;
    bottom_intake.move(Intake::power);
    top_intake.move(Intake::power);
}

void Intake::outtake(int power) {
    cooldown = 200;
    Intake::power = power;
    state = IntakeState::OUTTAKING;
    bottom_intake.move(-Intake::power);
    top_intake.move(-Intake::power);
}

void Intake::stop() {
    state = IntakeState::STOPPED;
    bottom_intake.move(0);
    top_intake.move(0);
    middlescore_piston.set_value(false);
    top_score.set_value(true);
}

void Intake::update() {
    if (cooldown != 0) {
        cooldown -= 10;
        return;
    }

    int reversedPower = 40; // Note current bug where the power gets 
    if (bottom_intake.get_current_draw() > 1200) {
        if (!reversed) {
            reversed = true;
            int tempPower = Intake::power;
            cooldown = 200;
            switch(state) {
                case STORING:
                    store(-reversedPower);
                    break;
                case SCORING:
                    score(-reversedPower);
                    break;
                case OUTTAKING:
                    outtake(-reversedPower);
                    break;
            }
            Intake::power = tempPower;
        }
    } else if (reversed) {
        reversed = false;
        switch(state) {
            case STORING:
                store(-power);
                break;
            case SCORING:
                score(-power);
                break;
            case OUTTAKING:
                outtake(-power);
                break;
        }
    }
}