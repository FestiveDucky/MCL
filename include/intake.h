#pragma once

#include "main.h"

#define intake Intake::getInstance()


enum IntakeState {
    STORING,
    SCORING,
    OUTTAKING,
    STOPPED
};

class Intake {
    private:
        std::unique_ptr<pros::Task> update_task;
        int jam_timer_ms = 0;
        int reverse_timer_ms = 0;
        Intake() = default;
        Intake(const Intake&) = delete;
        Intake& operator=(const Intake&) = delete;
    public:
        inline static Intake& getInstance() {
            static Intake INSTANCE;
            return INSTANCE;
        }

        int power = 0;
        int cooldown = 0;
        bool reversed = false;
        IntakeState state = STOPPED;
        void initialize();

        void store(int power);
        void score(int power, bool middle=false);
        void outtake(int power);
        void stop();

        void update();

};
