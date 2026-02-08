#pragma once

#include "main.h"
#include "pros/apix.h"
#include <memory>
#include <string>
#include <vector>

#define sc Screen::getInstance()

enum RobotState {
    PRE_MATCH,
    AUTONOMOUS,
    DRIVER
};

class Screen {
    private:
        
        std::unique_ptr<pros::Task> update_task;
        lv_obj_t* autonLabel = nullptr;
        lv_obj_t* infoLabel = nullptr;
        lv_obj_t* footerLabel = nullptr;
        pros::Mutex uiMutex;
        char autonText[64] = {0};
        char infoText[192] = {0};
        char footerText[64] = {0};
        Screen() = default;
        Screen(const Screen&) = delete;
        Screen& operator=(const Screen&) = delete;
        std::vector<std::string> autonNames;
        int lastPotValue = -1;
        int lastDisplayedAuton = -1;
    public:
        inline static Screen& getInstance() {
            static Screen INSTANCE;
            return INSTANCE;
        }
        RobotState state = PRE_MATCH;
        int selectedAuton = 0;
        void setAutonNames(const std::vector<std::string>& names);
        void initialize();
        void (*auton)();
        void hideSelector();
        void showInfoLabel(const char* text);
        void update();


};
