#pragma once

#include "main.h"
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
        std::unique_ptr<pros::Task> task;
        lv_obj_t* autonLabel = nullptr;
        lv_obj_t* infoLabel = nullptr;
        lv_obj_t* footerLabel = nullptr;
        pros::Mutex stateMutex;
        char autonText[64] = {0};
        char infoText[192] = {0};
        char footerText[64] = {0};
        char infoOverrideText[192] = {0};
        Screen() = default;
        Screen(const Screen&) = delete;
        Screen& operator=(const Screen&) = delete;
        std::vector<std::string> autonNames;
        int selectedAuton = 0;
        int lastPotValue = -1;
        int lastDisplayedAuton = -1;
        bool selectorHidden = false;
        bool lastSelectorHidden = false;
        bool mclPaused = true;
        bool infoOverrideEnabled = false;

        void createLabels();
        void update();
        void refreshSelector();
        void refreshInfo();
        void refreshVisibility();
    public:
        inline static Screen& getInstance() {
            static Screen INSTANCE;
            return INSTANCE;
        }
        RobotState state = PRE_MATCH;
        void setAutonNames(const std::vector<std::string>& names);
        int getSelectedAuton();
        void initialize();
        void setMCLPaused(bool paused);
        void hideSelector();
        void showInfoLabel(const char* text);
};
