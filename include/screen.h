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

extern int selectedAuton;

class Screen {
    private:
        
        std::unique_ptr<pros::Task> update_task;
        lv_obj_t* autonLabel;
        lv_obj_t* infoLabel;
        Screen() = default;
        Screen(const Screen&) = delete;
        Screen& operator=(const Screen&) = delete;
        std::vector<std::string> autonNames;
        std::vector<lv_obj_t*> buttons;

    static void btn_event_cb(lv_event_t* e);
    public:
        inline static Screen& getInstance() {
            static Screen INSTANCE;
            return INSTANCE;
        }
        RobotState state = PRE_MATCH;
        int selectedAuton = -1;
        void initialize();
        void (*auton)();
        void hideSelector();
        void showInfoLabel(const char* text);
        void update();


};