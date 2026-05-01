#pragma once

#include "main.h"
#include <cstddef>
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
        struct PathSample {
            float x = 0.0f;
            float y = 0.0f;
        };

        std::unique_ptr<pros::Task> task;
        std::unique_ptr<pros::Task> pathTask;
        lv_obj_t* field = nullptr;
        lv_obj_t* pathLine = nullptr;
        lv_obj_t* robotDot = nullptr;
        lv_obj_t* selectorLabel = nullptr;
        lv_obj_t* infoLabel = nullptr;
        lv_obj_t* legendLabel = nullptr;
        std::vector<lv_obj_t*> particleDots;
        std::vector<PathSample> pathSamples;
        std::vector<lv_point_precise_t> pathLinePoints;
        pros::Mutex stateMutex;
        char selectorText[160] = {0};
        char infoText[256] = {0};
        char infoOverrideText[256] = {0};
        Screen() = default;
        Screen(const Screen&) = delete;
        Screen& operator=(const Screen&) = delete;
        std::vector<std::string> autonNames;
        int selectedAuton = 0;
        bool selectorHidden = false;
        bool lastSelectorHidden = false;
        bool mclPaused = true;
        bool infoOverrideEnabled = false;

        void createUI();
        void samplePath();
        void refreshPath();
        void refreshField();
        void syncParticleDots(std::size_t count);
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
