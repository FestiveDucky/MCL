#include "screen.h"

#include "devices.h"
#include "lemlib/chassis/odom.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace {
constexpr int POT_MIN = 0;
constexpr int POT_MAX = 4095;
constexpr int POT_SPAN = POT_MAX - POT_MIN + 1;

template <std::size_t N>
bool writeText(char (&buffer)[N], const char* text) {
    const char* safeText = (text == nullptr) ? "" : text;
    char next[N] = {0};
    std::snprintf(next, sizeof(next), "%s", safeText);
    if (std::strncmp(buffer, next, sizeof(buffer)) == 0) return false;
    std::memcpy(buffer, next, sizeof(buffer));
    return true;
}

template <std::size_t N>
void setLabelText(lv_obj_t* label, char (&buffer)[N], const char* text) {
    if (label == nullptr) return;
    if (!writeText(buffer, text)) return;
    lv_label_set_text_static(label, buffer);
}

lv_obj_t* makeLabel(lv_align_t align, int x, int y, lv_color_t color, const lv_font_t* font) {
    lv_obj_t* label = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_align(label, align, x, y);
    return label;
}
} // namespace

void Screen::setAutonNames(const std::vector<std::string>& names) {
    if (!stateMutex.take(20)) return;
    autonNames = names;
    if (autonNames.empty()) selectedAuton = 0;
    else if (selectedAuton >= static_cast<int>(autonNames.size())) selectedAuton = static_cast<int>(autonNames.size()) - 1;
    lastDisplayedAuton = -1;
    lastPotValue = -1;
    stateMutex.give();
}

int Screen::getSelectedAuton() {
    int idx = 0;
    if (!stateMutex.take(20)) return idx;
    idx = selectedAuton;
    stateMutex.give();
    return idx;
}

void Screen::setMCLPaused(bool paused) {
    if (!stateMutex.take(20)) return;
    mclPaused = paused;
    infoOverrideEnabled = false;
    stateMutex.give();
}

void Screen::initialize() {
    if (!stateMutex.take(20)) return;
    const bool alreadyInitialized = (task != nullptr);
    const bool needsDefaultAutons = autonNames.empty();
    stateMutex.give();
    if (alreadyInitialized) return;

    if (needsDefaultAutons) setAutonNames({"Left", "Right", "Skills", "Nothing"});

    createLabels();
    update();

    task = std::make_unique<pros::Task>([this]() {
        while (true) {
            update();
            pros::delay(50);
        }
    });
}

void Screen::createLabels() {
    autonLabel = makeLabel(LV_ALIGN_CENTER, 0, 14, lv_color_hex(0xffffff), &lv_font_montserrat_24);
    infoLabel = makeLabel(LV_ALIGN_TOP_LEFT, 8, 8, lv_color_hex(0xcbd5e1), &lv_font_montserrat_14);
    footerLabel = makeLabel(LV_ALIGN_BOTTOM_MID, 0, -6, lv_color_hex(0x94a3b8), &lv_font_montserrat_14);
}

void Screen::update() {
    refreshSelector();
    refreshInfo();
    refreshVisibility();
}

void Screen::refreshSelector() {
    const int raw = std::clamp(static_cast<int>(potentiometer.get_value()), POT_MIN, POT_MAX);

    std::string name;
    int idx = 0;
    int count = 0;

    if (!stateMutex.take(20)) return;
    if (autonNames.empty()) {
        stateMutex.give();
        return;
    }

    count = static_cast<int>(autonNames.size());
    idx = static_cast<int>((static_cast<long long>(raw - POT_MIN) * count) / POT_SPAN);
    idx = std::clamp(idx, 0, count - 1);
    selectedAuton = idx;
    name = autonNames[idx];
    stateMutex.give();

    if (idx != lastDisplayedAuton) {
        char nextAuton[sizeof(autonText)] = {0};
        std::snprintf(nextAuton, sizeof(nextAuton), "%s\n(%d/%d)", name.c_str(), idx + 1, count);
        setLabelText(autonLabel, autonText, nextAuton);
        lastDisplayedAuton = idx;
    }

    if (raw != lastPotValue) {
        char nextFooter[sizeof(footerText)] = {0};
        const int pct = ((raw - POT_MIN) * 100) / (POT_MAX - POT_MIN);
        std::snprintf(nextFooter, sizeof(nextFooter), "Pot A: %4d (%3d%%)", raw, pct);
        setLabelText(footerLabel, footerText, nextFooter);
        lastPotValue = raw;
    }
}

void Screen::refreshInfo() {
    char nextInfo[sizeof(infoText)] = {0};
    bool useOverride = false;
    bool pausedNow = true;

    if (stateMutex.take(20)) {
        useOverride = infoOverrideEnabled;
        pausedNow = mclPaused;
        if (useOverride) std::snprintf(nextInfo, sizeof(nextInfo), "%s", infoOverrideText);
        stateMutex.give();
    }

    if (!useOverride) {
        if (pausedNow) {
            std::snprintf(nextInfo, sizeof(nextInfo), "MCL paused (enabled in autonomous)");
        } else {
            const lemlib::Pose poseMCL = chassis.getPose();
            const lemlib::Pose poseOLD = lemlib::getOldPose();
            const std::uint32_t calculationTime = lemlib::getCalculationTime();
            const std::int32_t confidenceFront = lemlib::getConfidence();
            std::snprintf(
                nextInfo,
                sizeof(nextInfo),
                "MCL X: %.2f Y: %.2f\nTh: %.2f OldX: %.2f OldY: %.2f\nCalc Time: %u  FrontConf: %d",
                poseMCL.x,
                poseMCL.y,
                poseMCL.theta,
                poseOLD.x,
                poseOLD.y,
                static_cast<unsigned>(calculationTime),
                static_cast<int>(confidenceFront));
        }
    }

    setLabelText(infoLabel, infoText, nextInfo);
}

void Screen::refreshVisibility() {
    bool hidden = false;
    if (!stateMutex.take(20)) return;
    hidden = selectorHidden;
    stateMutex.give();

    if (hidden == lastSelectorHidden) return;

    if (autonLabel != nullptr) {
        if (hidden) lv_obj_add_flag(autonLabel, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(autonLabel, LV_OBJ_FLAG_HIDDEN);
    }

    if (footerLabel != nullptr) {
        if (hidden) lv_obj_add_flag(footerLabel, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(footerLabel, LV_OBJ_FLAG_HIDDEN);
    }

    lastSelectorHidden = hidden;
}

void Screen::hideSelector() {
    if (!stateMutex.take(20)) return;
    selectorHidden = true;
    stateMutex.give();
}

void Screen::showInfoLabel(const char* text) {
    if (!stateMutex.take(20)) return;
    if (text == nullptr) {
        infoOverrideEnabled = false;
        infoOverrideText[0] = '\0';
    } else {
        std::snprintf(infoOverrideText, sizeof(infoOverrideText), "%s", text);
        infoOverrideEnabled = true;
    }
    stateMutex.give();
}
