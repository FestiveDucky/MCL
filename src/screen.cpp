#include "main.h"
#include "pros/rtos.hpp"
#include "devices.h"
#include "screen.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {
constexpr int POT_MIN = 0;
constexpr int POT_MAX = 4095;
constexpr int POT_SPAN = POT_MAX - POT_MIN + 1;
}

void Screen::setAutonNames(const std::vector<std::string>& names) {
    autonNames = names;
    if (autonNames.empty()) selectedAuton = 0;
    else if (selectedAuton >= static_cast<int>(autonNames.size())) selectedAuton = static_cast<int>(autonNames.size()) - 1;
}

void Screen::initialize() {
    if (autonNames.empty()) autonNames = {"Left", "Right", "Skills", "Nothing"};

    autonLabel = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(autonLabel, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(autonLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(autonLabel, LV_ALIGN_CENTER, 0, 14);
    lv_label_set_text_static(autonLabel, autonText);

    infoLabel = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(infoLabel, lv_color_hex(0xcbd5e1), LV_PART_MAIN);
    lv_obj_set_style_text_font(infoLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(infoLabel, LV_ALIGN_TOP_LEFT, 8, 8);
    lv_label_set_text_static(infoLabel, infoText);

    footerLabel = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(footerLabel, lv_color_hex(0x94a3b8), LV_PART_MAIN);
    lv_obj_set_style_text_font(footerLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(footerLabel, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_label_set_text_static(footerLabel, footerText);

    update_task = std::make_unique<pros::Task>([this]() {
        while (true) {
            update();
            pros::delay(30);
        }
    });
}

void Screen::update() {
    if (autonNames.empty()) return;

    const int raw = std::clamp(static_cast<int>(potentiometer.get_value()), POT_MIN, POT_MAX);
    const int count = static_cast<int>(autonNames.size());

    int idx = (static_cast<long long>(raw - POT_MIN) * count) / POT_SPAN;
    if (idx < 0) idx = 0;
    if (idx >= count) idx = count - 1;

    selectedAuton = idx;

    if (!uiMutex.take(20)) return;
    if (idx != lastDisplayedAuton) {
        std::snprintf(
            autonText,
            sizeof(autonText),
            "%s\n(%d/%d)",
            autonNames[idx].c_str(),
            idx + 1,
            count);
        lv_label_set_text_static(autonLabel, autonText);
        lastDisplayedAuton = idx;
    }

    // Keep this updated so the operator can see exactly where the selector is.
    if (raw != lastPotValue) {
        const int pct = ((raw - POT_MIN) * 100) / (POT_MAX - POT_MIN);
        std::snprintf(footerText, sizeof(footerText), "Pot A: %4d (%3d%%)", raw, pct);
        lv_label_set_text_static(footerLabel, footerText);
        lastPotValue = raw;
    }
    uiMutex.give();
}

void Screen::hideSelector() {
    if (!uiMutex.take(20)) return;
    if (autonLabel != nullptr) lv_obj_add_flag(autonLabel, LV_OBJ_FLAG_HIDDEN);
    if (footerLabel != nullptr) lv_obj_add_flag(footerLabel, LV_OBJ_FLAG_HIDDEN);
    uiMutex.give();
}

void Screen::showInfoLabel(const char* text) {
    if (!uiMutex.take(20)) return;
    if (infoLabel != nullptr) {
        const char* safeText = (text == nullptr) ? "" : text;
        char nextInfo[sizeof(infoText)] = {0};
        std::snprintf(nextInfo, sizeof(nextInfo), "%s", safeText);
        if (std::strncmp(infoText, nextInfo, sizeof(infoText)) != 0) {
            std::snprintf(infoText, sizeof(infoText), "%s", safeText);
            lv_label_set_text_static(infoLabel, infoText);
        }
    }
    uiMutex.give();
}
