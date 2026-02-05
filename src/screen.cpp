#include "liblvgl/font/lv_font.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "devices.h"
#include "screen.h"
#include "utils.h"
#include "autons.h"


// void Screen::initialize() {
//     update_task = std::make_unique<pros::Task>([this](){
//         while(true) {
//             update();
//             pros::delay(20);
//         }
//     });

//     autonLabel = lv_label_create(lv_screen_active());
//     lv_label_set_text(autonLabel, "Auton: ~");
//     lv_obj_set_style_text_color(autonLabel, lv_color_hex(0xffffff), LV_PART_MAIN);
//     lv_obj_set_style_text_font(autonLabel, &lv_font_montserrat_48, 0);
//     lv_obj_align(autonLabel, LV_ALIGN_CENTER, 0, 0);

// }

// void Screen::update() {
    
// }

struct BtnData {
    Screen* scr;
    size_t id; // or other info
};

void Screen::btn_event_cb(lv_event_t* e) {
    BtnData* data = static_cast<BtnData*>(lv_event_get_user_data(e));
    Screen* scr = data->scr;
    scr->selectedAuton = data->id;
    lv_label_set_text_fmt(scr->autonLabel, "Auton: %s", scr->autonNames[scr->selectedAuton].c_str());
}

void Screen::initialize() {
    // static lv_style_t style_selected;
    // lv_style_init(&style_selected);
    // lv_style_set_bg_color(&style_selected, LV_STATE_DEFAULT, LV_COLOR_BLUE);
    // lv_style_set_border_width(&style_selected, LV_STATE_DEFAULT, 3);
    // lv_style_set_border_color(&style_selected, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    infoLabel = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(infoLabel, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(infoLabel, LV_ALIGN_TOP_LEFT, 5, 5);

    autonNames = {"Left", "Right", "Skills", "Nothing"};

    update_task = std::make_unique<pros::Task>([this]() {
        while(true) {
            update();
            pros::delay(20);
        }
    });

    autonLabel = lv_label_create(lv_screen_active());
    lv_label_set_text(autonLabel, "Auton: ~");
    lv_obj_set_style_text_color(autonLabel, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(autonLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(autonLabel, LV_ALIGN_TOP_MID, 0, 10);

    size_t num = 0;
    for (size_t i = 0; i < floor(autonNames.size()/2.); i++) {
        for (size_t j = 0; j < ceil(autonNames.size()/2.); j++) {
            lv_obj_t* btn = lv_button_create(lv_screen_active());
            lv_obj_set_size(btn, 180, 50);
            lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 50 + j * 190, 50 + i * 60);

            lv_obj_t* label = lv_label_create(btn);
            lv_label_set_text(label, autonNames[num].c_str());
            lv_obj_center(label);

            BtnData* data = new BtnData{this, num};
            lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, data);

            buttons.push_back(btn);
            num++;
        }
    }
}

void Screen::update() {
    // if (selectedAuton >= 0 && selectedAuton < (int)autonNames.size()) {
    //     lv_label_set_text_fmt(autonLabel, "Auton: %s", autonNames[selectedAuton].c_str());
    // }
}

void Screen::hideSelector() {
    for (auto btn : buttons) {
        lv_obj_add_flag(btn, LV_OBJ_FLAG_HIDDEN);  // hides the button
    }
}

void Screen::showInfoLabel(const char* text) {
    lv_label_set_text(infoLabel, text);
}