#include "screen.h"

#include "devices.h"
#include "lemlib/chassis/odom.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace {
constexpr int POT_MIN = 0;
constexpr int POT_MAX = 4095;
constexpr int POT_SPAN = POT_MAX - POT_MIN + 1;
constexpr int FIELD_SIZE_PX = 216;
constexpr int FIELD_TILE_COUNT = 6;
constexpr int FIELD_PADDING_PX = 12;
constexpr int PARTICLE_DOT_SIZE_PX = 3;
constexpr int ROBOT_DOT_SIZE_PX = 7;
constexpr int PATH_LINE_WIDTH_PX = 2;
constexpr int PATH_SAMPLE_INTERVAL_MS = 100;
constexpr std::size_t PATH_MAX_POINTS = 1024;
constexpr float PATH_MIN_DISTANCE_IN = 2.0f;
constexpr float PATH_START_RESEED_DISTANCE_IN = 24.0f;
constexpr float FALLBACK_FIELD_HALF_IN = 70.75f;

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

lv_obj_t* makeLabel(lv_obj_t* parent, int width, lv_color_t color, const lv_font_t* font) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_width(label, width);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    return label;
}

lv_obj_t* makeBox(lv_obj_t* parent, int width, int height, lv_color_t bgColor, lv_color_t borderColor, int borderWidth) {
    lv_obj_t* box = lv_obj_create(parent);
    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, width, height);
    lv_obj_set_style_bg_color(box, bgColor, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(box, borderColor, 0);
    lv_obj_set_style_border_width(box, borderWidth, 0);
    lv_obj_set_style_radius(box, 0, 0);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    return box;
}

lv_obj_t* makeDot(lv_obj_t* parent, int size, lv_color_t color) {
    lv_obj_t* dot = lv_obj_create(parent);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, size, size);
    lv_obj_set_style_bg_color(dot, color, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_remove_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    return dot;
}

const char* stateName(RobotState state) {
    switch (state) {
        case AUTONOMOUS: return "AUTONOMOUS";
        case DRIVER: return "DRIVER";
        case PRE_MATCH:
        default: return "PRE-MATCH";
    }
}

float fieldHalfInches() {
    const float fieldHalf = lemlib::getMCLSettings().fieldHalf;
    return fieldHalf > 0.0f ? fieldHalf : FALLBACK_FIELD_HALF_IN;
}

int fieldPixelPointX(float x) {
    const float half = fieldHalfInches();
    const float span = std::max(half * 2.0f, 1.0f);
    const float normalized = std::clamp((x + half) / span, 0.0f, 1.0f);
    return static_cast<int>(std::lround(normalized * static_cast<float>(FIELD_SIZE_PX - 1)));
}

int fieldPixelPointY(float y) {
    const float half = fieldHalfInches();
    const float span = std::max(half * 2.0f, 1.0f);
    const float normalized = std::clamp(1.0f - ((y + half) / span), 0.0f, 1.0f);
    return static_cast<int>(std::lround(normalized * static_cast<float>(FIELD_SIZE_PX - 1)));
}

int fieldPixelX(float x, int dotSize) {
    return std::clamp(fieldPixelPointX(x) - dotSize / 2, 0, FIELD_SIZE_PX - dotSize);
}

int fieldPixelY(float y, int dotSize) {
    return std::clamp(fieldPixelPointY(y) - dotSize / 2, 0, FIELD_SIZE_PX - dotSize);
}

int fieldPixelEdgeX(float x) {
    const float half = fieldHalfInches();
    const float span = std::max(half * 2.0f, 1.0f);
    const float normalized = std::clamp((x + half) / span, 0.0f, 1.0f);
    return static_cast<int>(std::lround(normalized * static_cast<float>(FIELD_SIZE_PX)));
}

int fieldPixelEdgeY(float y) {
    const float half = fieldHalfInches();
    const float span = std::max(half * 2.0f, 1.0f);
    const float normalized = std::clamp(1.0f - ((y + half) / span), 0.0f, 1.0f);
    return static_cast<int>(std::lround(normalized * static_cast<float>(FIELD_SIZE_PX)));
}

void placeDot(lv_obj_t* dot, const lemlib::Pose& pose, int dotSize) {
    if (dot == nullptr) return;
    lv_obj_set_pos(dot, fieldPixelX(static_cast<float>(pose.x), dotSize), fieldPixelY(static_cast<float>(pose.y), dotSize));
}
} // namespace

void Screen::setAutonNames(const std::vector<std::string>& names) {
    if (!stateMutex.take(20)) return;
    autonNames = names;
    if (autonNames.empty()) selectedAuton = 0;
    else if (selectedAuton >= static_cast<int>(autonNames.size())) selectedAuton = static_cast<int>(autonNames.size()) - 1;
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

    createUI();
    update();

    task = std::make_unique<pros::Task>([this]() {
        while (true) {
            update();
            pros::delay(200);
        }
    });

    pathTask = std::make_unique<pros::Task>([this]() {
        while (true) {
            samplePath();
            pros::delay(PATH_SAMPLE_INTERVAL_MS);
        }
    });
}

void Screen::createUI() {
    lv_obj_t* screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0f172a), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    field = makeBox(screen, FIELD_SIZE_PX, FIELD_SIZE_PX, lv_color_hex(0x16324f), lv_color_hex(0xe2e8f0), 2);
    lv_obj_align(field, LV_ALIGN_LEFT_MID, FIELD_PADDING_PX, 0);

    const int tileSize = FIELD_SIZE_PX / FIELD_TILE_COUNT;
    for (int i = 1; i < FIELD_TILE_COUNT; i++) {
        lv_obj_t* vertical = makeBox(field, 1, FIELD_SIZE_PX, lv_color_hex(0x5b7083), lv_color_hex(0x5b7083), 0);
        lv_obj_set_pos(vertical, i * tileSize, 0);

        lv_obj_t* horizontal = makeBox(field, FIELD_SIZE_PX, 1, lv_color_hex(0x5b7083), lv_color_hex(0x5b7083), 0);
        lv_obj_set_pos(horizontal, 0, i * tileSize);
    }

    const auto& cfg = lemlib::getMCLSettings();
    for (const auto& element : cfg.fieldElements) {
        const int left = std::min(fieldPixelEdgeX(element.xMin), fieldPixelEdgeX(element.xMax));
        const int right = std::max(fieldPixelEdgeX(element.xMin), fieldPixelEdgeX(element.xMax));
        const int top = std::min(fieldPixelEdgeY(element.yMin), fieldPixelEdgeY(element.yMax));
        const int bottom = std::max(fieldPixelEdgeY(element.yMin), fieldPixelEdgeY(element.yMax));
        const int width = std::max(1, right - left);
        const int height = std::max(1, bottom - top);

        lv_obj_t* box = makeBox(field, width, height, lv_color_hex(0x7f1d1d), lv_color_hex(0xef4444), 1);
        lv_obj_set_style_bg_opa(box, LV_OPA_30, 0);
        lv_obj_set_pos(box, std::clamp(left, 0, FIELD_SIZE_PX - 1), std::clamp(top, 0, FIELD_SIZE_PX - 1));
    }

    pathLine = lv_line_create(field);
    lv_obj_set_size(pathLine, FIELD_SIZE_PX, FIELD_SIZE_PX);
    lv_obj_set_pos(pathLine, 0, 0);
    lv_obj_remove_flag(pathLine, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(pathLine, LV_OPA_TRANSP, 0);
    lv_obj_set_style_line_color(pathLine, lv_color_hex(0x7dd3fc), 0);
    lv_obj_set_style_line_opa(pathLine, LV_OPA_80, 0);
    lv_obj_set_style_line_width(pathLine, PATH_LINE_WIDTH_PX, 0);
    lv_obj_set_style_line_rounded(pathLine, true, 0);
    lv_obj_add_flag(pathLine, LV_OBJ_FLAG_HIDDEN);

    pathSamples.reserve(PATH_MAX_POINTS);
    pathLinePoints.reserve(PATH_MAX_POINTS);

    particleDots.reserve(static_cast<std::size_t>(std::max(0, lemlib::getMCLSettings().particleCount)));
    syncParticleDots(static_cast<std::size_t>(std::max(0, lemlib::getMCLSettings().particleCount)));

    robotDot = makeDot(field, ROBOT_DOT_SIZE_PX, lv_color_hex(0x22c55e));
    lv_obj_add_flag(robotDot, LV_OBJ_FLAG_HIDDEN);

    const int panelWidth = LV_HOR_RES - FIELD_SIZE_PX - (FIELD_PADDING_PX * 3);
    selectorLabel = makeLabel(screen, panelWidth, lv_color_hex(0xf8fafc), &lv_font_montserrat_24);
    lv_obj_align(selectorLabel, LV_ALIGN_TOP_RIGHT, -FIELD_PADDING_PX, FIELD_PADDING_PX);

    infoLabel = makeLabel(screen, panelWidth, lv_color_hex(0xcbd5e1), &lv_font_montserrat_14);
    lv_obj_align(infoLabel, LV_ALIGN_TOP_RIGHT, -FIELD_PADDING_PX, 92);

    legendLabel = makeLabel(screen, panelWidth, lv_color_hex(0x94a3b8), &lv_font_montserrat_14);
    lv_obj_align(legendLabel, LV_ALIGN_BOTTOM_RIGHT, -FIELD_PADDING_PX, -FIELD_PADDING_PX);
    lv_label_set_text_static(legendLabel, "Green: robot\nWhite: particles\nBlue: path\nRed: field elements");
}

void Screen::update() {
    refreshSelector();
    refreshInfo();
    refreshField();
    refreshVisibility();
}

void Screen::refreshSelector() {
    const int raw = std::clamp(static_cast<int>(potentiometer.get_value()), POT_MIN, POT_MAX);
    int idx = 0;
    int count = 0;
    std::string name;

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

    const int pct = ((raw - POT_MIN) * 100) / (POT_MAX - POT_MIN);
    char nextSelector[sizeof(selectorText)] = {0};
    std::snprintf(nextSelector,
                  sizeof(nextSelector),
                  "%s\nAuton: %s (%d/%d)\nPot A: %4d (%3d%%)",
                  stateName(state),
                  name.c_str(),
                  idx + 1,
                  count,
                  raw,
                  pct);
    setLabelText(selectorLabel, selectorText, nextSelector);
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
        const lemlib::Pose pose = chassis.getPose();
        const std::size_t particleCount = lemlib::getParticlesSnapshot().size();
        const std::uint32_t calculationTime = lemlib::getCalculationTime();
        const std::int32_t confidenceFront = lemlib::getConfidence();

        if (pausedNow) {
            std::snprintf(nextInfo,
                          sizeof(nextInfo),
                          "MCL: paused\nRobot X: %.2f\nRobot Y: %.2f\nTheta: %.2f\nParticles: %u",
                          pose.x,
                          pose.y,
                          pose.theta,
                          static_cast<unsigned>(particleCount));
        } else {
            std::snprintf(
                nextInfo,
                sizeof(nextInfo),
                "MCL: running\nRobot X: %.2f\nRobot Y: %.2f\nTheta: %.2f\nParticles: %u\nCalc: %u us  Front: %d",
                pose.x,
                pose.y,
                pose.theta,
                static_cast<unsigned>(particleCount),
                static_cast<unsigned>(calculationTime),
                static_cast<int>(confidenceFront));
        }
    }

    setLabelText(infoLabel, infoText, nextInfo);
}

void Screen::samplePath() {
    const lemlib::Pose pose = chassis.getPose();

    if (!stateMutex.take(20)) return;

    if (pathSamples.empty()) {
        pathSamples.push_back({static_cast<float>(pose.x), static_cast<float>(pose.y)});
        stateMutex.give();
        return;
    }

    const PathSample& last = pathSamples.back();
    const float dx = static_cast<float>(pose.x) - last.x;
    const float dy = static_cast<float>(pose.y) - last.y;
    const float distance = std::hypot(dx, dy);
    if (distance < PATH_MIN_DISTANCE_IN) {
        stateMutex.give();
        return;
    }

    // Treat a large first-point jump as the auton start pose being seeded after screen init.
    if (pathSamples.size() == 1 && distance >= PATH_START_RESEED_DISTANCE_IN) {
        pathSamples.back() = {static_cast<float>(pose.x), static_cast<float>(pose.y)};
        stateMutex.give();
        return;
    }

    if (pathSamples.size() >= PATH_MAX_POINTS) pathSamples.erase(pathSamples.begin());
    pathSamples.push_back({static_cast<float>(pose.x), static_cast<float>(pose.y)});
    stateMutex.give();
}

void Screen::refreshPath() {
    if (pathLine == nullptr) return;

    std::vector<PathSample> samples;
    if (stateMutex.take(20)) {
        samples = pathSamples;
        stateMutex.give();
    }

    pathLinePoints.clear();
    pathLinePoints.reserve(samples.size());
    for (const PathSample& sample : samples) {
        pathLinePoints.push_back({
            static_cast<lv_value_precise_t>(fieldPixelPointX(sample.x)),
            static_cast<lv_value_precise_t>(fieldPixelPointY(sample.y))
        });
    }

    if (pathLinePoints.size() < 2) {
        lv_obj_add_flag(pathLine, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_line_set_points_mutable(pathLine, pathLinePoints.data(), static_cast<std::uint32_t>(pathLinePoints.size()));
    lv_obj_remove_flag(pathLine, LV_OBJ_FLAG_HIDDEN);
}

void Screen::refreshField() {
    if (field == nullptr || robotDot == nullptr) return;

    refreshPath();

    const std::vector<Particle> particles = lemlib::getParticlesSnapshot();
    syncParticleDots(particles.size());

    for (std::size_t i = 0; i < particleDots.size(); i++) {
        lv_obj_t* dot = particleDots[i];
        if (i >= particles.size()) {
            lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        placeDot(dot, particles[i].pose_, PARTICLE_DOT_SIZE_PX);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_HIDDEN);
    }

    placeDot(robotDot, chassis.getPose(), ROBOT_DOT_SIZE_PX);
    lv_obj_remove_flag(robotDot, LV_OBJ_FLAG_HIDDEN);
}

void Screen::syncParticleDots(std::size_t count) {
    if (field == nullptr) return;

    while (particleDots.size() < count) {
        lv_obj_t* dot = makeDot(field, PARTICLE_DOT_SIZE_PX, lv_color_hex(0xffffff));
        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
        particleDots.push_back(dot);
    }

    if (robotDot != nullptr) {
        lv_obj_move_to_index(robotDot, static_cast<int32_t>(lv_obj_get_child_count(field) - 1));
    }
}

void Screen::refreshVisibility() {
    bool hidden = false;
    if (!stateMutex.take(20)) return;
    hidden = selectorHidden;
    stateMutex.give();

    if (hidden == lastSelectorHidden) return;

    if (selectorLabel != nullptr) {
        if (hidden) lv_obj_add_flag(selectorLabel, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(selectorLabel, LV_OBJ_FLAG_HIDDEN);
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
