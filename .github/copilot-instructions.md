# Copilot Instructions for MCL Robotics Codebase

## Project Overview
This is a VEX Robotics project (PROS framework) for an in-development 2024-2025 season robot called MCL (likely acronym for a team/concept). The codebase combines custom motion control with Monte Carlo Localization (MCL) for precise odometry on a 140"×140" field.

## Architecture

### Core Components

**Chassis & Motion Control** ([include/lemlib/chassis/](include/lemlib/))
- Uses LemLib library for kinematics, path planning, and odometry
- Custom MCL implementation with 4 distance sensors (front, left, right, back) for real-time pose estimation
- MCL tuning parameters in [src/devices.cpp](src/devices.cpp#L4-L50): particle count (500), sensor noise models, field boundaries
- Key files: `chassis.hpp`, `odom.hpp`, `trackingWheel.hpp`

**Device Management** ([include/devices.h](include/devices.h), [src/devices.cpp](src/devices.cpp))
- Centralized externs for all motors, sensors, and actuators
- Motor groups: `left_motor_group`, `right_motor_group`
- Intake system: dual motors (`bottom_intake`, `top_intake`)
- Pneumatic controls: scraper, wing, lifts (horizontal/vertical), descore, flappier
- Rotation sensors for tracking intake/mechanical states
- Distance sensors (ports 3, 15, 17, 21) mounted with precise x/y/heading offsets

**Autonomous System** ([include/autons.h](include/autons.h), [src/autons.cpp](src/autons.cpp))
- Autonomous routines selectable via potentiometer on controller (6 options: Left, Right, Skills, Five Inch, Test, Solo AWP)
- Auton table in [src/main.cpp](src/main.cpp#L21-L28) maps selection UI to function pointers
- Common patterns: `chassis.moveToPoint()`, `chassis.turnToPoint()`, `intake.store()/score()` calls
- MCL toggled on/off mid-routine with `lemlib::toggleMCL()` for critical sections
- Parametrized game element positions and timings (scraperDist, scoreDelay, matchLoaderSpeed)

**Screen UI** ([include/screen.h](include/screen.h), [src/screen.cpp](src/screen.cpp))
- Singleton pattern: `Screen::getInstance()` or use macro `sc`
- Displays auton selector, MCL pose info, and debug labels
- Mutex-protected for thread-safe updates from background task
- LVGL-based rendering to V5 brain screen

## Build & Deployment

**Build System**: Standard PROS makefile
- Command: `make build` (or `make clean build` to force rebuild)
- Compile commands generated: `pros build-compile-commands --no-analytics`
- Target: VEX V5 ARM Cortex-A9 (arm-none-eabi toolchain)
- C++23 standard enabled; warnings on unimplemented features

**Main Entry Points**:
- `void initialize()` - Runs once at startup; initializes chassis, MCL (paused), intake, screen
- `void autonomous()` - Runs selected auton from table
- `void opcontrol()` - Driver-controlled period

## Key Conventions & Patterns

### Motion & Odometry
- **Pose representation**: `lemlib::Pose(x, y, heading)` in inches and radians
- **MCL toggle**: Call `lemlib::toggleMCL()` to pause/resume particle filter (done mid-auton when transitioning between sensor-reliant and relative motion sections)
- **Motion calls are blocking** by default; always include timeout (ms): `chassis.moveToPoint(x, y, timeout, {...options})`
- Motion options include: `maxSpeed`, `forwards`, `headingCorrection`, `async` (for non-blocking)

### Device Access
- All motors/sensors declared as `extern` in [include/devices.h](include/devices.h); instantiated in [src/devices.cpp](src/devices.cpp)
- Pneumatics use `ADI::DigitalOut` (`set_value(true/false)`)
- Motor control: `move(voltage)` for raw, or use wrapper methods like `intake.store(127)`, `intake.score(127)`

### Intake Subsystem
- Dual-motor system with game-piece manipulation logic
- Methods: `store()` (intake pieces), `score()` (eject into goal), `stop()`
- Callable from autons and opcontrol; initialization in [src/intake.cpp](src/intake.cpp)

### State Management
- Global flags (e.g., `removerActivated`, `wingActivated`) track pneumatic toggle states
- Screen state enum: `PRE_MATCH`, `AUTONOMOUS`, `DRIVER` in [include/screen.h](include/screen.h)
- Atomic `mclPaused` flag in [src/main.cpp](src/main.cpp#L40) ensures thread-safe MCL control during autonomous

## Common Tasks

**Adding a New Autonomous Routine**:
1. Write function in [src/autons.cpp](src/autons.cpp) matching signature `void myAuton()`
2. Declare in [include/autons.h](include/autons.h)
3. Add to `AUTONS` array in [src/main.cpp](src/main.cpp#L23) with display name
4. Use `chassis.moveToPoint()`, `intake.store()/score()`, etc.

**Tuning MCL**:
- Edit distance sensor mounts (x/y/heading offsets) and noise parameters in [src/devices.cpp](src/devices.cpp#L4-L50)
- Increase `particleCount` for accuracy (default 500; computational cost trade-off)
- Key parameters: `sigma0XY` (base noise), `kDistXY`/`kTurnXY` (motion uncertainty), `estMsBandwidth` (pose estimation kernel)

**Debugging**:
- Screen shows MCL pose, calculation time, and sensor confidence
- Use `intake.initialize()` to reset intake state on startup
- Controller button presses managed in opcontrol loop with debouncing flags (`*PressedLast`)

## Integration Points & Dependencies

- **PROS Kernel**: V5 motor/sensor API via [include/pros/](include/pros/)
- **LemLib**: Kinematics, odometry, path planning (port IDs configured in [src/devices.cpp](src/devices.cpp))
- **LVGL**: UI rendering library ([include/liblvgl/](include/liblvgl/))
- **fmt library**: String formatting ([include/fmt/](include/fmt/))

## MCLSettings Structure ([include/lemlib/chassis/chassis.hpp](include/lemlib/chassis/chassis.hpp))

All MCL configuration variables live in the `MCLSettings` class. Group them by purpose:

### Particle Management
- **`particleCount`** - Number of position estimates (default 500). Higher = more accurate but slower
- **`neffResampleThreshold`** - Trigger particle resampling when effective particles drop below this fraction (0.0-1.0)

### Motion Model (how the filter predicts where robot moved)
- **`sigma0XY`** - Base position noise per cycle (inches)
- **`kDistXY`** - Extra noise per inch traveled (multiplier)
- **`kTurnXY`** - Extra noise per radian rotated (multiplier)
- **`maxStartPosErrorIn`** - Initial spread of particles from starting position (inches)
- **`clampDeltaSForNoise`** / **`maxDeltaSForNoise`** - Cap movement before applying noise (prevents noise explosions on large jumps)
- **`clampSigmaXY`** / **`maxSigmaXY`** - Cap total uncertainty spread (prevents filters from diverging)

### Pose Estimation (how filter refines its best estimate)
- **`estMsBandwidth`** - Kernel radius for mean-shift clustering (inches) — smaller = tighter clustering
- **`estMsIters`** - Max iterations for mean-shift refinement
- **`estMsEpsStop`** - Convergence threshold (inches) — when to stop refining
- **`estUseHuberRefinement`** - Enable robust outlier rejection during refinement
- **`estUseEmaSmoothing`** - Smooth pose estimates over time with exponential moving average
- **`estAlphaMin`** / **`estAlphaMax`** - EMA smoothing factor range (higher = more smoothing)
- **`estSigmaLo`** / **`estSigmaHi`** - Uncertainty thresholds where EMA smoothing increases
- **`estJumpThresh`** - Innovation threshold (inches) — large jumps trigger protective smoothing
- **`estAlphaJump`** - EMA alpha applied when a jump is detected (more conservative)

### Sensor Measurement Model (how distance sensors update particle weights)
- **`sigmaD`** - Sensor measurement spread (inches) — how much sensor readings vary
- **`zMin`** / **`zMax`** - Valid measurement range (inches, 0.1 to 85.0)
- **`sensorConfMax`** - Sensor confidence value (0-127) that means "fully trust this sensor"
- **`pFloor`** - Minimum likelihood floor (prevents particles from getting zero weight)
- **`wHit`** / **`wRand`** - Blend weights for hit/random models (reserved for future use)

### Field Constraints (keep particles realistic)
- **`fieldHalf`** - Half-field distance from center to wall (70.75 inches for 140"×140" field)
- **`useFieldMargin`** - Shrink valid region inward from walls
- **`fieldMarginIn`** - Margin distance from walls (inches)
- **`clampOobParticles`** - Clamp particles that go out-of-bounds to field edges
- **`penalizeOobParticles`** - Downweight out-of-bounds particles instead of clamping
- **`oobWeightMult`** - Weight multiplier for out-of-bounds particles (e.g., 1e-3 = 0.001x)

### "No Hit" Model (treat far-away measurements as evidence of no wall)
- **`useNoHitModel`** - When sensor reads > zMax, treat it as "no wall here" evidence
- **`noHitPenalty`** - Penalty applied if particle expected a wall but sensor saw nothing

### Distance Sensors ([src/devices.cpp](src/devices.cpp#L6-L9))
```cpp
cfg.distanceSensors = {
    {17, {-4.6f, 6.5f, 0.0f}},            // Port 17: Front
    {15, {-5.8f, -2.3f, -1.57079632679f}}, // Port 15: Left (-π/2 rad)
    {21, {5.8f, -2.3f, 1.57079632679f}},   // Port 21: Right (π/2 rad)
    {3, {3.5f, -3.2f, M_PI}}               // Port 3: Back (π rad)
};
```
Each sensor has `{port, {xOffset, yOffset, headingOffset}}` where offsets are relative to robot center.

## Tuning Workflows: Variable Changing Patterns

### MCL Tuning Workflow (in [src/devices.cpp](src/devices.cpp#L4-L57))

**Scenario 1: Robot drifts during autonomous (pose estimate diverges)**
```cpp
// Problem: Particle cloud spreading too wide
// Edit makeMCLSettings():

cfg.sigma0XY = 0.03f;          // ↓ Reduce from 0.06f (less base noise)
cfg.kDistXY = 0.40f;           // ↓ Reduce from 0.58f (less drift per inch)
cfg.kTurnXY = 0.15f;           // ↓ Reduce from 0.24f (less turn uncertainty)
cfg.maxSigmaXY = 1.20f;        // ↓ Reduce from 1.75f (cap uncertainty tighter)

// Then rebuild: make clean build
```

**Scenario 2: MCL pose updates are too jerky/jittery**
```cpp
// Problem: Pose estimates jumping around
// Increase smoothing in makeMCLSettings():

cfg.estUseEmaSmoothing = true; // Ensure enabled
cfg.estAlphaMin = 0.25f;       // ↑ Increase from 0.10f (more smoothing at low uncertainty)
cfg.estAlphaMax = 0.92f;       // ↑ Increase from 0.80f (more smoothing at high uncertainty)
cfg.estJumpThresh = 30.0f;     // ↑ Increase from 22.0f (higher threshold before triggering jump)
cfg.estAlphaJump = 0.95f;      // ↑ Increase from 0.84f (more conservative when jump detected)

// Rebuild and test: make clean build
```

**Scenario 3: Sensor readings are being ignored (low sensor confidence)**
```cpp
// Problem: Distance sensors not updating pose enough
// Boost sensor trust in makeMCLSettings():

cfg.sigmaD = 2.0f;             // ↓ Reduce from 3.0f (sensor noise smaller = more trusted)
cfg.sensorConfMax = 127.0f;    // ↑ Increase from 63.0f (max confidence more attainable)
cfg.pFloor = 1e-5;             // ↓ Reduce from 5e-5 (particles can get lower likelihood)
cfg.useNoHitModel = true;      // Ensure enabled (use >85" readings as "no wall" evidence)
```

**Scenario 4: Particles escape field boundaries (unrealistic positions)**
```cpp
// Problem: Out-of-bounds particles not being controlled
// Strengthen field constraints in makeMCLSettings():

cfg.useFieldMargin = true;     // Ensure enabled
cfg.fieldMarginIn = 8.0f;      // ↑ Increase from 5.5f (shrink valid region more)
cfg.clampOobParticles = true;  // Clamp particles that go OOB to field edge
cfg.penalizeOobParticles = true; // Also penalize (double-enforce)
cfg.oobWeightMult = 1e-4;      // ↓ Reduce from 1e-3 (OOB particles get 0.01% weight)
```

**Scenario 5: MCL calculation is too slow**
```cpp
// Problem: Pose estimation takes too long (lagging autonomous)
// Reduce computation in makeMCLSettings():

cfg.particleCount = 250;       // ↓ Reduce from 500 (half the particles)
cfg.estMsIters = 3;            // ↓ Reduce from 6 (fewer mean-shift iterations)
cfg.estHuberIters = 1;         // ↓ Reduce from 3 (skip some Huber refinement)
cfg.neffResampleThreshold = 0.50f; // ↓ Reduce from 0.80f (resample less often)

// Trade: Faster but less accurate. Only if timing is critical.
```

### PID Tuning Workflow (in [src/devices.cpp](src/devices.cpp#L108-L140))

**Scenario: Robot overshoots target during moveToPoint**
```cpp
// Problem: Lateral motion oscillates around target
// Edit lateral_controller in devices.cpp:

lemlib::ControllerSettings lateral_controller(
    3.5,     // ↓ kP from 4.23 (reduce proportional gain)
    0,       // kI stays 0
    12,      // ↑ kD from 9 (increase damping to stop oscillation)
    0,       // windupRange
    .25, 500, // small error range stays same
    .7, 2000, // large error range stays same
    20       // slew stays same
);
```

**Scenario: Robot can't reach target angle (undershoot)**
```cpp
// Problem: Angular motion stops short of target heading
// Edit angular_controller in devices.cpp:

lemlib::ControllerSettings angular_controller(
    4.0,     // ↑ kP from 3.175 (increase proportional gain)
    5,       // ↑ kI from 3 (increase integral to push to target)
    14,      // kD stays same
    3,       // windupRange stays same
    .5, 500, // small error stays same
    1, 800,  // large error stays same
    0        // slew stays same
);
```

### Distance Sensor Calibration ([src/devices.cpp](src/devices.cpp#L8-L12))

**Scenario: Front sensor reads are consistently 2" off**
```cpp
// Problem: Distance measurement bias
// Adjust sensor mount offset in makeMCLSettings():

cfg.distanceSensors = {
    {17, {-4.6f, 8.5f, 0.0f}},   // ↑ Y offset from 6.5f (move front sensor forward 2")
    // ... other sensors
};
// This changes where robot thinks sensor is mounted, compensating for bias
```

**Scenario: Left/Right sensors aren't symmetric**
```cpp
// Problem: Left sensor at port 15 reads 3" different from right sensor at port 21
// Adjust mount symmetry in makeMCLSettings():

cfg.distanceSensors = {
    {17, {-4.6f, 6.5f, 0.0f}},    // Front (unchanged)
    {15, {-5.0f, -2.3f, -1.57079632679f}}, // ↓ Left X from -5.8f (move left sensor forward 0.8")
    {21, {5.8f, -2.3f, 1.57079632679f}},   // Right (unchanged)
    {3, {3.5f, -3.2f, M_PI}}       // Back (unchanged)
};
```

### Rebuild & Test Cycle

After ANY variable change:
```bash
# 1. Edit src/devices.cpp (variables in makeMCLSettings())
# 2. Rebuild project
make clean build

# 3. Optional: Generate compile commands for IDE
pros build-compile-commands --no-analytics

# 4. Deploy to robot
pros upload

# 5. Run autonomous routine and observe MCL pose on screen
# (Check screen task output in src/main.cpp line 82-100)
```

## File Organization
- `include/` - All headers; mirrors src structure
- `src/` - Implementation; main.cpp is startup/loop, autons.cpp contains motion routines
- `include/lemlib/` - Custom/wrapped MCL and chassis extensions
- `firmware/`, `bin/` - Build outputs (linker scripts, binaries, compilation databases)
