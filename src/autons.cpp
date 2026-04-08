#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "intake.h"

void move(double power, double turn, bool swing=false, double time=10000) {
    chassis.cancelAllMotions();

    int left = power + turn;
    int right = power - turn;
    // double t = time; 

    if (swing && left < 0) {left = 0;}
    if (swing && right < 0) {right = 0;}

    left_motor_group.move(left);
    right_motor_group.move(right);
    pros::delay(time);
    left_motor_group.brake();
    right_motor_group.brake();
    
    // left_center_motor.move(left);
    // left_back_motor.move(left);
    // right_front_motor.move(left);
    // right_center_motor.move(left);
    // right_back_motor.move(left);
}

void soloAWP() {
    chassis.setPose(13, -47, 90);
    descore.set_value(true);

    int scraperDist = 66;
    int scraperTimeout = 900;
    float matchLoaderSpeed = 52.5;
    int scraperDelay = 0;
    int scoreDelay = 800;

    // First match loader
    chassis.moveToPoint(46.5, -48, 1000, {}, false);
    scraper_piston.toggle();
    chassis.turnToPoint(46.5, -67, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(46.5, -67, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Score in long goal
    chassis.moveToPoint(48, -24, 1000, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(700);
    scraper_piston.toggle();
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Intake center 3 balls
    chassis.moveToPoint(48, -48, 1000, {}, false);
    chassis.turnToPoint(18, -18, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(18, -24, 800, {}, true);
    pros::delay(700);
    scraper_piston.toggle();
    
    // Move to second set of 3 balls
    chassis.turnToPoint(-24, -24, 400, {}, false);
    scraper_piston.toggle();
    chassis.moveToPoint(-24, -24, 1100, {}, true);
    pros::delay(900);
    scraper_piston.toggle();
    intake.stop();

    // Score middle
    lemlib::toggleMCL();
    chassis.turnToPoint(-4, -4, 600, {.forwards=false}, false); //was -6, -6
    top_intake.move(-100);
    bottom_intake.move(-20);
    chassis.moveToPoint(-4, -4, 750, {.forwards=false}, true); // was -6, -6
    chassis.moveToPoint(-6, -6, 750, {}, true);
    // chassis.moveToPoint(-4, -4, 750, {.forwards=false}, true);
    intake.score(100, true);
    pros::delay(scoreDelay + 50);
    //intake.outtake(40);
    middlescore_piston.set_value(true);
    top_score.set_value(true);
    bottom_intake.move(127);


    //Second match loader
    chassis.moveToPoint(-43, -48, 1200, {}, true);
    pros::delay(150);
    chassis.turnToPoint(-43, -65, 1200, {}, true);
    pros::delay(150);
    chassis.moveToPoint(-43, -65, 1200, {}, true);
    pros::delay(150);

    //score on second long goal
    chassis.turnToPoint(-45, -24, 800, {.forwards=false}, true);

    intake.store(127);
    lemlib::toggleMCL();
    chassis.moveToPoint(-45, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    pros::delay(5000);

}

void skills() {
    chassis.setPose(-5, -46, 0);
    descore.set_value(true);

    int scraperDist = 69.5;
    int scraperTimeout = 1000;
    float matchLoaderSpeed = 55;
    int scraperDelay = 1800;
    int scoreDelay = 2400;
    int goalPosition = 24;
    int iterTime = 100;
    bool jiggle = true;


    // Get first 4 balls & score in middle
    intake.store(127);
    chassis.turnToPoint(-22, -26, 800, {}, true);
    chassis.moveToPoint(-22, -26, 1000,{}, true);
    pros::delay(525);
    scraper_piston.toggle();
    pros::delay(200);

    lemlib::toggleMCL();
    chassis.turnToPoint(3, 0, 800, {.forwards = false}, false);
    intake.stop();
    top_intake.move(-100);
    bottom_intake.move(-20);
    chassis.moveToPoint(3, 0, 1000, {.forwards = false}, false);
    intake.score(70, true);
    move(30, 0 ,false, 200);
    move(-20,0,false, 1000);
    // The above takes 6.8s

    // First match loader
    chassis.moveToPoint(-50, -50, 1000, {}, false);
    intake.stop();
    intake.store(127);
    lemlib::toggleMCL();  
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, true);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout-150, {.maxSpeed=matchLoaderSpeed},false);
    
    // Jiggle
    if (jiggle) { 
        for (int i = 0; i < scraperDelay/iterTime; i++) {
            int sign = i % 2 ? -1 : 1; 
            move(sign * 30, 0, false, iterTime + sign * 30);
        }
    } else {
        move(30, 0, false, scraperDelay);
    }
    

    // Move through tunnel
    // chassis.moveToPoint(-46.5, -47, 800, {.forwards = false},true);
    chassis.turnToPoint(-61.5, -36, 650,{.forwards = false}, true);
    intake.stop();
    chassis.moveToPoint(-61.5, -36, 900,{.forwards = false},true);
    chassis.turnToPoint(-61.5, 36, 650,{.forwards = false},true);
    chassis.moveToPoint(-61.5, 36, 1600,{.forwards = false},true);
    chassis.turnToPoint(-48, 36,800, {},true);
    chassis.moveToPoint(-48, 36, 800, {}, true);

    // Score first 6 balls
    chassis.turnToPoint(-48, goalPosition, 600, {.forwards = false}, true);
    chassis.moveToPoint(-48, goalPosition, 800, {.forwards = false}, true);
    pros::delay(600);
    intake.score(110);
    pros::delay(scoreDelay);
    intake.stop();

    // Second match loader
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    
    // Jiggle
    if (jiggle) { 
        for (int i = 0; i < scraperDelay/iterTime; i++) {
            int sign = i % 2 ? -1 : 1; 
            move(sign * 30, 0, false, iterTime + sign * 30);
        }
    } else {
        move(30, 0, false, scraperDelay);
    }
    
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48, goalPosition, 1200, {.forwards = false}, true);
    pros::delay(1000);
    intake.score(110);
    pros::delay(scoreDelay);
    intake.stop();

    // SHOULD BE AT 25s 

    // Park ball clear
    scraper_piston.set_value(true);
    chassis.moveToPoint(-48, 53, 750, {}, true);
    chassis.turnToPoint(-13, 64, 600, {}, true);
    chassis.moveToPoint(-13, 64, 900, {}, true);
    pros::delay(750);
    intake.store(127);
    pros::delay(150);
    
    move(45, 5, false, 1600);  // 400
    move(70, 5, false, 250);
    scraper_piston.set_value(false);
    move(70, 5, false, 250);

    chassis.turnToHeading(0, 800, {}, false);
    resetPositionFromTwoDistanceSensors(0, 2);

    chassis.turnToPoint(22, 26, 500, {.forwards=false}, false);
    chassis.moveToPoint(22, 26, 1000, {.forwards=false}, false);
    chassis.turnToPoint(-1.5, 0, 500, {.forwards=false}, true);
    pros::delay(300);
    lemlib::toggleMCL();
    intake.stop();
    top_intake.move(-127);
    bottom_intake.move(-40);
    chassis.moveToPoint(-1.5, 0, 1000, {.forwards=false}, false);
    intake.score(65, true);
    chassis.turnToHeading(50, 800, {}, true);
    move(30, 0 ,false, 200);
    move(-20,0,false, 2000);

    // Move across field
    chassis.turnToPoint(46.5, 36, 800, {}, true);
    chassis.moveToPoint(46.5, 36, 1400,{}, true);
    pros::delay(500);
    intake.stop();
    lemlib::toggleMCL();

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, true);
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed}, false);
    
    // Jiggle
    if (jiggle) { 
        for (int i = 0; i < scraperDelay/iterTime; i++) {
            int sign = i % 2 ? -1 : 1; 
            move(sign * 30, 0, false, iterTime + sign * 30);
        }
    } else {
        move(30, 0, false, scraperDelay);
    }

    // Move through tunnel
    // chassis.moveToPoint(46.5, 47, 800,{.forwards=false}, true);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false}, true);
    chassis.moveToPoint(61.5, 36, 900,{.forwards = false}, true);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false}, true);
    intake.stop();
    chassis.moveToPoint(61.5, -36, 1600,{.forwards = false}, true);
    chassis.turnToPoint(48, -36, 800, {}, true);
    chassis.moveToPoint(48, -36, 800, {},  true);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -goalPosition, 600, {.forwards = false}, true);
    chassis.moveToPoint(48, -goalPosition, 800, {.forwards = false}, true);
    pros::delay(600);
    intake.score(110);
    pros::delay(scoreDelay);

    // Fourth match loader
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(44.5, -scraperDist-1, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    
    // Jiggle
    if (jiggle) { 
        for (int i = 0; i < scraperDelay/iterTime; i++) {
            int sign = i % 2 ? -1 : 1; 
            move(sign * 30, 0, false, iterTime + sign * 30);
        }
    } else {
        move(30, 0, false, scraperDelay);
    }
    
    // Score fourth set of 6 balls
    chassis.moveToPoint(48, -goalPosition, 1200, {.forwards = false}, true);
    pros::delay(1000);
    intake.score(110);
    scraper_piston.set_value(true);
    pros::delay(scoreDelay);
    intake.stop();

    // chassis.setPose(48, -26, -180);
    // descore.set_value(true);

    chassis.moveToPoint(48, -50, 700, {}, true);
    chassis.turnToPoint(13, -64, 600, {}, true);
    chassis.moveToPoint(13, -64, 900, {}, true);
    pros::delay(750);
    scraper_piston.set_value(false);
    intake.store(127);
    pros::delay(150);
    
    move(122, 5, false, 400);
    scraper_piston.set_value(true);

    // ---------------- END --------------------
}

void right() {
    chassis.setPose(16, -47, 0);
    descore.set_value(true);

    int scraperDist = 69.5;
    int scraperTimeout = 700;
    float matchLoaderSpeed = 55;
    int scraperDelay = 600;
    int scoreDelay = 1500;
    int iterTime = 100;

    // Get first set of 3 balls
    chassis.turnToPoint(26, -18, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(26, -18, 800, {}, true);
    pros::delay(550);
    scraper_piston.toggle();

    // Go to matchloader
    chassis.turnToPoint(46.5, -48, 1000, {}, false);
    chassis.moveToPoint(46.5, -48, 1000, {}, false);
    chassis.turnToPoint(46.5, -scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    // Jiggle
    for (int i = 0; i < scraperDelay/iterTime; i++) {
        int sign = i % 2 ? -1 : 1; 
        move(sign * 30, 0, false, iterTime + sign * 30);
    }

    // Score in long goal
    chassis.moveToPoint(48, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    scraper_piston.toggle();
    pros::delay(scoreDelay);
    intake.stop();

    // Push with deosicourwing
    chassis.turnToHeading(0, 1500, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed=60}, false);
    chassis.moveToPoint(60, -30, 600, {.forwards=false}, false);
    descore.set_value(false);
    chassis.moveToPoint(60, -8, 1000, {}, false);
}

void left() {
    chassis.setPose(-13, -47, -90);
    descore.set_value(true);

    int scraperDist = 70;
    int scraperTimeout = 700;
    float matchLoaderSpeed = 40;
    int scraperDelay = 600;
    int scoreDelay = 1100;
    int iterTime = 100;

    // First match loader
    chassis.moveToPoint(-46.5, -48, 1000, {}, false);
    scraper_piston.toggle();
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    // Jiggle
    for (int i = 0; i < scraperDelay/iterTime; i++) {
        int sign = i % 2 ? -1 : 1; 
        move(sign * 30, 0, false, iterTime + sign * 30);
    }

    // Score in long goal
    scraper_piston.toggle();
    chassis.moveToPoint(-48, -10, 1200, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(650);
    intake.score(115);
    move(-10, 0, false, scoreDelay);
    intake.stop();

    // Intake center 3 balls
    chassis.turnToPoint(-20, -24, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, true);
    intake.store(127);
    chassis.moveToPoint(-20, -24, 1500, {.maxSpeed=70}, true);
    pros::delay(400);
    scraper_piston.toggle();
    chassis.turnToPoint(-4, -2, 800, {.forwards = false}, true);

    pros::delay(100);
    lemlib::toggleMCL();
    intake.stop();
    top_intake.move(-100);
    bottom_intake.move(-20);
    chassis.moveToPoint(-4, -2, 1000, {.forwards = false}, false);
    intake.score(95, true);
    move(-20,0,false, 1500);
    intake.stop();
    scraper_piston.toggle();

    // Push with deosicourwing
    chassis.moveToPoint(-38, -26, 1200, {}, false);
    // lemlib::toggleMCL();
    descore.set_value(false);
    chassis.turnToHeading(-5, 1000, {.direction=AngularDirection::CW_CLOCKWISE}, true);
    chassis.moveToPoint(-40, -6, 1000, {}, true);
    chassis.turnToHeading(60, 1000, {}, false);
    
}

// ASSET(path1_txt);
void test() {
    // chassis.setPose(0, 0, 0);
    // lemlib::toggleMCL();
    // chassis.turnToHeading(90, 1000, {}, false);

    
    chassis.setPose(-48, 21, 0);
    descore.set_value(true);

    int scraperDist = 69.5;
    int scraperTimeout = 1000;
    float matchLoaderSpeed = 55;
    int scraperDelay = 1800;
    int scoreDelay = 2400;
    int goalPosition = 24;
    int iterTime = 100;
    bool jiggle = true;


    // Park ball clear
    scraper_piston.set_value(true);
    chassis.moveToPoint(-48, 53, 750, {}, true);
    chassis.turnToPoint(-13, 64, 600, {}, true);
    chassis.moveToPoint(-13, 64, 900, {}, true);
    pros::delay(750);
    intake.store(127);
    pros::delay(150);
    
    move(45, 5, false, 1600);  // 400
    move(70, 5, false, 250);
    scraper_piston.set_value(false);
    move(70, 5, false, 250);

    chassis.turnToHeading(0, 800, {}, false);
    resetPositionFromTwoDistanceSensors(0, 2);

    chassis.turnToPoint(22, 26, 500, {.forwards=false}, false);
    chassis.moveToPoint(22, 26, 1000, {.forwards=false}, false);
    chassis.turnToPoint(-3, 0, 500, {.forwards=false}, true);
    pros::delay(300);
    lemlib::toggleMCL();
    intake.stop();
    top_intake.move(-127);
    bottom_intake.move(-40);
    chassis.moveToPoint(-3, 0, 1000, {.forwards=false}, false);
    intake.score(65, true);
    chassis.turnToHeading(50, 800, {}, true);
    move(30, 0 ,false, 200);
    move(-20,0,false, 2000);

    // Move across field
    chassis.turnToPoint(46.5, 36, 800, {}, true);
    chassis.moveToPoint(46.5, 36, 1400,{}, true);
    pros::delay(500);
    intake.stop();
    lemlib::toggleMCL();

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, true);
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed}, false);
    
    // Jiggle
    if (jiggle) { 
        for (int i = 0; i < scraperDelay/iterTime; i++) {
            int sign = i % 2 ? -1 : 1; 
            move(sign * 30, 0, false, iterTime + sign * 30);
        }
    } else {
        move(30, 0, false, scraperDelay);
    }



    
    

    // chassis.moveToPointRamsete(-48, -48, -180, 5000, {.maxSpeed=110, .maxAccel=15, .zeta=2, .headingBlendStart=0.9, .poseFilterAlpha=0.3, .lateralDeadband=0.5, .voltageSlew=1000, .kS=15, .kP=10}, false);
    // chassis.moveToPointRamsete(-48, -48, -180, 5000, {}, false);
    // chassis.moveToPoint(-48, -48, 5000, {}, true);
    // chassis.turnToPoint(-48, -70, 5000, {}, true);
    // chassis.follow(path1_txt, 10, 100000, false);

}

void fiveInch() {
    chassis.setPose(0, 0, 0);
    lemlib::toggleMCL();
    chassis.moveToPoint(0, 8, 4000);
    chassis.waitUntilDone();
}


void skills2() {
    chassis.setPose(-5, -46, 0);
    descore.set_value(true);

    int scraperDist = 69.5;
    int scraperTimeout = 1000;
    float matchLoaderSpeed = 50;
    int scraperDelay = 1900;
    int scoreDelay = 2500;
    int goalPosition = 25;
    int iterTime = 100;


    // Get first 4 balls & score in middle
    intake.store(100);
    chassis.turnToPoint(-24, -24, 800, {}, true);
    chassis.moveToPoint(-24, -24, 1000,{}, true);
    pros::delay(525);
    scraper_piston.toggle();
    pros::delay(200);

    lemlib::toggleMCL();
    chassis.turnToPoint(-3, -6, 800, {.forwards = false}, false);
    intake.stop();
    top_intake.move(-100);
    bottom_intake.move(-20);
    chassis.moveToPoint(-3, -6, 1000, {.forwards = false}, false);
    intake.score(100, true);
    move(-20,0,false, 1500);
    // The above takes 6.6s

    // First match loader
    chassis.moveToPoint(-50, -50, 900, {}, false);
    intake.stop();
    intake.store(127);
    lemlib::toggleMCL();  
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, true);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    
    // move(30, 0, false, scraperDelay);
    // Jiggle
    for (int i = 0; i < scraperDelay/iterTime; i++) {
        int sign = i % 2 ? -1 : 1; 
        move(sign * 30, 0, false, iterTime + sign * 30);
    }
    

    // Move through tunnel
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false},true);
    chassis.turnToPoint(-61.5, -36, 650,{.forwards = false}, true);
    intake.stop();
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false},true);
    chassis.turnToPoint(-61.5, 36, 650,{.forwards = false},true);
    chassis.moveToPoint(-61.5, 36, 2000,{.forwards = false},true);
    chassis.turnToPoint(-48, 36,800, {},true);
    chassis.moveToPoint(-48, 36, 800, {}, true);

    // Score first 6 balls
    chassis.turnToPoint(-48, goalPosition, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, goalPosition, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Second match loader
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    // move(30, 0, false, scraperDelay);
    // Jiggle
    for (int i = 0; i < scraperDelay/iterTime; i++) {
        int sign = i % 2 ? -1 : 1; 
        move(sign * 30, 0, false, iterTime + sign * 30);
    }
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48, goalPosition, 1500, {.forwards = false}, false);
    intake.score(100);
    pros::delay(scoreDelay);
    intake.stop();

    // SHOULD BE AT 25s 

    // Move across field 
    chassis.turnToPoint(46.5, 36, 800,{.direction=lemlib::AngularDirection::CW_CLOCKWISE}, true);
    chassis.moveToPoint(46.5, 36, 2000,{}, true);

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, true);
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed}, false);
    // move(30, 0, false, scraperDelay);
    // Jiggle
    for (int i = 0; i < scraperDelay/iterTime; i++) {
        int sign = i % 2 ? -1 : 1; 
        move(sign * 30, 0, false, iterTime + sign * 30);
    }

    // Move through tunnel
    chassis.moveToPoint(46.5, 47, 800,{.forwards=false}, true);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false}, true);
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false}, true);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false}, true);
    intake.stop();
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false}, true);
    chassis.turnToPoint(48, -36, 800, {}, true);
    chassis.moveToPoint(48, -36, 800, {},  true);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -goalPosition, 800, {.forwards = false}, true);
    chassis.moveToPoint(48, -goalPosition, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);

    // Fourth match loader
    scraper_piston.set_value(false);
    intake.store(127);
    chassis.moveToPoint(44.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    // move(30, 0, false, scraperDelay);
    // Jiggle
    for (int i = 0; i < scraperDelay/iterTime; i++) {
        int sign = i % 2 ? -1 : 1; 
        move(sign * 30, 0, false, iterTime + sign * 30);
    }
    
    // Score fourth set of 6 balls
    chassis.moveToPoint(48, -goalPosition, 1500, {.forwards = false}, false);
    intake.score(100);
    scraper_piston.set_value(true);
    pros::delay(scoreDelay);
    intake.stop();

    // chassis.setPose(48, -26, -180);
    // descore.set_value(true);

    chassis.moveToPoint(48, -53, 700, {}, true);
    chassis.turnToPoint(13, -64, 600, {}, true);
    chassis.moveToPoint(13, -64, 900, {}, true);
    pros::delay(750);
    scraper_piston.set_value(false);
    intake.store(127);
    pros::delay(150);
    
    move(122, 5, false, 400);
    scraper_piston.set_value(true);

    // ---------------- END --------------------








    // Park ball intake
    // chassis.moveToPoint(48, -53, 700, {}, true);
    // chassis.turnToPoint(13, -64, 600, {}, true);
    // chassis.moveToPoint(13, -64, 900, {}, true);
    // pros::delay(810);
    // scraper_piston.set_value(true);
    // intake.store(127);
    // pros::delay(50);
    // left_motor_group.move(90);
    // right_motor_group.move(100);
    // // 105, 125
    // pros::delay(150);
    // scraper_piston.set_value(false);
    // pros::delay(200);
    // left_motor_group.move(105);
    // right_motor_group.move(125);
    // pros::delay(800);
    // // move(115, -10, false, 650);
    // scraper_piston.set_value(true);
    
    
    // pros::delay(200);
    // left_motor_group.brake();
    // right_motor_group.brake();









    // OLD MIDDLE
    // // chassis.setPose(-48,23,0);

    // // chassis.moveToPose(-17, 63.5, 82, 1500, {.lead = 0.3}, false);
    // // horLift.set_value(true);
    // // verLift.set_value(true);
    // // pros::delay(200);
    // // intake.store(127);
    // // lemlib::toggleMCL();

    // // // Going Over barrier 
    // // move(100,0,false,100);
    // // // scraper.set_value(true);
    // // move(127,6,false,225);
    // // scraper.set_value(false);
    // // move(127,7,false,775);
    // // scraper.set_value(true);
    // // move(90,8,false,250);
    // // scraper.set_value(false);
    // // // move(30, 0, false, 200);
    // // pros::delay(1000);

    // // horLift.set_value(false);
    // // verLift.set_value(false);
    // // chassis.setPose(30, 60, imu.get_heading());
    // // pros::delay(200);
    // // lemlib::toggleMCL();
    // // chassis.turnToPoint(21, 25, 800,{},false);
    // // scraper.set_value(true);
    
}


