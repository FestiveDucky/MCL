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
    right_motor_group.move(left);
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
    float matchLoaderSpeed = 60;
    int scraperDelay = 0;
    int scoreDelay = 800;

    // First match loader
    chassis.moveToPoint(46.5, -48, 1000, {}, false);
    scraper_piston.toggle();
    chassis.turnToPoint(46.5, -scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Score in long goal
    scraper_piston.toggle();
    chassis.moveToPoint(48, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Intake center 3 balls
    chassis.moveToPoint(48, -48, 1000, {}, false);
    chassis.turnToPoint(18, -18, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(18, -24, 800, {}, true);
    pros::delay(600);
    scraper_piston.toggle();
    
    // Move to second set of 3 balls
    chassis.turnToPoint(-24, -24, 400, {}, false);
    scraper_piston.toggle();
    chassis.moveToPoint(-24, -24, 1500, {}, true);
    pros::delay(950);
    scraper_piston.toggle();
    intake.stop();

    // Score middle
    lemlib::toggleMCL();
    chassis.turnToPoint(-6, -6, 600, {.forwards=false}, false);
    top_intake.move(-100);
    bottom_intake.move(-50);
    chassis.moveToPoint(-6, -6, 1000, {.forwards=false}, false);
    intake.score(100, true);
    pros::delay(scoreDelay);
    intake.stop();

    chassis.moveToPoint(-48, -48, 1200, {}, false);
    intake.store(127);
    lemlib::toggleMCL();
    chassis.turnToPoint(-48, -24, 800, {.forwards=false}, false);
    chassis.moveToPoint(-48, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    pros::delay(5000);

}

void skills() {
    chassis.setPose(-13, -47, -90);
    descore.set_value(true);

    int scraperDist = 66;
    int scraperTimeout = 1200;
    float matchLoaderSpeed = 60;
    int scraperDelay = 1200;
    int scoreDelay = 2200;

    // First match loader
    chassis.moveToPoint(-46.5, -48, 2000, {}, false);
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    scraper_piston.toggle();
    pros::delay(500);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false},false);
    chassis.turnToPoint(-61.5, -36, 600,{.forwards = false},false);
    intake.stop();
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false},false);
    chassis.turnToPoint(-61.5, 36, 800,{.forwards = false},false);
    chassis.moveToPoint(-61.5, 36, 2000,{.forwards = false, .maxSpeed=90},false);
    chassis.turnToPoint(-48, 36,800, {},false);
    chassis.moveToPoint(-48, 36, 800, {}, false);

    // Score first 6 balls
    chassis.turnToPoint(-48, 24, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, 24, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Second match loader
    intake.store(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    pros::delay(scraperDelay);
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false,.maxSpeed = 40}, false);
    intake.stop();
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();
    
    // Move across field 
    chassis.moveToPoint(-48, 36, 900, {}, false);
    chassis.turnToPoint(46.5, 36, 800,{},false);
    chassis.moveToPoint(46.5, 36, 2500,{},false);

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(46.5, 47, 800,{.forwards=false}, false);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false}, false);
    intake.stop();
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false},false);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false},false);
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false},false);
    chassis.turnToPoint(48, -36, 800, {},false);
    chassis.moveToPoint(48, -36, 800, {}, false);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -24, 800, {.forwards = false}, false);
    chassis.moveToPoint(48, -24, 800, {.forwards = false}, false);
	intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Fourth match loader
    intake.store(127);
    chassis.moveToPoint(46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);

    pros::delay(scraperDelay);
    
    // Score fourth set of 6 balls
    chassis.moveToPoint(48, -24, 1500, {.forwards = false,.maxSpeed = 40}, false);
    intake.stop();
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Get first set of 4 center balls
    chassis.moveToPoint(48, -48, 1000, {}, false);
    chassis.turnToPoint(18, -18, 800, {}, false);
    scraper_piston.toggle();
    intake.store(127);
    chassis.moveToPoint(18, -18, 1500, {.maxSpeed=55}, false);

    // Get Second set of 4 center balls
    chassis.turnToPoint(-35, -24, 800, {}, false);
    chassis.moveToPoint(-35, -24, 2000, {.maxSpeed=55}, false);
    pros::delay(700);
    intake.stop();

    // Turn to middle and score
    chassis.moveToPoint(-24, -24, 1000, {.forwards = false}, false);
    chassis.turnToPoint(-5, -5, 800, {.forwards = false}, false);
    chassis.moveToPoint(-5, -5, 1500, {.forwards = false, .maxSpeed=40, .headingCorrection=0}, false);
    scraper_piston.toggle();
    bottom_intake.move(-60);
    top_intake.move(-127);
    pros::delay(800);
    intake.score(127, true);
    pros::delay(scoreDelay + 1000);
    intake.stop();
    scraper_piston.toggle();

    // Move to park zone and Park
    chassis.moveToPoint(-48, -48, 3000, {.maxSpeed=40}, false);
    pros::delay(500);
    // chassis.setPose(-48, -48, 90);
    chassis.turnToPoint(-17, -62, 800, {}, false);
    chassis.moveToPoint(-20, -62, 1300, {.maxSpeed=40}, false);
    chassis.turnToHeading(100, 800, {}, false);
    // chassis.moveToPose(-20, -60, 100, 1250, {.minSpeed=40, .earlyExitRange=5}, false);
    scraper_piston.toggle();
    bottom_intake.move(-127);
    pros::delay(500);
    
    chassis.tank(127, 127, true);
    pros::delay(550);
    chassis.tank(0, 0, true);
    scraper_piston.toggle();
    pros::delay(10000);
}

void right() {
    chassis.setPose(16, -47, 0);
    descore.set_value(true);

    int scraperDist = 70;
    int scraperTimeout = 1400;
    float matchLoaderSpeed = 60;
    int scraperDelay = 0;
    int scoreDelay = 1500;

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
    pros::delay(scraperDelay);

    // Score in long goal
    chassis.moveToPoint(48, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    scraper_piston.toggle();
    pros::delay(scoreDelay);
    intake.stop();

    // Push with deosicourwing
    chassis.turnToHeading(0, 1500, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    chassis.moveToPoint(60, -30, 600, {.forwards=false}, false);
    descore.set_value(false);
    chassis.moveToPoint(60, -8, 1000, {}, false);
}

void left() {
    chassis.setPose(-13, -47, -90);
    descore.set_value(true);

    int scraperDist = 70;
    int scraperTimeout = 900;
    float matchLoaderSpeed = 60;
    int scraperDelay = 0;
    int scoreDelay = 1000;

    // First match loader
    chassis.moveToPoint(-46.5, -48, 1000, {}, false);
    scraper_piston.toggle();
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Score in long goal
    scraper_piston.toggle();
    chassis.moveToPoint(-48, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Intake center 3 balls
    chassis.moveToPoint(-46, -46, 1000, {}, false);
    chassis.turnToPoint(-18, -18, 800, {}, false);
    intake.store(100);
    chassis.moveToPoint(-18, -18, 1500, {.headingCorrection=5}, false);

    lemlib::toggleMCL();
    // Score middle
    chassis.turnToPoint(-4, -4, 800, {.forwards=false}, false);
    intake.stop();
    top_intake.move(-100);
    chassis.moveToPoint(-4, -4, 1000, {.forwards=false}, false);
    intake.score(100, true);
    pros::delay(scoreDelay);
    intake.stop();

    // Push with deosicourwing
    chassis.moveToPoint(-37, -37, 1200, {}, false);
    lemlib::toggleMCL();
    descore.set_value(false);
    chassis.turnToHeading(0, 1000, {.direction=AngularDirection::CW_CLOCKWISE}, false);
    chassis.moveToPoint(-37, -8, 1000, {}, false);
    
}

void test() {
    chassis.setPose(24, -48, 0);
    intake.score(127);
    pros::delay(100000);
}

void fiveInch() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 4000);
    chassis.waitUntilDone();
}


void skills2() {
    chassis.setPose(-5, -46, 0);
    descore.set_value(true);

    int scraperDist = 65;
    int scraperTimeout = 1200;
    float matchLoaderSpeed = 50;
    int scraperDelay = 1500;
    int scoreDelay = 2500;
      // Get first 4 balls & score in middle
    // chassis.moveToPose(-22, -22, -45, 1500, {.lead = .3}, false);
    intake.store(127);
    chassis.turnToPoint(-21,-25,800,{},false);
    chassis.moveToPoint(-21,-25,1000,{},false);
    lemlib::toggleMCL();

    intake.stop();
    chassis.turnToPoint(-9.37,-7.63, 800, {.forwards = false}, false);
    top_intake.move(-60);
    chassis.moveToPoint(-9.37,-7.63, 1000, {.forwards = false}, false);
    // scoreMiddle();
    intake.score(90, true);

    move(-20,0,false, 600);
    pros::delay(300);

// //     // First match loader
    chassis.turnToPoint(-48, -48, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(-48, -48, 1000, {}, false);
    chassis.turnToHeading(-180,800,{}, false);
    lemlib::toggleMCL();
    scraper_piston.set_value(true);
    pros::delay(800);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false, .maxSpeed=90},false);
    chassis.turnToPoint(-61.5, -36, 650,{.forwards = false},false);
    intake.stop();
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false, .maxSpeed = 90},false);
    chassis.turnToPoint(-61.5, 36, 650,{.forwards = false},false);
    chassis.moveToPoint(-61.5, 36, 2000,{.forwards = false, .maxSpeed = 90},false);
    chassis.turnToPoint(-48, 36,800, {},false);
    chassis.moveToPoint(-48, 36, 800, {.maxSpeed = 70}, false);

    // Score first 6 balls
    chassis.turnToPoint(-48, 24, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, 24, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Second match loader
    scraper_piston.set_value(true);
    intake.store(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48, 23, 1500, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();
    scraper_piston.set_value(false);
    move(40, 0, false, 400);
    move(-45, 0, false, 500);

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

    // Move across field 
    chassis.moveToPoint(-48, 36, 900, {}, false);
    chassis.turnToPoint(46.5, 36, 800,{},false);
    intake.score(127);
    chassis.moveToPoint(46.5, 36, 2000,{},false);

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    scraper_piston.set_value(true);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(46.5, 47, 800,{.forwards=false}, false);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false},false);
    // scraper.set_value(false);
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false, .maxSpeed = 90},false);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false},false);
    intake.stop();
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false},false);
    chassis.turnToPoint(48, -36, 800, {},false);
    chassis.moveToPoint(48, -36, 800, {.maxSpeed = 70}, false);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -24, 800, {.forwards = false}, false);
    chassis.moveToPoint(48, -24, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);

    // Fourth match loader
    scraper_piston.set_value(true);
    intake.store(127);
    chassis.moveToPoint(44.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);
    
    // Score fourth set of 6 balls/
    chassis.moveToPoint(48, -24, 1500, {.forwards = false}, false);
    intake.score(127);
    scraper_piston.set_value(false);
    pros::delay(scoreDelay);
    intake.stop();
    move(40, 0, false, 450);
    move(-45, 0, false, 450);

    chassis.moveToPose(17, -64, -82, 1500, {.lead = 0.3}, false);
    pros::delay(200);
    intake.store(127);

    pros::delay(200);
    move(100,0,false,100);
    scraper_piston.set_value(true);
    move(127,6,false,700);
    scraper_piston.set_value(false);
}
