#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "intake.h"

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
    pros::delay(800);
    bottom_intake.move(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false, .maxSpeed=50},false);
    chassis.turnToPoint(-61.5, -36, 600,{.forwards = false},false);
    bottom_intake.move(0);
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false, .maxSpeed = 50},false);
    chassis.turnToPoint(-61.5, 36, 800,{.forwards = false},false);
    chassis.moveToPoint(-61.5, 36, 2000,{.forwards = false, .maxSpeed = 70},false);
    chassis.turnToPoint(-48, 36,800, {},false);
    chassis.moveToPoint(-48, 36, 800, {.maxSpeed = 70}, false);

    // Score first 6 balls
    chassis.turnToPoint(-48, 24, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, 24, 800, {.forwards = false}, false);
	top_score.set_value(false);
    top_intake.move(127);
    bottom_intake.move(127);
    pros::delay(scoreDelay);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);

    // Second match loader
    bottom_intake.move(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    pros::delay(scraperDelay);
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false,.maxSpeed = 40}, false);
    top_score.set_value(false);
    top_intake.move(127);
    pros::delay(scoreDelay);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);
    
    // Move across field 
    chassis.moveToPoint(-48, 36, 900, {}, false);
    chassis.turnToPoint(46.5, 36, 800,{},false);
    chassis.moveToPoint(46.5, 36, 2500,{},false);

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    bottom_intake.move(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(46.5, 47, 800,{.forwards=false, .maxSpeed=50}, false);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false},false);
    bottom_intake.move(0);
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false, .maxSpeed = 50},false);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false},false);
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false, .maxSpeed = 70},false);
    chassis.turnToPoint(48, -36, 800, {},false);
    chassis.moveToPoint(48, -36, 800, {.maxSpeed = 70}, false);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -24, 800, {.forwards = false}, false);
    chassis.moveToPoint(48, -24, 800, {.forwards = false}, false);
	top_score.set_value(false);
    top_intake.move(127);
    bottom_intake.move(127);
    pros::delay(scoreDelay);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);

    // Fourth match loader
    bottom_intake.move(127);
    chassis.moveToPoint(46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);

    pros::delay(scraperDelay);
    
    // Score fourth set of 6 balls
    chassis.moveToPoint(48, -24, 1500, {.forwards = false,.maxSpeed = 40}, false);
    top_score.set_value(false);
    top_intake.move(127);
    pros::delay(scoreDelay);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);

    // Get first set of 4 center balls
    chassis.moveToPoint(48, -48, 1000, {}, false);
    chassis.turnToPoint(18, -18, 800, {}, false);
    scraper_piston.toggle();
    bottom_intake.move(127);
    chassis.moveToPoint(18, -18, 1500, {.maxSpeed=55}, false);

    // Get Second set of 4 center balls
    chassis.turnToPoint(-35, -24, 800, {}, false);
    chassis.moveToPoint(-35, -24, 2000, {.maxSpeed=55}, false);
    pros::delay(700);
    bottom_intake.move(0);

    // Turn to middle and score
    chassis.moveToPoint(-24, -24, 1000, {.forwards = false}, false);
    chassis.turnToPoint(-5, -5, 800, {.forwards = false}, false);
    chassis.moveToPoint(-5, -5, 1500, {.forwards = false, .maxSpeed=40, .headingCorrection=true}, false);
    scraper_piston.toggle();
    bottom_intake.move(-60);
    top_intake.move(-127);
    pros::delay(1000);
    middlescore_piston.set_value(true);
	bottom_intake.move(127);
	top_intake.move(127);
    pros::delay(scoreDelay + 1000);
    top_intake.move(0);
    bottom_intake.move(0);
    middlescore_piston.set_value(false);
    scraper_piston.toggle();

    // Move to park zone and Park
    chassis.moveToPoint(-48, -48, 3000, {.maxSpeed=40}, false);
    pros::delay(500);
    // chassis.setPose(-48, -48, 90);
    chassis.turnToPoint(-17, -60, 800, {}, false);
    chassis.moveToPoint(-20, -60, 1300, {.maxSpeed=40}, false);
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

}

void left() {

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