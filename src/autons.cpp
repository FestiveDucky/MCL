#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "intake.h"



void soloAWP() {
    chassis.setPose(-13, -47, -90);
    // descore.set_value(true);

    int scraperDist = 66;
    int scraperTimeout = 800;
    float matchLoaderSpeed = 60;
    int scraperDelay = 0;
    int scoreDelay = 1000;

    // First match loader
    chassis.moveToPoint(-46.5, -48, 1000, {}, false);
    // scraper.toggle();
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);

    // Score in long goal
    // scraper.toggle();
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
    chassis.moveToPoint(-4, -4, 1000, {.forwards=false}, false);
    intake.score(127, true);
    pros::delay(scoreDelay);
    intake.stop();

    // Go to second set of 3 balls
    chassis.moveToPoint(-26, -26, 1000, {}, false);
    lemlib::toggleMCL();
    chassis.turnToPoint(24, -24, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(26, -24, 2000, {.headingCorrection=5}, false);

    // Go to second long goal
    chassis.turnToPoint(48, -48, 800, {}, false);
    chassis.moveToPoint(48, -48, 1500, {}, false);
    intake.stop();
    chassis.turnToPoint(48, -24, 800, {.forwards=false}, false);
    chassis.moveToPoint(48, -24, 1000, {.forwards=false, .maxSpeed=70}, false);
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
    // chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    chassis.turnToHeading(-180, 800);
    // scraper.toggle();
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
    // scraper.toggle();
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
    // scraper.toggle();
    bottom_intake.move(-60);
    top_intake.move(-127);
    pros::delay(800);
    intake.score(127, true);
    pros::delay(scoreDelay + 1000);
    intake.stop();
    // scraper.toggle();

    // Move to park zone and Park
    chassis.moveToPoint(-48, -48, 3000, {.maxSpeed=40}, false);
    pros::delay(500);
    // chassis.setPose(-48, -48, 90);
    chassis.turnToPoint(-17, -62, 800, {}, false);
    chassis.moveToPoint(-20, -62, 1300, {.maxSpeed=40}, false);
    chassis.turnToHeading(100, 800, {}, false);
    // chassis.moveToPose(-20, -60, 100, 1250, {.minSpeed=40, .earlyExitRange=5}, false);
    // scraper.toggle();
    bottom_intake.move(-127);
    pros::delay(500);
    
    chassis.tank(127, 127, true);
    pros::delay(550);
    chassis.tank(0, 0, true);
    // scraper.toggle();
    pros::delay(10000);
}

void right() {
    chassis.setPose(16, -47, 0);
    descore.set_value(true);

    int scraperDist = 66;
    int scraperTimeout = 800;
    float matchLoaderSpeed = 60;
    int scraperDelay = 0;
    int scoreDelay = 1000;

    // Get first set of 3 balls
    chassis.turnToPoint(26, -18, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(26, -18, 800, {}, false);

    //POSSIBLY DROP SCRAPER WITH DISABLING ASYNC AND JUST USING A PROS::DELAY TO TIME IT

    // Go to matchloader
    chassis.turnToPoint(46.5, -48, 1000, {}, false);
    intake.stop();
    chassis.moveToPoint(46.5, -48, 1000, {}, false);
    // scraper.toggle();
    chassis.turnToPoint(46.5, -scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    pros::delay(scraperDelay);
    intake.stop();

    // Score in long goal
    // scraper.toggle();
    chassis.moveToPoint(48, -24, 1200, {.forwards=false, .maxSpeed=70}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Push with deosicourwing
    chassis.swingToPoint(39, -5, lemlib::DriveSide::RIGHT, 1000, {}, false);
    descore.set_value(false);
    chassis.moveToPoint(39, -5, 1000, {}, false);
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
