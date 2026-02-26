#include "lemlib/chassis/chassis.hpp"
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
    wing.set_value(true);

    int scraperDist = 66;
    int scraperTimeout = 1200;
    float matchLoaderSpeed = 50;
    int scraperDelay = 1200;
    int scoreDelay = 2200;

    // First match loader
    chassis.moveToPoint(-46.5, -48, 2000, {}, false);
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    scraper.set_value(true);
    pros::delay(800);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, 300);
    pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false, .maxSpeed=50},false);
    chassis.turnToPoint(-61.5, -36, 600,{.forwards = false},false);
    scraper.set_value(false);
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false, .maxSpeed = 50},false);
    chassis.turnToPoint(-61.5, 36, 800,{.forwards = false},false);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(-61.5, 36, 2000,{.forwards = false, .maxSpeed = 70},false);
    chassis.turnToPoint(-48, 36,800, {},false);
    chassis.moveToPoint(-48, 36, 800, {.maxSpeed = 70}, false);

    // Score first 6 balls
    chassis.turnToPoint(-48, 24, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, 24, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Second match loader
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, 500);
    pros::delay(scraperDelay);
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48, 23, 1500, {.forwards = false,.maxSpeed = 40}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();
    scraper.set_value(false);
    
    // Move across field 
    chassis.moveToPoint(-48, 36, 900, {}, false);
    chassis.turnToPoint(46.5, 36, 800,{},false);
    chassis.moveToPoint(46.5, 36, 2500,{},false);

    // Third match loader
    chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, 300);
    pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(46.5, 47, 800,{.forwards=false, .maxSpeed=50}, false);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false},false);
    scraper.set_value(false);
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false, .maxSpeed = 50},false);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false},false);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false, .maxSpeed = 70},false);
    chassis.turnToPoint(48, -36, 800, {},false);
    chassis.moveToPoint(48, -36, 800, {.maxSpeed = 70}, false);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -24, 800, {.forwards = false}, false);
    chassis.moveToPoint(48, -24, 800, {.forwards = false}, false);
    intake.score(127);
    pros::delay(scoreDelay);

    // Fourth match loader
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(44.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, 300);
    pros::delay(scraperDelay);
    
    // Score fourth set of 6 balls
    chassis.moveToPoint(48, -24, 1500, {.forwards = false,.maxSpeed = 40}, false);
    intake.score(127);
    scraper.set_value(false);
    pros::delay(scoreDelay);
    intake.stop();

    // Get first set of 4 center balls
    // chassis.moveToPoint(48, -48, 1000, {}, false);
    // chassis.turnToPoint(18, -18, 800, {}, false);
    // scraper.set_value(true);
    // intake.store(127);
    // chassis.moveToPoint(18, -18, 1500, {.maxSpeed=55}, false);

    // // Get Second set of 4 center balls
    // chassis.turnToPoint(-35, -24, 800, {}, false);
    // chassis.moveToPoint(-35, -24, 2000, {.maxSpeed=55}, false);
    // pros::delay(700);
    // intake.stop();
    // scraper.set_value(false);

    // Turn to middle and score
    // chassis.moveToPoint(-24, -24, 1000, {.forwards = false}, false);
    // chassis.turnToPoint(-5, -5, 800, {.forwards = false}, false);
    // chassis.moveToPoint(-5, -5, 1500, {.forwards = false, .maxSpeed=40, .headingCorrection=true}, false);
    
    // pros::delay(1000);
    // intake.score(127, true);
    // pros::delay(scoreDelay + 1000);
    // intake.stop();

    // Move to park zone and Park
    chassis.moveToPose(24, -62, -90, 3000, {.lead = .3}, false);
    pros::delay(500);

    horLift.set_value(true);
    verLift.set_value(true);

    move(120,0,false,100);
    scraper.set_value(true);
    move(120,6,false,225);
    scraper.set_value(false);
    move(127,6,false,300);
}

void skills2(){
    wing.set_value(true);
    intake.store(127);

    chassis.setPose(-5, -46, 0);

    int scraperDist = 65;
    int scraperTimeout = 1200;
    float matchLoaderSpeed = 50;
    int scraperDelay = 1500;
    int scoreDelay = 2500;
      // Get first 4 balls & score in middle
    // chassis.moveToPose(-22, -22, -45, 1500, {.lead = .3}, false);
    chassis.turnToPoint(-21,-25,800,{},false);
    chassis.moveToPoint(-21,-25,1000,{},false);
    lemlib::toggleMCL();

    // chassis.turnToPoint(-9.37,-7.63, 800, {.forwards = false}, false);
    // chassis.moveToPoint(-9.37,-7.63, 1000, {.forwards = false}, false);
    chassis.turnToPoint(-8,-8, 800, {.forwards = false}, false);
    chassis.moveToPoint(-8,-8, 1000, {.forwards = false}, false);
    // scoreMiddle();
    intake.score(100, true);

    move(-20,0,false, 450);

// //     // First match loader
    chassis.turnToPoint(-46.5, -48, 500, {}, false);
    intake.store(127);
    chassis.moveToPoint(-46.5, -48, 1000, {}, false);
    chassis.turnToHeading(-180,800,{}, false);
    lemlib::toggleMCL();
    scraper.set_value(true);
    pros::delay(800);
    intake.store(127);
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false},false);
    intake.stop();
    chassis.turnToPoint(-61.5, -36, 650,{.forwards = false},false);
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false, .headingCorrection = 1000},false);
    chassis.turnToPoint(-61.5, 36, 650,{.forwards = false},false);
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(-61.5, 36, 1700,{.forwards = false},false);
    // chassis.turnToPoint(-48, 36,800, {},false);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(-48, 36, 800, {.headingCorrection = 1000}, false);

    // Score first 6 balls
    chassis.turnToPoint(-48, 24, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, 24, 650, {.forwards = false});
    intake.outtake(10);
    pros::delay(300);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();

    // Second match loader
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(-46.5, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);
    
    // Score second set of 6 balls
    chassis.moveToPoint(-48.5, 23, 800, {.forwards = false, .maxSpeed = 65}, false);
    // intake.outtake(10);
    // pros::delay(550);
    intake.score(127);
    chassis.turnToHeading(0, 500);
    pros::delay(scoreDelay);
    // intake.stop();
    scraper.set_value(false);
    // move(40, 0, false, 400);
    // move(-45, 0, false, 500);




    // chassis.setPose(-48,23,0);

    chassis.moveToPose(-15.5, 64.5, 85, 1700, {.lead = 0.35}, false);
    horLift.set_value(true);
    verLift.set_value(true);
    pros::delay(200);
    lemlib::toggleMCL();
    intake.store(127);
    // lemlib::toggleMCL();

    // // // Going Over barrier 
    move(75,-7,false,2000);
    scraper.set_value(true);
    // scraper.set_value(true);
    // move(90,-6,false,225);
    // // scraper.set_value(false);
    // move(90,-7,false,775);
    // // scraper.set_value(true);
    // move(90,-8,false,250);
    // scraper.set_value(false);
    // move(30, 0, false, 200);
    pros::delay(50);

    horLift.set_value(false);
    verLift.set_value(false);
    chassis.setPose(30, 62, imu.get_heading());
    pros::delay(400);
    scraper.set_value(false);


    // chassis.turnToPoint(22, 22, 900);
    lemlib::toggleMCL();
    chassis.turnToHeading(20, 800);
    

    // lemlib::toggleMCL();
    chassis.turnToPoint(21, 24, 500,{.forwards = false});
    chassis.moveToPoint(21, 24, 800,{.forwards = false}, false);
    // chassis.turnToHeading(45,700,{},false);
    // chassis.turnToPoint(5, 10.8, 700, {.forwards = false}, false);
    // lemlib::toggleMCL();
    // chassis.moveToPoint(5, 10.8, 1000, {.forwards = false});
    chassis.turnToPoint(8.5, 8.5, 700, {.forwards = false}, false);
    lemlib::toggleMCL();
    chassis.moveToPoint(8.5, 8.5, 1000, {.forwards = false});



    chassis.turnToPoint(46.5, 48, 800, {}, false);
    intake.score(127, true);

    move(-10,0,false, 600);

    pros::delay(400);    
    intake.score(60, true);
    pros::delay(1000);


    



    // // chassis.turnToPoint(21, 25, 800,{},false);
    // // scraper.set_value(true);

    // Move across field 
    // chassis.moveToPoint(-48, 36, 900, {}, false);
    // chassis.turnToPoint(46.5, 36, 800,{},false);
    // intake.score(127);
    // chassis.moveToPoint(46.5, 36, 2000,{},false);

    lemlib::toggleMCL();
    intake.store(127);
    scraper.set_value(true);
    chassis.moveToPoint(46.5, 48, 1700, {}, false);
    chassis.turnToHeading(0,800,{}, false);
    // lemlib::toggleMCL();

    // // Third match loader
    // chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    intake.store(127);
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);

    // Move through tunnel
    chassis.moveToPoint(46.5, 47, 800,{.forwards=false}, false);
    intake.stop();
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false},false);
    // scraper.set_value(false);
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false},false);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false},false);
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false},false);
    chassis.turnToHeading(-90, 800);
    // chassis.turnToPoint(48, -36, 800, {},false);
    chassis.moveToPoint(48, -36, 800, {.headingCorrection = 1000}, false);

    // Score third set of 6 balls
    chassis.turnToPoint(48, -24, 800, {.forwards = false}, false);
    chassis.moveToPoint(48, -24, 800, {.forwards = false});
    intake.outtake(10);
    pros::delay(150);
    intake.score(127);
    pros::delay(scoreDelay);

    // Fourth match loader
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(44.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    // pros::delay(scraperDelay);
    
    // Score fourth set of 6 balls/
    chassis.moveToPoint(49, -24, 800, {.forwards = false, .maxSpeed = 65}, false);
    intake.score(127);
    pros::delay(scoreDelay);
    intake.stop();
    scraper.set_value(false);



    chassis.moveToPose(17, -64, -82, 1500, {.lead = 0.3}, false);
    horLift.set_value(true);
    verLift.set_value(true);
    pros::delay(20);
    intake.store(127);


    move(100,0,false,100);
    scraper.set_value(true);
    move(127,6,false,800);
    scraper.set_value(false);
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
    // intake.stop();

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
