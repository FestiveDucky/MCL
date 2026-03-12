#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "intake.h"

// Distance sensor position reset: call only when perpendicular to a wall.
// resetPositionFront(); resetPositionBack(); resetPositionLeft(); resetPositionRight();

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
    // resetPositionRight();  // optional: reset X when perpendicular to left wall

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

void skills96(){
    wing.set_value(true);
    intake.store(127);

    chassis.setPose(-5, -46, 0);

    int scraperDist = 65;
    int scraperTimeout = 1200;
    float matchLoaderSpeed = 50;
    int scraperDelay = 1500;
    int scoreDelay = 2500;

    // Get first 4 balls
    chassis.turnToPoint(-21,-25,650,{},false);
    chassis.moveToPoint(-21,-25,1000,{},false);

    // MCL OFF
    lemlib::toggleMCL();

    //Score Middle
    chassis.turnToPoint(-8,-8, 650, {.forwards = false}, false);
    chassis.moveToPoint(-8,-8, 1000, {.forwards = false}, false);
    intake.score(100, true);
    move(-20,0,false, 450);

    // Move to First match loader
    chassis.turnToPoint(-46.5, -48, 500, {}, false);
    // intake.store(127);
    chassis.moveToPoint(-46.5, -48, 1000, {}, false);
    scraper.set_value(true);
    chassis.turnToHeading(-180,800,{}, false);

    // MCL ON 
    lemlib::toggleMCL();
    intake.store(127);
    
    // Scrape First Loader
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);

    // Move through tunnel
    // chassis.moveToPoint(-46.5, -47, 800, {.forwards = false});
    // intake.stop();
    // chassis.turnToPoint(-61.5, -36, 650,{.forwards = false});

    chassis.swingToPoint(-61.5, -36, lemlib::DriveSide::RIGHT, 650,{.forwards = false}, false);
    intake.stop();
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false, .headingCorrection = 1000});
    chassis.turnToPoint(-61.5, 36, 650,{.forwards = false});
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(-61.5, 36, 1700,{.forwards = false});
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(-48, 36, 800, {.headingCorrection = 1000});

    // Score first 6 balls
    chassis.turnToPoint(-48, 23, 800, {.forwards = false},false);
    chassis.moveToPoint(-48, 23, 650, {.forwards = false});
    // intake.outtake(10);
    pros::delay(300);
    intake.score(127);
    move(-15,0,false, scoreDelay);
    intake.stop();

    // Second match loader
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(-46.0, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);

    
    // Score second set of 6 balls
    chassis.moveToPoint(-48.5, 23, 1000, {.forwards = false, .maxSpeed = 65}, false);
    // pros::delay(300);
    intake.score(127);
    move(-15,0,false, scoreDelay);



    // chassis.moveToPoint(-48.5, 23, 1200, {.forwards = false, .maxSpeed = 65}, false);
    // intake.score(127);
    scraper.set_value(false);
    move(40, 0, false, 400);
    move(-35, 0, false, 600);





    chassis.moveToPose(-15.5, 64.5, 85, 1700, {.lead = 0.35}, false);
    horLift.set_value(true);
    verLift.set_value(true);
    lemlib::toggleMCL();
    intake.store(127);

    // Going Over barrier 
    move(82,-9,false,1800);
    scraper.set_value(true);
    horLift.set_value(false);
    verLift.set_value(false);


    // Distance Reset 
    chassis.turnToHeading(180, 800,{}, false);
    // pros::delay(200);
    resetPositionBack();
    resetPositionLeft();
    

    lemlib::toggleMCL();
    chassis.turnToPoint(25, 25, 400);
    chassis.moveToPoint(25, 25, 1000,{}, false);
    lemlib::toggleMCL();


    chassis.turnToPoint(8, 3, 700, {.forwards = false}, false);
    chassis.moveToPoint(8, 3, 1000, {.forwards = false}, false);


    move(-30,0,false, 300);
    intake.score(127, true);

    move(-25,0,false, 500);

    intake.score(50, true);
    pros::delay(1000);
    intake.store(127);


    




    // originally MCL TOGGLE WAS HERE
    // lemlib::toggleMCL();
 

    chassis.turnToPoint(46.5, 48, 300, {}, false);
    chassis.moveToPoint(46.5, 48, 1100, {}, false);
    intake.score(127);

    lemlib::toggleMCL();
    // chassis.turnToHeading(0,800,{}, false);
    chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    intake.store(127);

    // // Third match loader
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);

    // Move through tunnel

    // I am CHANGING THIS CAUSE ITS INEFFICIENT
    // chassis.moveToPoint(46.5, 47, 800,{.forwards=false}, false);
    // intake.stop();
    // chassis.turnToPoint(61.5, 36, 600,{.forwards = false});
    chassis.swingToPoint(61.5, 36, lemlib::DriveSide::RIGHT, 600,{.forwards = false}, false);
    

    intake.stop();
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false});
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.turnToPoint(61.5, -36, 600,{.forwards = false});
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false});
    chassis.turnToHeading(-90, 650);
    chassis.moveToPoint(48, -36, 800, {.headingCorrection = 1000});

    // Score third set of 6 balls
    chassis.turnToPoint(48, -23, 650, {.forwards = false}, false);




    chassis.moveToPoint(48, -23, 650, {.forwards = false});
    // intake.outtake(10);
    pros::delay(300);
    intake.score(127);
    move(-25,0,false, scoreDelay);


    // Fourth match loader
    scraper.set_value(true);
    intake.store(127);
    chassis.moveToPoint(44.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);
    
    // Score fourth set of 6 balls
    chassis.moveToPoint(49, -23, 875, {.forwards = false, .maxSpeed = 65}, false);
    intake.score(127);
    move(-20,0,false, scoreDelay);

    intake.stop();
    move(45, 0, false, 400);
    move(-35, 0, false, 600);

    scraper.set_value(false);



    chassis.moveToPose(17, -64, -82, 1500, {.lead = 0.3}, false);
    horLift.set_value(true);
    verLift.set_value(true);
    pros::delay(20);
    intake.store(127);


    move(100,0,false,100);
    scraper.set_value(true);
    move(127,6,false,700);
    scraper.set_value(false);



}

void skills106(){
    wing.set_value(true);
    intake.store(127);

    chassis.setPose(-5, -46, 0);

    int scraperDist = 65;
    int scraperTimeout = 1200;
    float matchLoaderSpeed = 50;
    int scraperDelay = 1500;
    int scoreDelay = 2300;

    // Get first 4 balls
    chassis.turnToPoint(-21,-25,300,{});
    chassis.moveToPoint(-21,-25,700,{});
    pros::delay(550);
    scraper.set_value(true);
    

    // MCL OFF
    chassis.waitUntilDone();
    lemlib::toggleMCL();
    
    //Score Middle
    chassis.turnToPoint(-8,-8, 500, {.forwards = false});
    chassis.moveToPoint(-8,-8, 900, {.forwards = false,.minSpeed=20},false);
    // pros::delay(350);
    intake.score(115, true);
    chassis.waitUntilDone();
    pros::delay(350);
    // Move to First Goal
    chassis.turnToPoint(-50, -47.1, 500, {});
    intake.stop();
    chassis.moveToPoint(-50, -47.1, 1000, {.minSpeed=60});
    flappier.set_value(false);
    flappy.set_value(false);
    chassis.turnToPoint(-51, -29.1, 600, {.forwards = false});
    chassis.moveToPoint(-51, -29.1, 1000, {.forwards = false, .minSpeed=60});
    pros::delay(500);
    

    // MCL ON
    intake.score(127);
    lemlib::toggleMCL();
    move(-30,0,false, 1000);
    intake.store(127);
    
    // // Scrape First Loader
    chassis.moveToPoint(-46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);

    chassis.moveToPoint(-60, -36, 700,{.forwards = false,.minSpeed=90});
    pros::delay(300);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(-60, 27, 1200,{.forwards = false,.minSpeed=90});
    chassis.moveToPoint(-48, 40, 750, {.forwards = false,.minSpeed=30});
    chassis.turnToPoint(-48, 23, 500, {.forwards = false});
    chassis.moveToPoint(-48, 23, 1200, {.forwards = false});
    pros::delay(650);
    intake.score(127);
    move(-15,0,false, scoreDelay);

    intake.store(127);
    chassis.moveToPoint(-46.0, scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);

    
    // Score second set of 6 balls
    chassis.moveToPoint(-48.5, 23, 1000, {.forwards = false, .minSpeed = 50});
    pros::delay(600);
    intake.score(127);
    move(-15,0,false, 1600);
    scraper.set_value(false);

    move(40,0,false, 300);
    move(-30,0,false, 500);

    // MCL OFF
    lemlib::toggleMCL();
    intake.store(127);

    // Low Goal 
    chassis.turnToHeading(97, 900);
    chassis.moveToPoint(-25, 24, 600);
    pros::delay(100);
    // scraper.set_value(true);
    chassis.turnToPoint(-14.8, 18.3, 500, {});
    // scraper.set_value(false);
    chassis.moveToPoint(-14.8, 18.3, 950, {});
    descore.set_value(true);
    pros::delay(300);
    intake.outtake(115);
    pros::delay(1500);

    // // intake.stop();

    // Move to barrier
    chassis.turnToPoint(-36.1, 61.4, 600, {.forwards = false,.minSpeed=90});
    descore.set_value(false);
    chassis.moveToPoint(-36.1, 61.4, 1000,{.forwards = false, .minSpeed=90});
    chassis.turnToPoint(-20, 65, 400, {.minSpeed=60});
    chassis.moveToPoint(-20, 65, 700, {.minSpeed=60},false);
    // chassis.turnToHeading(78, 300,{},false);
    horLift.set_value(true);
    // verLift.set_value(true);
    intake.store(127);

    // // Going Over barrier 
    move(74,-8,false,1500);
    scraper.set_value(true);
    move(60,-8,false,250);


    horLift.set_value(false);
    // verLift.set_value(false);

    // // // // Distance Reset 
    chassis.turnToHeading(90, 500,{}, false);
    resetPositionLeft();
    resetPositionFront();

    
    // // MCL ON
    lemlib::toggleMCL();
    scraper.set_value(false);



    // chassis.turnToPoint(-5.9,23.1, 800,{.forwards=false});
    // chassis.moveToPoint(-5.9,23.1, 1300,{.forwards=false},false);

    chassis.turnToPoint(0,30, 800,{.forwards=false});
    chassis.moveToPoint(0,30, 1300,{.forwards=false},false);
    //29
    // // // MCL OFF
    lemlib::toggleMCL();

    // // Move to Middle Goal
    chassis.turnToPoint(19.5, 18.5, 750, {.forwards = false, .maxSpeed = 90});
    chassis.moveToPoint(19.5, 18.5, 1000, {.forwards = false,.maxSpeed = 90});
    chassis.turnToPoint(8.5, 7.5, 700, {.forwards = false});
    chassis.moveToPoint(8.5, 7.5, 800, {.forwards = false},false);


    // WORKING MIDDLE 
    // chassis.turnToPoint(18.5, 17.5, 750, {.forwards = false, .maxSpeed = 90});
    // chassis.moveToPoint(18.5, 17.5, 1000, {.forwards = false,.maxSpeed = 90});
    // chassis.turnToPoint(10, 9, 700, {.forwards = false});
    // chassis.moveToPoint(10, 9, 800, {.forwards = false},false);

    
    // // // // // Score Middle Goal 
    intake.score(127, true);
    move(-15,0,false, 150);
    intake.score(60, true);
    pros::delay(2000);
    chassis.turnToPoint(48.5, 51, 300,{},false);
    intake.store(127);
    pros::delay(150);


    // Move to High Goal
    chassis.moveToPoint(48.5, 51, 900, {.minSpeed=60});
    pros::delay(250);
    scraper.set_value(true);
    chassis.waitUntilDone();
   
    chassis.turnToPoint(51, 34, 500,{.forwards=false});
    chassis.moveToPoint(51, 34, 800,{.forwards=false});
    pros::delay(500);
    intake.score(127);
     // MCL ON/SCORE 
    lemlib::toggleMCL();
    move(-30,0,false, 1400);
    intake.store(127);

    // Third match loader
    chassis.moveToPoint(46.5, scraperDist, scraperTimeout,{.maxSpeed=matchLoaderSpeed},false);
    move(40, 0, false, scraperDelay);
    
    chassis.moveToPoint(60, 36, 700,{.forwards = false,.minSpeed=90});
    pros::delay(300);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);
    chassis.moveToPoint(60, -27, 1200,{.forwards = false, .minSpeed=90});
    chassis.moveToPoint(48, -40, 800, {.forwards = false,.minSpeed=30});
    chassis.turnToPoint(48, -23, 500, {.forwards = false});
    chassis.moveToPoint(48, -23, 1000, {.forwards = false}, false);
    //score 3rd set of blocks
    intake.score(127);
    move(-15,0,false, scoreDelay);



    // Fourth match loader
    intake.store(127);
    chassis.moveToPoint(46.5, -scraperDist, scraperTimeout, {.maxSpeed=matchLoaderSpeed}, false);
    move(40, 0, false, scraperDelay);
    intake.stop();
    flappy.set_value(false);
    flappier.set_value(false);

    chassis.moveToPoint(49, -23, 1000, {.forwards = false, .minSpeed = 60}, false);
    intake.score(127);
    move(-15,0,false, scoreDelay);
    scraper.set_value(false);


    move(40,0,false, 300);
    move(-30,0,false, 500);

    // // MCL OFF
    // lemlib::toggleMCL();
    // intake.store(127);

    // // Low Goal 
    // chassis.turnToHeading(-83, 900);
    // chassis.moveToPoint(25, -24, 600);
    // pros::delay(100);
    // // scraper.set_value(true);
    // chassis.turnToPoint(13, -14, 500, {});
    // // scraper.set_value(false);
    // chassis.moveToPoint(13, -14, 950, {});
    // // descore.set_value(true);
    // pros::delay(300);
    // intake.outtake(110);
    // pros::delay(800);


    // chassis.turnToPoint(36.1, -61.4, 600, {.forwards = false,.minSpeed=60});
    // // descore.set_value(false);
    // chassis.moveToPoint(36.1, -61.4, 1000,{.forwards = false, .minSpeed=60});
    // chassis.turnToPoint(20, -65, 400, {.minSpeed=60});
    // chassis.moveToPoint(20, -65, 700, {.minSpeed=60},false);
    chassis.moveToPose(17, -64, -82, 1500, {.lead = 0.3}, false);


    horLift.set_value(true);
    // verLift.set_value(true);
    intake.store(100);

    //park
    move(100,0,false,100);
    scraper.set_value(true);
    move(127,6,false,350);
    intake.outtake(127);
    move(127, 6, false, 400);
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
