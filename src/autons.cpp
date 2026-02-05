#include "main.h"
#include "devices.h"
#include "intake.h"

// void distanceResetLeft() {

//         double leftDist = left_distance.get() / 25.4;
//         double frontLeftDist = front_left_distance.get() / 25.4;

//         double heading = chassis.getPose().theta * (M_PI / 180.0);

//         float xWall = (leftDist + left_distance_offset) * cos(heading);
//         float yWall = (frontLeftDist + front_left_distance_offset) * cos(heading);

//         float globalX = 72 - xWall;
//         float globalY = 72 - yWall;


//         chassis.setPose(globalX, globalY, chassis.getPose().theta);
//         printf("Global X: %.2f, Global Y: %.2f\n", globalX, globalY);

// }

void skills() {
    chassis.setPose(-13, -47, -90);

    int scraperDist = 70;

    descore.set_value(true);
    chassis.moveToPoint(-46.5, -48, 2000, {}, false);
    chassis.turnToPoint(-46.5, -scraperDist, 800, {}, false);
    scraper_piston.toggle();
    pros::delay(300);
    bottom_intake.move(127);
    chassis.moveToPoint(-46.5, -scraperDist, 1800, {.maxSpeed=45},false);
    pros::delay(1800);
    chassis.moveToPoint(-46.5, -47, 800, {.forwards = false, .maxSpeed=50},false);
    chassis.turnToPoint(-61.5, -36, 600,{.forwards = false},false);
    chassis.moveToPoint(-61.5, -36, 800,{.forwards = false, .maxSpeed = 50},false);
    chassis.turnToPoint(-61.5, 36, 800,{.forwards = false},false);
    chassis.moveToPoint(-61.5, 36, 2000,{.forwards = false, .maxSpeed = 70},false);
    chassis.turnToPoint(-48, 36,800, {},false);
    chassis.moveToPoint(-48, 36, 800, {.maxSpeed = 70}, false);
    chassis.turnToPoint(-48, 24, 800, {.forwards = false}, false);
    chassis.moveToPoint(-48, 24, 800, {.forwards = false}, false);
	top_score.set_value(false);
    top_intake.move(127);
    bottom_intake.move(127);
    pros::delay(1800);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);
    chassis.moveToPoint(-46.5, scraperDist, 1800, {.maxSpeed=45}, false);
    bottom_intake.move(127);
    pros::delay(1600);
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false,.maxSpeed = 40}, false);
    top_score.set_value(false);
    top_intake.move(127);
    pros::delay(1800);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);
    chassis.moveToPoint(-48, 36, 900, {}, false);
    chassis.turnToPoint(46.5, 36, 800,{},false);
    chassis.moveToPoint(46.5, 36, 2500,{},false);
    chassis.turnToPoint(46.5, scraperDist, 800, {}, false);
    chassis.moveToPoint(46.5, scraperDist, 1800,{},false);
    bottom_intake.move(127);
    pros::delay(1600);
    chassis.moveToPoint(46.5, 47, 1000,{.forwards=false, .maxSpeed=50}, false);
    chassis.turnToPoint(61.5, 36, 600,{.forwards = false},false);
    chassis.moveToPoint(61.5, 36, 800,{.forwards = false, .maxSpeed = 50},false);
    chassis.turnToPoint(61.5, -36, 800,{.forwards = false},false);
    chassis.moveToPoint(61.5, -36, 2000,{.forwards = false, .maxSpeed = 70},false);
    chassis.turnToPoint(48, -36,800, {},false);
    chassis.moveToPoint(48, -36, 800, {.maxSpeed = 70}, false);
    chassis.turnToPoint(48, -24, 800, {.forwards = false}, false);
    chassis.moveToPoint(48, -24, 800, {.forwards = false}, false);
	top_score.set_value(false);
    top_intake.move(127);
    bottom_intake.move(127);
    pros::delay(1800);
    top_intake.move(0);
    bottom_intake.move(0);
    top_score.set_value(true);
    





    

    

    


}


void test() {
    chassis.setPose(24, -48, 0);
    chassis.moveToPoint(24, -24, 3000);
    chassis.waitUntilDone();
    // intake.store(40);
    // distanceResetLeft();
    // pros::delay(100000);
}

void right() {
    chassis.setPose(0, 0, 0);           //move to balls and intake balls
    chassis.moveToPoint(0, 29, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(26, 400);
    chassis.waitUntilDone();
    bottom_intake.move(127);
    chassis.moveToPoint(7, 44, 1000, {.maxSpeed=55});
    chassis.waitUntilDone();
    bottom_intake.move(0);              //move to bottom center goal
    chassis.turnToHeading(-59, 4000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-1, 50, 4000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-59, 4000);
    bottom_intake.move(-127);           //adjust intake so only one ball comes out at a time
    pros::delay(500);
    bottom_intake.move(127);
    pros::delay(500);
    bottom_intake.move(-127);           //score balls
    pros::delay(700);
    bottom_intake.move(0);
    pros::delay(100);
    bottom_intake.move(127);
    pros::delay(200);
    bottom_intake.move(0);              //postition in front of match loader
    chassis.moveToPoint(30, 29, 4000, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-180, 4000);
    chassis.waitUntilDone();            //piston down and move into match loader
    scraper_piston.toggle();
    chassis.turnToPoint(31, 5.5, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(31, 4, 1000, {.minSpeed=65});
    bottom_intake.move(127);            //intake and adjust at match loader
    chassis.waitUntilDone();
    chassis.moveToPoint(31, 7, 500);
    chassis.waitUntilDone();
    chassis.turnToHeading(192, 500);
    chassis.waitUntilDone();
    bottom_intake.move(0);              //move to long goal and score
    chassis.moveToPoint(31, 36, 1000, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(185, 300);
    chassis.waitUntilDone();
    bottom_intake.move(127);
    top_intake.move(127);
    pros::delay(3000);
    bottom_intake.move(0);
    top_intake.move(0);


}


//EITHER SCORES 9 BALLS OR 9 ROLLING VIOLATIONSwe k
//we really need an alligner and horizontal odom
void right9ball(){
    chassis.setPose(0, 0, 0);           //move to balls
    chassis.moveToPoint(0, 28, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(26, 400);
    chassis.waitUntilDone();
    bottom_intake.move(127);            //intake 3 balls
    chassis.moveToPoint(8, 45, 1000, {.maxSpeed=55});
    chassis.waitUntilDone();
    bottom_intake.move(0);              //move to 2 balls (under long goal)

    // chassis.turnToHeading(56, 300);
    chassis.turnToPoint(22, 54, 600);
    chassis.waitUntilDone();            //intake 2 balls
    chassis.moveToPoint(22, 54, 600);
    chassis.waitUntilDone();
    bottom_intake.move(127);
    chassis.turnToPoint(26, 53.6, 600);
    chassis.waitUntilDone();
    chassis.moveToPoint(26, 53.6, 700);
    chassis.waitUntilDone();
    scraper_piston.toggle();
    // pros::delay(300);                   //move back and position in front of match loader
    chassis.moveToPoint(10, 46, 1000, {.forwards=false});
    chassis.waitUntilDone();
    descore.set_value(true);
    float longGoalX = 33.3;
    chassis.turnToPoint(longGoalX, 15, 1000);
    chassis.waitUntilDone();
    bottom_intake.move(0);
    chassis.moveToPoint(longGoalX, 15, 1000);
    chassis.waitUntilDone();
    chassis.turnToPoint(longGoalX, 36.5, 1000, {.forwards=false});
    //chassis.turnToPoint(31, 36, 500, {.forwards = false});
    chassis.waitUntilDone();            //move to and score on long goal
    bottom_intake.move(-100);
    chassis.moveToPoint(longGoalX, 36.5, 500, {.forwards=false});
    chassis.waitUntilDone();
    // chassis.turnToHeading(188, 500); //changed from 189-> 190
    // chassis.waitUntilDone();
    bottom_intake.move(127);
    top_intake.move(127);
    pros::delay(1700);
    top_intake.move(0);                 //scraper down and move to match loader
    // scraper_piston.toggle();
    chassis.turnToPoint(longGoalX, 2, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(longGoalX, 2, 1500, {.minSpeed=65});
    bottom_intake.move(127);       //intake and adjust at match loader
    chassis.waitUntilDone();

    for (int i = 0; i < 3; i++) {
        chassis.moveToPoint(longGoalX, 2, 150, {.forwards=false, .minSpeed=90});
        chassis.waitUntilDone();
        chassis.moveToPoint(longGoalX, -2, 150, {.minSpeed=90});
        chassis.waitUntilDone();
    }

    chassis.turnToPoint(longGoalX, 36, 500, {.forwards=false});
    chassis.waitUntilDone();
    bottom_intake.move(0);              //move to long goal and score
    chassis.moveToPoint(longGoalX, 36, 1000, {.forwards=false});
    chassis.waitUntilDone();
    bottom_intake.move(127);
    top_intake.move(127);
    pros::delay(3000);
    bottom_intake.move(0);
    top_intake.move(0);







}


void left(){
    chassis.setPose(0, 0, 0);           //move to and intake balls
    chassis.moveToPoint(0, 29, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-25, 500);
    chassis.waitUntilDone();
    bottom_intake.move(127);
    chassis.moveToPoint(-9, 45, 1000, {.maxSpeed=35});
    chassis.waitUntilDone(); //move to top center goal
    // chassis.turnToHeading(80, 1000); //tune here
    chassis.turnToPoint(6, 50, 800, {.forwards=false});
    chassis.waitUntilDone();
    bottom_intake.move(0);
    chassis.moveToPoint(6, 50, 1500, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-142.5, 500);
    chassis.waitUntilDone();            //score 2-3 balls
    bottom_intake.move(95);
    top_intake.move(127);
    pros::delay(200);
    bottom_intake.move(0);
    top_intake.move(0);                 //position in front of match loader
    chassis.turnToHeading(-135, 400);
    chassis.waitUntilDone();
    chassis.moveToPoint(-30, 18, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(-180, 1000);
    chassis.waitUntilDone();            //scraper down, move to, and intake from match loader
    scraper_piston.toggle();
    pros::delay(200);
    bottom_intake.move(127);
    chassis.moveToPoint(-30, 0, 800, {.maxSpeed=100});
    chassis.waitUntilDone();

    for (int i = 0; i < 3; i++) {
        chassis.moveToPoint(-30, 3, 150, {.forwards=false, .minSpeed=90});
        chassis.waitUntilDone();
        chassis.moveToPoint(-30, 0, 150, {.minSpeed=90});
        chassis.waitUntilDone();
    }


    // pros::delay(700);            //move to and score on long goal
    chassis.moveToPoint(-29, 36, 1000, {.forwards=false});
    chassis.waitUntilDone();
    bottom_intake.move(127);
    top_intake.move(127);
    pros::delay(800);
    bottom_intake.move(0);
    top_intake.move(0);

    chassis.turnToPoint(-30, 15, 300);
    chassis.waitUntilDone();
    chassis.moveToPoint(-30, 15, 1000);
    chassis.waitUntilDone();

    chassis.turnToPoint(-40, 27, 300, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-40, 27, 1000, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToPoint(-40, 50, 300, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-40, 50, 1000, {.forwards=false});
    chassis.waitUntilDone();






}




// void mill1right() {
//     chassis.setPose(0, 0, 0);

//     // chassis.turnToHeading(-40, 4000);
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(-5, 50, 4000);
//     // chassis.waitUntilDone();

//     chassis.moveToPoint(1, 29, 4000);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(31, 4000);
//     chassis.waitUntilDone();    //move to balls
//     bottom_intake.move(127);
//     chassis.moveToPoint(9, 42, 4000, {.maxSpeed=45});
//     chassis.waitUntilDone();
//     bottom_intake.move(0);  //intake the balls
//     chassis.turnToHeading(-40, 4000);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-3, 53, 4000);
//     // chassis.waitUntilDone();
//     // bottom_intake.move(-127);



//     // pros::delay(20);    //score balls in lower goal
//     //chassis.moveToPoint(0, 24, 4000, {.forwards=false});
//     // chassis.waitUntilDone();
//     // chassis.turnToHeading(-21, 4000);
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(0, 24, 4000);
//     // chassis.waitUntilDone();    //move in front of match loader
//     // chassis.moveToPoint(0, 24, 4000);
//     // chassis.waitUntilDone();
//     // bottom_intake.move(127);
//     // scraper_piston.toggle();
//     // pros::delay(20);    //move to and intake match loader
//     // chassis.moveToPoint(0, 24, 4000);
//     // chassis.waitUntilDone();
//     // top_left.toggle();
// 	// top_right.toggle();
//     // top_intake.move(127);   //move to and outake top goal





// }

void fiveInch() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 4000);
    chassis.waitUntilDone();
}


// void mill1left() {
//     chassis.setPose(0, 0, 0);
//     chassis.moveToPoint(0, 24, 4000);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-21, 4000);
//     chassis.waitUntilDone();
//     bottom_intake.move(127);
//     //chassis.moveToPoint(0, 24, 4000);
//     chassis.moveToPoint(-8, 45, 4000, {.maxSpeed=45});
//     chassis.waitUntilDone();
//     pros::delay(500);
//     bottom_intake.move(0);
//     chassis.turnToHeading(-128, 4000);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(4, 52, 4000, {.forwards=false});
//     chassis.waitUntilDone();
//     bottom_intake.move(127);
//     top_intake.move(127);
//     pros::delay(2000);
//     // chassis.moveToPoint(-20, -4, 4000);


// }


// void odom() {
//     chassis.setPose(0, 0, 0);
//     chassis.turnToHeading(180, 4000);
//     // chassis.moveToPoint(0, 12, 4000);
// }

// void left() {
//     chassis.setPose(0, 0, 0);
//     // chassis.turnToHeading(70, 4000);
//     bottom_intake.move(127);
//     top_intake.move(-127);
//     chassis.moveToPoint(0, 35, 4000, {.maxSpeed=35});
//     chassis.waitUntilDone();
//     bottom_intake.move(0);
//     top_intake.move(0);
//     chassis.moveToPoint(0, 26, 4000, {.forwards=false});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(57, 4000);
//     chassis.waitUntilDone();
//     scraper_piston.toggle();

//     chassis.setPose(0, 0, 0);

//     chassis.moveToPoint(0, 16.5, 1000, {.maxSpeed=50});
//     chassis.waitUntilDone();
//     bottom_intake.move(80);
//     top_intake.move(80);
//     pros::delay(2000);
//     bottom_intake.move(0);
//     top_intake.move(0);

//     chassis.moveToPoint(0, -36.5, 4000, {.forwards=false});
//     // scraper_piston.toggle();
//     chassis.waitUntilDone();
//     chassis.turnToHeading(137, 4000);
//     chassis.waitUntilDone();

//     chassis.setPose(0, 0, 0);


//     // chassis.moveToPoint(0, -15, 4000, {.forwards=false});
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(0, 10, 2000, {.maxSpeed=70, .earlyExitRange=5});
//     chassis.moveToPoint(0, 14, 1000, {.maxSpeed=130, .earlyExitRange=5});
//     chassis.waitUntilDone();



//     bottom_intake.move(127);
//     top_intake.move(-127);
//     pros::delay(600); //2500 for all balls
//     bottom_intake.move(0);
//     top_intake.move(0);
//     chassis.moveToPoint(0, -25, 1500, {.forwards=false, .earlyExitRange=5});
//     chassis.waitUntilDone();

//     top_left.toggle();
// 	top_right.toggle();
//     bottom_intake.move(127);
//     top_intake.move(-127);
//     pros::delay(5000);


// }

// void right() {
//     chassis.setPose(0, 0, 0);

//     bottom_intake.move(127);
//     top_intake.move(-127);
//     chassis.moveToPoint(0, 35, 4000, {.maxSpeed=35});
//     chassis.waitUntilDone();
//     bottom_intake.move(0);
//     top_intake.move(0);
//     chassis.moveToPoint(0, 28, 4000, {.forwards=false});
//     chassis.waitUntilDone();

//     chassis.turnToHeading(-62, 4000);
//     chassis.waitUntilDone();
//     // scraper_piston.toggle();

//     chassis.setPose(0, 0, 0);
//     chassis.moveToPoint(0, 12, 4000, {.maxSpeed=50});
//     chassis.waitUntilDone();
//     bottom_intake.move(-100);
//     top_intake.move(127);
//     pros::delay(2000);
//     bottom_intake.move(0);
//     top_intake.move(0);

//     chassis.moveToPoint(0, -33.5, 4000, {.forwards=false});
//     chassis.waitUntilDone();

//     chassis.turnToHeading(-137, 4000);
//     chassis.waitUntilDone();
//     scraper_piston.toggle();
//     pros::delay(200);

//     chassis.setPose(0, 0, 0);

//     chassis.moveToPoint(0, 16, 1000, {.maxSpeed=130, .earlyExitRange=8});
//     chassis.waitUntilDone();

//     bottom_intake.move(127);
//     top_intake.move(-127);
//     pros::delay(600); //2500 for all balls
//     bottom_intake.move(0);
//     top_intake.move(0);
//     chassis.moveToPoint(0, -14, 1500, {.forwards=false, .earlyExitRange=5});
//     chassis.waitUntilDone();

//     top_left.toggle();
// 	top_right.toggle();
//     bottom_intake.move(127);
//     top_intake.move(-127);
//     pros::delay(5000);



// }

// void sevenRight() {
//     chassis.setPose(0, 0, 0);

//     bottom_intake.move(127);
//     top_intake.move(-127);
//     chassis.moveToPoint(0, 35, 4000, {.maxSpeed=35});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, 28, 4000, {.forwards=false});
//     chassis.waitUntilDone();
//     bottom_intake.move(0);
//     top_intake.move(0);

//     chassis.turnToHeading(-62, 4000);
//     chassis.waitUntilDone();
//     chassis.setPose(0, 0, 0);

//     // chassis.moveToPoint(0, 12, 4000, {.maxSpeed=50});
//     // chassis.waitUntilDone();
//     // bottom_intake.move(-100);
//     // top_intake.move(127);
//     // pros::delay(2000);
//     // bottom_intake.move(0);
//     // top_intake.move(0);

//     chassis.moveToPoint(0, -33.5, 4000, {.forwards=false});
//     chassis.waitUntilDone();

//     chassis.turnToHeading(-135, 4000);
//     chassis.waitUntilDone();
//     scraper_piston.toggle();
//     pros::delay(200);

//     chassis.setPose(0, 0, 0);

//     chassis.moveToPoint(0, 18, 1000, {.maxSpeed=130, .earlyExitRange=8});
//     chassis.waitUntilDone();

//     bottom_intake.move(127);
//     top_intake.move(-127);
//     pros::delay(600); //2500 for all balls
//     bottom_intake.move(0);
//     top_intake.move(0);
//     chassis.moveToPoint(0, -14, 1500, {.forwards=false, .earlyExitRange=5});
//     chassis.waitUntilDone();

//     top_left.toggle();
// 	top_right.toggle();
//     bottom_intake.move(127);
//     top_intake.move(-127);
//     pros::delay(5000);

// }



// void skills() {
//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, 27, 4000);

//     chassis.turnToHeading(38, 1000);
//     bottom_intake.move(127);
//     top_intake.move(-127);
//     //chassis.moveToPoint(15, 46, 5000,{.maxSpeed = 35});
//     chassis.moveToPose(
//         15,
//         46,
//         24,
//         4000,
//         {.lead = 0.3}

//     );

//   chassis.waitUntilDone();

// }
