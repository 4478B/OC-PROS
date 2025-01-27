#include "auton_routes.h"
#include "auto_clamp.h"
#include "auton_selector.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include <cstdlib>
#include "devices.h"
#include "pros/motors.h"
#include "testing.h"
#include <iomanip>
#include "color_sort.h"
#include "old_systems.h"

// This method is designed for testing sections of autons separately

// IF CALLED IN COMPETITION/WITH COMM SWITCH:
// -- functions as regular delay
// IF CALLED NOT IN COMPETITION & WITHOUT COMM SWITCH:
// -- delays until user presses X or it times out
// -- prints section information to controller screen

// in the future we can make it print information about ending positions

int autonSection = 0;
void endSection(int delay)
{

    // functions as normal delay
    if (inCompetition)
    {
        pros::delay(delay);
    }
    else
    {
        // handle updating timers
        int startTime = pros::millis();
        int deltaTime = startTime - prevTime;
        totalTime += deltaTime;
        prevTime = startTime;

        // print timer positions to console for permanent logging
        std::cout << std::setw(14) << autonSection << " | "
                  << std::setw(14) << deltaTime << " | "
                  << std::setw(14) << totalTime << " | "
                  << std::endl;

        // print timer positions on controller and screen for temporary logging
        // pros::lcd::print(5, "Auton Section: %f", autonSection);
        // pros::lcd::print(5, "Section Time: %f", deltaTime);
        // pros::lcd::print(5, "Total Time: %f", totalTime);

        // controller.set_text(2,1,std) // controller WIP bc set_text is bad

        // while button hasn't been pressed and hasn't timed out
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && pros::millis() - startTime < delay)
        {
            pros::delay(20);
        }
        // updates controller screen with section information
        autonSection++;
        controller.clear_line(1);
        controller.set_text(1, 1, std::to_string(autonSection).c_str());
    }
}

// This file includes all of the routes coded in PROS for our robot
// The routes should have linked path.jerryio files for reference

bool isRedTeam = competitionSelector.isRedTeam;

void progSkills(){
    chassis.setPose(-58, 0, 270);
    oc_motor.move(127);
    delay(500);
    oc_motor.move(-127);
    delay(100);
    clamp.set_value(LOW);
    drivePID(-9, 1000);
    oc_motor.brake();
    // endSection(50000);
    delay(300);
    chassis.turnToHeading(358, 2000);
    drivePID(-22,1000,30);
    drivePID(-16, 1000, 30);
    clamp.set_value(HIGH);
    delay(150);
    chassis.turnToHeading(90, 2000);
    intake.move(127);
    drivePID(25);
    delay(300);
    chassis.turnToHeading(125, 2000);
    drivePID(30);
    endSection(500);
    chassis.turnToHeading(180, 2000);
    drivePID(16);
    delay(3000);
    endSection(500);
    drivePID(-9);
    chassis.turnToHeading(270, 2000);
    drivePID(64, 6000, 10);
    delay(500);
    drivePID(-13);
    chassis.turnToHeading(180, 2000);
    drivePID(16,3000,30);
    delay(150);
    drivePID(-16);
    chassis.turnToHeading(45, 2000);
    drivePID(-21,3000,25);
    intake.move(-90);
    clamp.set_value(LOW);
    endSection(1000);
    drivePID(19);
    intake.move(127);
    chassis.turnToHeading(180, 2000);
    
    drivePID(-55,5000,35);

    chassis.turnToHeading(180,2000);

    drivePID(-22);
    drivePID(-10, 1000, 35);
    clamp.set_value(HIGH);
    delay(150);
    endSection(50000);
    drivePID(5, 600);
    endSection(500);
    chassis.turnToHeading(90, 2000);
    intake.move(127);
    drivePID(25);
    delay(300);
    chassis.turnToHeading(55, 2000);
    drivePID(30);
    endSection(500);
    chassis.turnToHeading(0, 2000);
    drivePID(18);
    delay(3000);
    endSection(500);
    drivePID(-10);
    chassis.turnToHeading(274, 2000);
    drivePID(64, 6000, 10);
    delay(500);
    drivePID(-13);
    chassis.turnToHeading(0, 2000);
    drivePID(16,3000,30);
    delay(150);
    drivePID(-16);
    chassis.turnToHeading(135, 2000);
    drivePID(-23,3000,25);
    intake.move(-90);
    clamp.set_value(LOW);
    endSection(1000);
    drivePID(25);
    intake.move(127);
}



void ringRush(int mode){
    // 6 ring on one goal ringside route
    if(isRedTeam){

        chassis.setPose(58, 48 - 3.5 - 13.5/2,90);

        // * Ring Rush
        intake.move(127);
        chassis.moveToPoint(chassis.getPose().x + 50, chassis.getPose().y + 10, 2000, {.minSpeed=10});
        delay(800);
        left_doinker.extend(); // extend
        chassis.waitUntilDone();
        delay(500);
        //color_sort.waitUntilDetected(2000,RingColor::red);
        intake.brake();

        // * Retreat
        chassis.moveToPoint(-33, 36, 1000, {.forwards=false}, false);
        left_doinker.retract();
        

        // * Goal
        Pose goal(-24,24,0);
        chassis.swingToPoint(goal.x, goal.y, lemlib::DriveSide::RIGHT,1000,{.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE},false);
        chassis.moveToPoint(goal.x, goal.y,1000,{.forwards=false});
        //while(chassis.isInMotion() && !auto_clamp.isDetected()){
        //    delay(20);
        //}
        clamp.extend();
        delay(50);
        // switch to primitive control if odom fails
        /*if(!auto_clamp.isGoalClamped()){
            clamp.retract();
            all_motors.move_velocity(-50);
            auto_clamp.waitUntilClamp(10,1000);
            clamp.extend();
            delay(50);
            all_motors.brake();
        }*/
        chassis.waitUntilDone();

        // * Safe
        chassis.turnToPoint(-24,48,1000,{},false);
        intake.move(127);
        chassis.moveToPoint(-24,56,2000,{.maxSpeed=50},false);
        intake.brake();
        // TODO: STOP IF BLUE INTAKED

        // * Corner
        /*Pose corner(-65,65,-45);
        chassis.moveToPose(corner.x,corner.y,corner.theta,1000,{.minSpeed=20});
        while(chassis.isInMotion() && chassis.getPose().distance(corner) > 6){
            delay(20);
        }
        intake.move(127);
        chassis.waitUntilDone();
        
        // * Reset
        Pose cornerReset(-65,65,0);
        all_motors.move_velocity(40);
        delay(500);

        // debug start
        pros::lcd::print(1,"x start: %d", chassis.getPose().x);
        pros::lcd::print(2,"y start: %d", chassis.getPose().y);
        delay(500);
        pros::lcd::print(3,"x end: %d", chassis.getPose().x);
        pros::lcd::print(4,"y end: %d", chassis.getPose().y);
        // debug end

        chassis.setPose(cornerReset.x,cornerReset.y,imu.get_heading());
        all_motors.brake();*/

        // * Preload
        Pose preload(-55,42,0);
        chassis.swingToHeading(200,lemlib::DriveSide::RIGHT,1000,{.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE},false);
        chassis.moveToPoint(preload.x,preload.y,1000,{.minSpeed=40,.earlyExitRange=5});

        // * Alliance
        Pose alliance(-47,0,0);
        chassis.moveToPoint(alliance.x,alliance.y,1000,{.minSpeed=20,.earlyExitRange=5});
        while(chassis.isInMotion() && chassis.getPose().distance(alliance) > 6){
            delay(20);
        }
        redirect.extend();
        chassis.waitUntilDone();
        chassis.moveToPoint(alliance.x,alliance.y-10, 1000,{.maxSpeed=40,.minSpeed=40,.earlyExitRange=5});

        //color_sort.waitUntilDetected(1000);
        //redirect.retract();
        chassis.waitUntilDone();
        intake.brake();

        // This is where the route splits into different modes
        // depending on alliances and strategies

        if(mode == 1){
            // * ALLIANCE STAKE
            Pose stake(-63,0,270);
            chassis.moveToPose(stake.x,stake.y,stake.theta,3000,{.minSpeed=20});
            // score on stake  
        }
        else if(mode == 2){
            Pose ladder(-10,0,90);
            oc_piston.extend();
            chassis.moveToPose(ladder.x,ladder.y,ladder.theta,3000,{.maxSpeed=50,.minSpeed=20});
        }
        else{
            Pose posCorner(-65,-65,45);
            if (mode == 3){
                chassis.moveToPose(posCorner.x,posCorner.y+5,180,1000,{.minSpeed=70},false);
            }
            else if(mode == 4){
                chassis.turnToPoint(posCorner.x,posCorner.y,1000,{.forwards=false},false);
                chassis.moveToPose(posCorner.x,posCorner.y,posCorner.theta,1000,{.forwards=false,.minSpeed=70},false);
            }

        }
    }
    else{

    }
}

void redGoalSideSugarRush(int i)
{
    clamp.set_value(LOW);
    right_doinker.set_value(HIGH);
    chassis.setPose(0, 0, 90);
    drivePID(40, 1000, 70);
    endSection();

    chassis.turnToHeading(275, 1000, {}, false);
    right_doinker.set_value(LOW);
    endSection();

    chassis.turnToHeading(320, 1000);
    intake.move(127);
    drivePID(15);
    delay(270);
    intake.brake();
    //drivePID(8);
    endSection(100);

    chassis.turnToHeading(180, 1000, {}, false);
    drivePID(-27,1000,30);
    clamp.set_value(HIGH);
    endSection(500);

    chassis.turnToHeading(270, 1000, {}, false);
    intake.move(127);
    drivePID(35, 1000);
    endSection();

    chassis.turnToHeading(190, 1000, {}, false);
    right_doinker.set_value(HIGH);
    drivePID(34, 1000);
    endSection();

    chassis.turnToHeading(90, 1000, {}, false);
    endSection();

    intake.brake();
    drivePID(20, 1000);
    clamp.set_value(LOW);
    endSection();

    right_doinker.set_value(LOW);
    chassis.turnToHeading(225, 1000, {}, false);
    endSection();

    drivePID(-24, 1000, 80);
    chassis.turnToHeading(270, 1000);
}
void blueGoalSideSugarRush(int i)
{
    clamp.set_value(LOW);
    left_doinker.set_value(HIGH);
    chassis.setPose(0, 0, -90);
    drivePID(40, 1000, 70);
    endSection();

    chassis.turnToHeading(-275, 1000, {}, false);
    left_doinker.set_value(LOW);
    endSection();

    chassis.turnToHeading(-320, 1000);
    intake.move(127);
    drivePID(15);
    delay(270);
    intake.brake();
    //drivePID(8);
    endSection(100);

    chassis.turnToHeading(-180, 1000, {}, false);
    drivePID(-27,1000,30);
    clamp.set_value(HIGH);
    endSection(500);

    chassis.turnToHeading(-270, 1000, {}, false);
    intake.move(127);
    drivePID(35, 1000);
    endSection();

    chassis.turnToHeading(-190, 1000, {}, false);
    left_doinker.set_value(HIGH);
    drivePID(34, 1000);
    endSection();

    chassis.turnToHeading(-90, 1000, {}, false);
    endSection();

    intake.brake();
    drivePID(20, 1000);
    clamp.set_value(LOW);
    endSection();

    left_doinker.set_value(LOW);
    chassis.turnToHeading(-225, 1000, {}, false);
    endSection();

    drivePID(-24, 1000, 80);
    chassis.turnToHeading(-270, 1000);
}

    void LocalFullAWP(int i)
    {
        if(i==1){
                // * Right SIDE
                clamp.set_value(HIGH);
            chassis.setPose(0, 0, 221);

            // arm functions
            //setArmAlliance();
            delay(1000);
            drivePID(-6);
            //setArmTop();
            endSection(700);

            // move to alliance ring and score it
            chassis.turnToHeading(159, 1000, {}, false);
            intake.move(80);
            drivePID(27, 1000, 45);
            //setArmBottom();
            delay(150);
            intake.move(20);
            // chassis.turnToHeading(180 ,1000,{},false);

            drivePID(-15, 1000);
            delay(500);
            intake.brake();
            chassis.turnToHeading(252, 700, {}, false);
            intake.brake();

            // go to goal and clamp
            drivePID(-18, 1500);
            drivePID(-10, 1000, 42.5);
            clamp.set_value(LOW);
            waitUntilAnyIntake(300);
            intake.move(127);
            endSection(500);
            chassis.turnToHeading(20, 1000, {}, false);
            intake.move(-50);
            delay(100);
            intake.move(127);

            // score ring 2
            drivePID(29, 1000, 45);
            endSection();

            // go to middle
            chassis.turnToHeading(165, 1000, {}, false);
            //setArmMid();
            left_motors.move(42);
            right_motors.move(42);
            delay(5000);
            left_motors.brake();
            right_motors.brake();
        }
        else{
            // * Left SIDE
            delay(500);
            clamp.set_value(HIGH);
            chassis.setPose(0, 0, -221);

            // arm functions
            //setArmAlliance();
            delay(1000);
            drivePID(-6);
            //setArmTop();
            endSection(700);

            // move to alliance ring and score it
            chassis.turnToHeading(-159, 1000, {}, false);
            intake.move(80);
            drivePID(27, 1000, 45);
            //setArmBottom();
            delay(150);
            intake.move(20);
            // chassis.turnToHeading(180 ,1000,{},false);

            drivePID(-15, 1000);
            delay(500);
            intake.brake();
            chassis.turnToHeading(-252, 700, {}, false);
            intake.brake();

            // go to goal and clamp
            drivePID(-18, 1500);
            drivePID(-10, 1000, 42.5);
            clamp.set_value(LOW);
            waitUntilAnyIntake(300);
            intake.move(127);
            endSection(500);
            chassis.turnToHeading(-20, 1000, {}, false);
            intake.move(-50);
            delay(100);
            intake.move(127);

            // score ring 2
            drivePID(29, 1000, 45);
            endSection();

            // go to middle
            chassis.turnToHeading(-165, 1000, {}, false);
            //setArmMid();
            left_motors.move(42);
            right_motors.move(42);
            delay(5000);
            left_motors.brake();
            right_motors.brake();
        }
       
    }
void safeAWP(int mode){
    if(mode == -1){
        // * LEFT SIDE
        // todo: fix the brake types
        all_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

        // push middle ring
        intake.move(-127);
        chassis.setPose(0, 0, 0);
        drivePID(12,800);
        
        // face and score ring on alliance stake
        chassis.turnToHeading(90, 1000, {}, false);
        drivePID(2,500);
        oc_motor.move(127);
        delay(500);
        oc_motor.move(-127);
        delay(100);
        drivePID(-12,800);
        oc_motor.brake();
        // todo: fix the brake types
        all_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
        drivePID(3,500);

        // turn and clamp
        chassis.turnToHeading(42, 1000, {}, false);
        drivePID(-40,3000,30);
        clamp.extend();

        // turn and score ring on mogo
        chassis.turnToHeading(180, 1000, {}, false);
        intake.move(127);
        drivePID(30,2000,30);

        // turn to mid and touch
        chassis.turnToHeading(-10, 1000, {}, false);
        oc_piston.extend();
        drivePID(50,2000,30);
    }
    else{
        // * RIGHT SIDE
        // todo: fix the brake types
        all_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

        // push middle ring
        intake.move(-127);
        chassis.setPose(0, 0, 0);
        drivePID(12,800);
        
        // face and score ring on alliance stake
        chassis.turnToHeading(-90, 1000, {}, false);
        drivePID(2,500);
        oc_motor.move(127);
        delay(500);
        oc_motor.move(-127);
        delay(100);
        drivePID(-12,800);
        oc_motor.brake();
        // todo: fix the brake types
        all_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
        drivePID(3,500);

        // turn and clamp
        chassis.turnToHeading(-42, 1000, {}, false);
        drivePID(-40,3000,30);
        clamp.extend();

        // turn and score ring on mogo
        chassis.turnToHeading(-180, 1000, {}, false);
        intake.move(127);
        drivePID(30,2000,30);

        // turn to mid and touch
        chassis.turnToHeading(-10, 1000, {}, false);
        oc_piston.extend();
        drivePID(50,2000,30);
    }
}


void GoalRush(){

    if(isRedTeam){

    }
    else{

    }
}
void ringAWP(){
    if(isRedTeam){

    }
    else{

    }
}
void goalAWP(){
    if(isRedTeam){

    }
    else{

    }
}