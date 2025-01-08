#include "auton_routes.h"
#include "auto_clamp.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include <cstdlib>
#include "devices.h"
#include "testing.h"
#include <iomanip>
#include "color_sort.h"

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

void progSkills(bool isRedTeam){
    
}
void RingRush(bool isRedTeam){

    // 6 ring on one goal ringside route
    if(isRedTeam){

        chassis.setPose(58, 48 - 3.5 - 13.5/2,90);

        // * Ring Rush
        intake.move(127);
        chassis.moveToPoint(chassis.getPose().x + 50, chassis.getPose().y + 10, 2000, {.minSpeed=10});
        delay(800);
        left_doinker.extend(); // extend
        chassis.waitUntilDone();
        color_sort.waitUntilDetected(2000,RingColor::red);
        intake.brake();

        // * Retreat
        chassis.moveToPoint(-33, 36, 1000, {.forwards=false}, false);
        left_doinker.retract();
        

        // * Goal
        Pose goal(-24,24,0);
        chassis.swingToPoint(goal.x, goal.y, lemlib::DriveSide::RIGHT,1000,{.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE},false);
        chassis.moveToPoint(goal.x, goal.y,1000,{.forwards=false});
        while(chassis.isInMotion() && !auto_clamp.isDetected()){
            delay(20);
        }
        clamp.extend();
        delay(50);
        // switch to primitive control if odom fails
        if(!auto_clamp.isGoalClamped()){
            clamp.retract();
            all_motors.move_velocity(-50);
            auto_clamp.waitUntilClamp(10,1000);
            clamp.extend();
            delay(50);
            all_motors.brake();
        }
        chassis.waitUntilDone();

        // * Safe
        chassis.turnToPoint(-24,48,1000,{},false);
        intake.move(127);
        chassis.moveToPoint(-24,56,2000,{.maxSpeed=50},false);
        intake.brake();

        // * Corner
        Pose corner(-65,65,-45);
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
        all_motors.brake();

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
        color_sort.waitUntilDetected(1000);
        if(color_sort.isDetected(RingColor::red)){

        }
        redirect.retract();
        chassis.waitUntilDone();

        // * OPTIONAL: Positive Corner
        // THIS NEEDS TO BE TOGGLED ON/OFF DEPENDING ON THE ALLIANCE
        if(false){
            Pose posCorner(-65,-65,45);
            bool allianceClearsCorner = false;
            if(allianceClearsCorner){
                chassis.turnToPoint(posCorner.x,posCorner.y,1000,{.forwards=false},false);
                chassis.moveToPose(posCorner.x,posCorner.y,posCorner.theta,1000,{.forwards=false,.minSpeed=70},false);
            }
            else{
                chassis.moveToPose(posCorner.x,posCorner.y+5,180,1000,{.minSpeed=70},false);
            }
        }

    }
    else{

    }
}
void GoalRush(bool isRedTeam){

    if(isRedTeam){

    }
    else{

    }
}
void ringAWP(bool isRedTeam){
    if(isRedTeam){

    }
    else{

    }
}
void goalAWP(bool isRedTeam){
    if(isRedTeam){

    }
    else{

    }
}