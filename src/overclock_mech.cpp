#include "overclock_mech.h"
#include "devices.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include <cmath>

namespace OCConfig {
    const double GOAL_THRESHOLD = 3;
    const double MAX_TORQUE = 999;
}

OCPos::OCPos(double angle, lemlib::AngularDirection direction, bool isUsingPID, pros::motor_brake_mode_e_t brakeMode)
    : angle(angle), direction(direction), isUsingPID(isUsingPID), brakeMode(brakeMode) {}

double OCPos::getAngle() { return angle; }
lemlib::AngularDirection OCPos::getDirection() { return direction; }
double OCPos::getDistanceBetween(double angleDeg) { return angle - angleDeg; }

bool OCPos::equals(OCPos pos) { return angle == pos.getAngle(); }
pros::motor_brake_mode_e OCPos::getBrakeMode() { return brakeMode; }
bool OCPos::getIsUsingPID() { return isUsingPID; }
lemlib::AngularDirection OCPos::getAngularDirection() { return direction; }

// when looking at it so brain is on left
// for motor, negative is clockwise, positive is counterclockwise
// for rotation sensor, negative is counterclockwise, postive is clockwise
namespace OCMovement {
    OCPos HIGH_POS(101, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, false, pros::E_MOTOR_BRAKE_HOLD);
    OCPos LOW_POS(333, lemlib::AngularDirection::CW_CLOCKWISE, true, pros::E_MOTOR_BRAKE_COAST);
    OCPos TOP_POS(200, lemlib::AngularDirection::AUTO, true, pros::E_MOTOR_BRAKE_HOLD);
    OCPos MID_POS(247,lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,true,pros::E_MOTOR_BRAKE_HOLD);
    OCPos NONE_POS(-1, lemlib::AngularDirection::AUTO, true, pros::E_MOTOR_BRAKE_INVALID);
}

OverclockMech::OverclockMech() : targetPos(OCMovement::NONE_POS), isActive(false) {}

void OverclockMech::enable() { setActive(true); }
void OverclockMech::disable() { setActive(false); }
void OverclockMech::setActive(bool active) {
    if (isActive != active) {
        isActive = active;
        oc_motor.set_brake_mode(targetPos.getBrakeMode());
        if (targetPos.getIsUsingPID()) ocPID.reset();
        else {
            oc_motor.brake();
        }
    }
}
bool OverclockMech::getIsActive() { return isActive; }
void OverclockMech::setTargetPos(OCPos targetPosition) {
    if (targetPosition.equals(OCMovement::NONE_POS)) {
        disable();
    } else {
        targetPos = targetPosition;
        if (!isActive) enable();
    }
}
bool OverclockMech::isGoalMet(double error) {
    
    if (!getTargetPos().getIsUsingPID()) {
        lemlib::AngularDirection direction = getTargetPos().getDirection();
        if (direction == lemlib::AngularDirection::CW_CLOCKWISE) return error < 0;
        if (direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) return error > 0;
        if (direction == lemlib::AngularDirection::AUTO) return fabs(error) < OCConfig::GOAL_THRESHOLD;
    }
    return fabs(error) < OCConfig::GOAL_THRESHOLD;
}

OCPos OverclockMech::getTargetPos() { return targetPos; }
double OverclockMech::getCurrentPos() { return ocRot.get_angle() / 100.0; }
bool OverclockMech::waitUntilDone(int msecTimeout) {
    int startTime = pros::millis();
    while (isActive && pros::millis() - startTime < msecTimeout) {
        pros::delay(20);
    }
    return !isActive;
}

void oc_control_task(void* param) {
    
    while (oc_mech.getIsActive()) {

        oc_mech.error = oc_mech.getTargetPos().getDistanceBetween(oc_mech.getCurrentPos());

        if (oc_mech.isGoalMet(oc_mech.error) || oc_motor.get_torque() > OCConfig::MAX_TORQUE) {
            oc_mech.goalCount++;
            if (oc_mech.goalCount >= OCConfig::GOAL_THRESHOLD) {
                ocPID.reset();
                oc_motor.brake();
                oc_mech.goalCount = 0;
                oc_mech.disable();
                
            }
        } else {
            oc_mech.goalCount = 0;
            double error = oc_mech.getTargetPos().getDistanceBetween(oc_mech.getCurrentPos());
            pros::lcd::print(5, "Error: %f", error);
            if (oc_mech.getTargetPos().getIsUsingPID()) {
                double nextMovement = ocPID.update(error);
                nextMovement = std::clamp(nextMovement, -200.0, 200.0);
                //oc_motor.move_velocity(nextMovement);
            } else {
                if (oc_mech.getTargetPos().getAngularDirection() == AngularDirection::AUTO) {
                    pros::lcd::print(1, "ERROR: AUTO DIRECTION NOT SUPPORTED");
                } else if (oc_mech.getTargetPos().getAngularDirection() == AngularDirection::CW_CLOCKWISE) {
                    //oc_motor.move(-40);
                } else if (oc_mech.getTargetPos().getAngularDirection() == AngularDirection::CCW_COUNTERCLOCKWISE) {
                    //oc_motor.move(40);
                }
            }
        }
    }
}
// Display all the information about the overclock mechanism on the LCD screen on lines 1-7
void oc_screen_task(void* param) {
    while (true) {
        pros::lcd::print(1, "OC Target: %f", oc_mech.getTargetPos().getAngle());
        pros::lcd::print(2, "OC Current: %f", oc_mech.getCurrentPos());
        pros::lcd::print(3, "OC Active: %s", oc_mech.getIsActive() ? "YES" : "NO");
        pros::lcd::print(4, "OC Torque: %f", oc_motor.get_torque());
        pros::lcd::print(5,"OC isGoalMet %s" , oc_mech.isGoalMet(oc_mech.error) ? "YES" : "NO");
        pros::lcd::print(6,"OC Error %f", oc_mech.error);
        //pros::lcd::print(5, "OC Brake Mode: %s", oc_mech.getTargetPos().getBrakeMode() == pros::E_MOTOR_BRAKE_HOLD ? "HOLD" : "COAST");
        //pros::lcd::print(6, "OC Direction: %s", oc_mech.getTargetPos().getAngularDirection() == AngularDirection::AUTO ? "AUTO" : oc_mech.getTargetPos().getAngularDirection() == AngularDirection::CW_CLOCKWISE ? "CW" : "CCW");
        pros::lcd::print(7, "OC Using PID: %s", oc_mech.getTargetPos().getIsUsingPID() ? "YES" : "NO");
        pros::delay(100);
    }
}
