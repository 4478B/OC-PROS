#include "overclock_mech.h"
#include "devices.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include <cmath>

namespace OCConfig {
    const double GOAL_THRESHOLD = 3;
    const double MAX_TORQUE = 999;
}

OCPos::OCPos(int angle, lemlib::AngularDirection direction, bool isUsingPID, pros::motor_brake_mode_e_t brakeMode)
    : angle(angle), direction(direction), isUsingPID(isUsingPID), brakeMode(brakeMode) {}

double OCPos::getAngle() { return angle; }
lemlib::AngularDirection OCPos::getDirection() { return direction; }
double OCPos::getDistanceBetween(double angleDeg) { return angle - angleDeg; }
bool OCPos::isGoalMet(double currentPos) {
    double error = getDistanceBetween(currentPos);
    if (isUsingPID) {
        if (direction == lemlib::AngularDirection::CW_CLOCKWISE) return error < 0;
        if (direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) return error > 0;
        if (direction == lemlib::AngularDirection::AUTO) return fabs(error) < OCConfig::GOAL_THRESHOLD;
    }
    return fabs(error) < OCConfig::GOAL_THRESHOLD;
}
bool OCPos::equals(OCPos pos) { return angle == pos.getAngle(); }
pros::motor_brake_mode_e OCPos::getBrakeMode() { return brakeMode; }
bool OCPos::getIsUsingPID() { return isUsingPID; }
lemlib::AngularDirection OCPos::getAngularDirection() { return direction; }

namespace OCMovement {
    OCPos HIGH_POS(110, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, false, pros::E_MOTOR_BRAKE_HOLD);
    OCPos LOW_POS(338, lemlib::AngularDirection::CW_CLOCKWISE, true, pros::E_MOTOR_BRAKE_COAST);
    OCPos TOP_POS(200, lemlib::AngularDirection::AUTO, true, pros::E_MOTOR_BRAKE_HOLD);
    OCPos NONE_POS(-1, lemlib::AngularDirection::AUTO, true, pros::E_MOTOR_BRAKE_INVALID);
}

OverclockMech::OverclockMech() : targetPos(OCMovement::NONE_POS), returnLowAfterMove(false), isActive(false) {}

void OverclockMech::enable() { setActive(true); }
void OverclockMech::disable() { setActive(false); }
void OverclockMech::setActive(bool active) {
    if (isActive != active) {
        isActive = active;
        if (active) {
            oc_motor.set_brake_mode(targetPos.getBrakeMode());
            if (targetPos.getIsUsingPID()) ocPID.reset();
        } else {
            oc_motor.brake();
        }
    }
}
bool OverclockMech::getIsActive() { return isActive; }
void OverclockMech::setTargetPos(OCPos targetPosition, bool returnLowAfterMove) {
    if (targetPosition.equals(OCMovement::NONE_POS)) {
        disable();
    } else {
        this->returnLowAfterMove = returnLowAfterMove;
        targetPos = targetPosition;
        oc_motor.set_brake_mode(targetPos.getBrakeMode());
        if (!isActive) enable();
    }
}
OCPos OverclockMech::getTargetPos() { return targetPos; }
double OverclockMech::getCurrentPos() { return ocRot.get_angle() / 100.0; }
bool OverclockMech::getReturnLowAfterMove() { return returnLowAfterMove; }
bool OverclockMech::waitUntilDone(int msecTimeout) {
    int startTime = pros::millis();
    while (isActive && pros::millis() - startTime < msecTimeout) {
        pros::delay(20);
    }
    return !isActive;
}

void oc_control_task(void *param) {
    int goalCount = 0;
    while (overclock_mech.getIsActive()) {
        if (overclock_mech.getTargetPos().isGoalMet(overclock_mech.getCurrentPos()) || oc_motor.get_torque() > OCConfig::MAX_TORQUE) {
            goalCount++;
            if (goalCount >= OCConfig::GOAL_THRESHOLD) {
                ocPID.reset();
                oc_motor.set_brake_mode(overclock_mech.getTargetPos().getBrakeMode());
                oc_motor.brake();
                overclock_mech.disable();
                goalCount = 0;
                if (overclock_mech.getReturnLowAfterMove()) {
                    overclock_mech.setTargetPos(OCMovement::LOW_POS, false);
                }
            }
        } else {
            goalCount = 0;
            double error = overclock_mech.getTargetPos().getDistanceBetween(overclock_mech.getCurrentPos());
            if (overclock_mech.getTargetPos().getIsUsingPID()) {
                double nextMovement = ocPID.update(error);
                nextMovement = std::clamp(nextMovement, -200.0, 200.0);
                oc_motor.move_velocity(nextMovement);
            } else {
                if (overclock_mech.getTargetPos().getAngularDirection() == AngularDirection::AUTO) {
                    pros::lcd::print(1, "ERROR: AUTO DIRECTION NOT SUPPORTED");
                } else if (overclock_mech.getTargetPos().getAngularDirection() == AngularDirection::CW_CLOCKWISE) {
                    oc_motor.move(127);
                } else if (overclock_mech.getTargetPos().getAngularDirection() == AngularDirection::CCW_COUNTERCLOCKWISE) {
                    oc_motor.move(-127);
                }
            }
        }
    }
}
// Display all the information about the overclock mechanism on the LCD screen on lines 1-7
void oc_screen_task(void *param) {
    while (true) {
        pros::lcd::print(1, "OC Pos: %d", overclock_mech.getTargetPos().getAngle());
        pros::lcd::print(2, "OC Current: %f", overclock_mech.getCurrentPos());
        pros::lcd::print(3, "OC Active: %d", overclock_mech.getIsActive());
        pros::lcd::print(4, "OC Return Low: %d", overclock_mech.getReturnLowAfterMove());
        pros::lcd::print(5, "OC Brake Mode: %d", overclock_mech.getTargetPos().getBrakeMode());
        pros::lcd::print(6, "OC Using PID: %d", overclock_mech.getTargetPos().getIsUsingPID());
        pros::lcd::print(7, "OC Direction: %d", overclock_mech.getTargetPos().getAngularDirection());
        pros::delay(100);
    }
}
