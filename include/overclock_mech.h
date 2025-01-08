#ifndef OVERCLOCK_MECH_H
#define OVERCLOCK_MECH_H

#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"

namespace OCConfig {
    extern const double GOAL_THRESHOLD;
    extern const double MAX_TORQUE;
}

/**
 * @brief Class representing an overclock position.
 */
class OCPos {
public:
    /**
     * @brief Constructor for OCPos.
     * @param angle The angle in degrees.
     * @param direction The angular direction.
     * @param isUsingPID Whether PID control is used.
     * @param brakeMode The motor brake mode.
     */
    OCPos(int angle, lemlib::AngularDirection direction, bool isUsingPID, pros::motor_brake_mode_e_t brakeMode);

    /**
     * @brief Get the angle.
     * @return The angle in degrees.
     */
    double getAngle();

    /**
     * @brief Get the angular direction.
     * @return The angular direction.
     */
    lemlib::AngularDirection getDirection();

    /**
     * @brief Get the distance between the current angle and a given angle.
     * @param angleDeg The angle in degrees to compare with.
     * @return The distance between the angles.
     */
    double getDistanceBetween(double angleDeg);

    /**
     * @brief Check if the goal is met based on the current position.
     * @param currentPos The current position.
     * @return True if the goal is met, false otherwise.
     */
    bool isGoalMet(double currentPos);

    /**
     * @brief Check if two OCPos objects are equal.
     * @param pos The OCPos object to compare with.
     * @return True if the objects are equal, false otherwise.
     */
    bool equals(OCPos pos);

    /**
     * @brief Get the motor brake mode.
     * @return The motor brake mode.
     */
    pros::motor_brake_mode_e getBrakeMode();

    /**
     * @brief Check if PID control is used.
     * @return True if PID control is used, false otherwise.
     */
    bool getIsUsingPID();

    /**
     * @brief Get the angular direction.
     * @return The angular direction.
     */
    lemlib::AngularDirection getAngularDirection();
private:
    double angle; /**< The angle in degrees. */
    lemlib::AngularDirection direction; /**< The angular direction. */
    bool isUsingPID; /**< Whether PID control is used. */
    pros::motor_brake_mode_e_t brakeMode; /**< The motor brake mode. */
};

namespace OCMovement {
    extern OCPos HIGH_POS; /**< The high position. */
    extern OCPos LOW_POS; /**< The low position. */
    extern OCPos TOP_POS; /**< The top position. */
    extern OCPos NONE_POS; /**< The none position. */
}

/**
 * @brief Class representing the overclock mechanism.
 */
class OverclockMech {
public:
    /**
     * @brief Constructor for OverclockMech.
     */
    OverclockMech();

    /**
     * @brief Enable the overclock mechanism.
     */
    void enable();

    /**
     * @brief Disable the overclock mechanism.
     */
    void disable();

    /**
     * @brief Set the active state of the overclock mechanism.
     * @param active The active state.
     */
    void setActive(bool active);

    /**
     * @brief Get the active state of the overclock mechanism.
     * @return True if active, false otherwise.
     */
    bool getIsActive();

    /**
     * @brief Set the target position for the overclock mechanism.
     * @param targetPosition The target position.
     * @param returnLowAfterMove Whether to return to low position after move.
     */
    void setTargetPos(OCPos targetPosition, bool returnLowAfterMove);

    /**
     * @brief Get the target position of the overclock mechanism.
     * @return The target position.
     */
    OCPos getTargetPos();

    /**
     * @brief Get the current position of the overclock mechanism.
     * @return The current position.
     */
    double getCurrentPos();

    /**
     * @brief Get whether to return to low position after move.
     * @return True if returning to low position after move, false otherwise.
     */
    bool getReturnLowAfterMove();

    /**
     * @brief Wait until the overclock mechanism is done.
     * @param msecTimeout The timeout in milliseconds.
     * @return True if done, false otherwise.
     */
    bool waitUntilDone(int msecTimeout);
private:
    bool returnLowAfterMove; /**< Whether to return to low position after move. */
    OCPos targetPos; /**< The target position. */
    bool isActive; /**< The active state. */
};

/**
 * @brief The global overclock mechanism instance.
 */
extern OverclockMech overclock_mech;

/**
 * @brief The control task for the overclock mechanism.
 * @param param The task parameter.
 */
void oc_control_task(void *param);

#endif // OVERCLOCK_MECH_H
