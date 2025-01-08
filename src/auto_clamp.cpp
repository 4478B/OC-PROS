#include "auto_clamp.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdlib>
#include "devices.h"
#include "screen.h"

namespace Goal {
    const int MIN_DETECTION = 3; // Amount of detections needed to quit loop
    const int MAX_DISTANCE = 10; // Maximum distance goal is to be counted
}

bool AutoClamp::isActive = false;

bool AutoClamp::isDetected() {
    int currentGoalDist = 255 - goalSens.get_proximity(); // Get the current distance from the sensor
    bool inRange = currentGoalDist <= Goal::MAX_DISTANCE; // Determine if goal is within proximity
    return inRange;
}

void AutoClamp::waitUntilClamp(int maxDist, int maxTime) {
    int startTime = pros::millis(); // Record the start time of the function
    int goalDetected = 0; // Counter for consecutive goal detections

    // Ensure clamp is up
    clamp.retract();

    // Set up motor for distance tracking
    left_motors.tare_position(0);

    // Convert maxDist from inches to rotations
    double maxDistRotations = maxDist / (lemlib::Omniwheel::NEW_275 * M_PI);

    if(Screen::current_screen == screen_state::TEMP1){
        pros::lcd::print(1, "Max Distance (inches): %d", maxDist);
        pros::lcd::print(2, "Max Time: %d ms", maxTime);
    }
    

    // Loop until goal is detected enough times or it times out
    while (pros::millis() - startTime < maxTime && goalDetected < Goal::MIN_DETECTION && left_motors.get_position(0) > -maxDistRotations) {
        bool detected = isDetected();

        if (detected) {
            // Increment the detection counter if conditions are met
            goalDetected++;
        } else {
            // Reset the detection counter if the conditions are not met
            goalDetected = 0;
        }

        if(Screen::current_screen == screen_state::TEMP1){
            double currentGoalDistInches = (255 - goalSens.get_proximity()) * (lemlib::Omniwheel::NEW_275 * M_PI);
            pros::lcd::print(3, "Current Goal Distance (inches): %f", currentGoalDistInches);
            pros::lcd::print(4, "Goal Detected Count: %d", goalDetected);
            pros::lcd::print(5, "Left Motor Position (inches): %f", left_motors.get_position(0) * (lemlib::Omniwheel::NEW_275 * M_PI));
        }

        pros::delay(20);
    }

    // Clamp goal
    clamp.extend();
}

bool AutoClamp::isGoalClamped() {
    // If goal is detected and clamp is down
    return (isDetected() && clamp.is_extended());
}

bool AutoClamp::isEnabled() {
    return isActive;
}

void AutoClamp::setActive(bool active) {
    isActive = active;
}

void AutoClamp::enable() {
    setActive(true);
}
void AutoClamp::disable() {
    setActive(false);
}

void auto_clamp_task(void *param)
{
    int goalDetected = 0; // Counter for consecutive goal detections

    // Loop forever
    while (true)
    {
        if(auto_clamp.isEnabled() && !clamp.is_extended())
        {

            // Check if goal is detected
            bool detected = AutoClamp::isDetected();

            // If goal is detected
            if (detected)
            {
                // Increment the detection counter if conditions are met
                goalDetected++;
            }
            else
            {
                // Reset the detection counter if the conditions are not met
                goalDetected = 0;
                // Retract the clamp if goal is not detected and clamp is down
                clamp.retract();
            }

            // If goal is detected enough times and clamp is up
            if (goalDetected >= Goal::MIN_DETECTION && !clamp.is_extended())
            {
                // Extend the clamp
                clamp.extend();
            }
        }

        // Delay to save resources
        pros::delay(20);
        
    }
}    

