#include <cstdlib>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "color_sort.h"
#include "devices.h"

namespace Hue {
    const int RED = 0;
    const int BLUE = 210;
    const int ANY = -1;
}

namespace Ring {
    const int HUE_RANGE = 10;
    const int MIN_RING_DETECTION = 1;
    const int MAX_RING_DISTANCE = 10;
}

bool ColorSort::isActive = false;
int ColorSort::autoRedirectHue = Hue::ANY; // any means no redirect
int ColorSort::autoIntakeHue = Hue::ANY;

int ColorSort::getLastDetection(int hue) const {
    return hue == Hue::ANY ? lastAnyDetection : (hue == Hue::RED ? lastRedDetection : lastBlueDetection);
}

int ColorSort::wrapHue(int hue) {
    return (hue + 360) % 360;
}

bool ColorSort::isDetected(int hue) {

    // Define acceptable hue range based on the desired ring color
    int hueMin = wrapHue(hue - Ring::HUE_RANGE); // Wrap around to ensure valid range
    int hueMax = wrapHue(hue + Ring::HUE_RANGE); // Wrap around to ensure valid range
    int currentHue = ringSens.get_hue(); // Get the current hue value from the sensor
    int currentDist = ringSens.get_proximity(); // Get the current proximity value from the sensor

    // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees
    bool inHueRange;
    if (hueMin <= hueMax) {
        inHueRange = (currentHue >= hueMin && currentHue <= hueMax);
    } else {
        inHueRange = (currentHue >= hueMin || currentHue <= hueMax);
    }

    bool detected = inHueRange && currentDist < Ring::MAX_RING_DISTANCE;

    // Update the detection timestamps
    if (detected) {
        lastAnyDetection = pros::millis();
        if (hue == Hue::RED) {
            lastRedDetection = pros::millis();
        } else {
            lastBlueDetection = pros::millis();
        }
    }

    return detected;
}

bool ColorSort::waitUntilDetected(int msecTimeout, int hue) {
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0; // Counter for consecutive ring detections

    while(pros::millis() - startTime < msecTimeout && ringDetected < Ring::MIN_RING_DETECTION) {
        // Check for ring detection based on the specified color
        bool detected = isDetected(hue);

        if(detected) {
            // Increment the detection counter if the conditions are met
            ringDetected++;
            // Update the detection timestamps
            lastAnyDetection = pros::millis();
            if (hue == Hue::RED) {
                lastRedDetection = lastAnyDetection;
            } else {
                lastBlueDetection = lastAnyDetection;
            }
        } else {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }

        pros::delay(20); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    
    return ringDetected >= Ring::MIN_RING_DETECTION;
}

void ColorSort::toggleActive() {
    // Toggle the active state of the color sorter if auto-redirect color has been set
    
    if(autoRedirectHue == Hue::ANY) {
        pros::lcd::print(1, "WARN: AutoRedirect toggle blocked b/c color not set");
    }
    else {
        isActive = !isActive;
    }
}

// Set the auto-redirect color for the color sorter
void ColorSort::setAutoRedirect(int hue) {
    if (hue == Hue::ANY) {
        pros::lcd::print(1, "WARN: AutoRedirect disabled b/c color set to any");
        isActive = false;
        autoRedirectHue = Hue::ANY;
        autoIntakeHue = Hue::ANY;
    }
    else if (hue == Hue::RED || hue == Hue::BLUE) {
        autoRedirectHue = hue;
        autoIntakeHue = ColorSort::autoRedirectHue == Hue::RED ? Hue::BLUE : Hue::RED;
    }
    
}

int ColorSort::getRedirectHue() {
    return autoRedirectHue;
}
int ColorSort::getIntakeHue() {
    return autoIntakeHue;
}
bool ColorSort::getActive() {
    return isActive;
}

void color_sort_task(void *param){

    while(true){
        if(color_sort.getActive()){
            
            if(color_sort.isDetected(color_sort.getRedirectHue())){
                redirect.extend();
            }
            else if(color_sort.isDetected(color_sort.getIntakeHue())){
                redirect.retract();
            }
        }
        pros::delay(20);
    }

}