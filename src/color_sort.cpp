#include <cstdlib>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "color_sort.h"
#include "devices.h"

double Hue::wrapHue(double hue) {
    return fmod(hue + 360.0, 360.0);
}
Hue::Hue(double hue) : hue(hue), lastDetection(pros::millis()) {}
double Hue::getHue() const {
    return hue;
}
double Hue::min() const {
    return wrapHue(hue - RingConfig::HUE_RANGE);
}
double Hue::max() const {
    return wrapHue(hue + RingConfig::HUE_RANGE);
}
bool Hue::inRange(double sensorHue) const {
    if (min() <= max()) {
        return (sensorHue >= min() && sensorHue <= max());
    } else {
        return (sensorHue >= min() || sensorHue <= max());
    }
}
int Hue::getLastDetection() {
    return lastDetection;
}
void Hue::setLastDetection(int time) {
    lastDetection = time;
}
bool Hue::equals(const Hue& hue) {
    return this->hue == hue.getHue();
}

namespace RingColor {
    Hue red(0.0);
    Hue blue(210.0);
    Hue any(-1.0);
}

namespace RingConfig {
    const int HUE_RANGE = 10;
    const int MIN_RING_DETECTION = 1;
    const int MAX_RING_DISTANCE = 10;
}

bool ColorSort::isActive = false;
Hue ColorSort::autoRedirectHue = RingColor::any; // any means no redirect
Hue ColorSort::autoIntakeHue = RingColor::any;

int ColorSort::getLastDetection(Hue hue) const {
    return hue.getLastDetection();
}
bool ColorSort::isDetected(Hue hue) {


    double currentHue = ringSens.get_hue(); // Get the current hue value from the sensor
    int currentDist = ringSens.get_proximity(); // Get the current proximity value from the sensor

    // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees
    bool inHueRange = hue.inRange(currentHue);

    bool detected = inHueRange && currentDist < RingConfig::MAX_RING_DISTANCE;

    // Update the detection timestamps
    if (detected) {
        int detectionTime = pros::millis();
        RingColor::any.setLastDetection(detectionTime);
        if(hue.equals(RingColor::red)) {
            RingColor::red.setLastDetection(detectionTime);
        } 
        else {
            RingColor::blue.setLastDetection(detectionTime);
        }
    }
    return detected;
}

bool ColorSort::waitUntilDetected(int msecTimeout, Hue hue) {
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0; // Counter for consecutive ring detections

    while(pros::millis() - startTime < msecTimeout && ringDetected < RingConfig::MIN_RING_DETECTION) {
        // Check for ring detection based on the specified color
        bool detected = isDetected(hue);

        if(detected) {
            // Increment the detection counter if the conditions are met
            ringDetected++;
            // Update the detection timestamps
            int detectionTime = pros::millis();
            RingColor::any.setLastDetection(detectionTime);
            if(hue.equals(RingColor::red)) {
                RingColor::red.setLastDetection(detectionTime);
            } else {
                RingColor::blue.setLastDetection(detectionTime);
            }
        } 
        else {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }

        pros::delay(20); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    
    return ringDetected >= RingConfig::MIN_RING_DETECTION;
}

void ColorSort::setActive(bool active) {
    if(autoRedirectHue.equals(RingColor::any) && active != isEnabled()) {
        pros::lcd::print(1, "WARN: AutoRedirect toggle blocked b/c color not set");
    }
    else {
        isActive = active;
    }
}

void ColorSort::enable() {
    setActive(true);
}
void ColorSort::disable() {
    setActive(false);
}

// Set the auto-redirect color for the color sorter
void ColorSort::setAutoRedirect(Hue hue) {
    if (hue.equals(RingColor::any)) {
        pros::lcd::print(1, "WARN: AutoRedirect disabled b/c color set to any");
        isActive = false;
        autoRedirectHue = RingColor::any;
        autoIntakeHue = RingColor::any;
    }
    else if (hue.equals(RingColor::red) || hue.equals(RingColor::blue)) {
        autoRedirectHue = hue;
        autoIntakeHue = ColorSort::autoRedirectHue.equals(RingColor::red) ? RingColor::blue : RingColor::red;
    }
    
}

Hue ColorSort::getRedirectHue() {
    return autoRedirectHue;
}
Hue ColorSort::getIntakeHue() {
    return autoIntakeHue;
}
bool ColorSort::isEnabled() {
    return isActive;
}

// Task for controlling the color sorter
void color_sort_task(void *param) {
    while (true) {
        // Check if the color sorter is enabled
        if (color_sort.isEnabled()) {
            // Check if the detected color matches the auto-redirect hue
            if (color_sort.isDetected(color_sort.getRedirectHue())) {
                // Extend the redirect mechanism if the redirect hue is detected
                redirect.extend();
            }
            // Check if the detected color matches the auto-intake hue
            else if (color_sort.isDetected(color_sort.getIntakeHue())) {
                // Retract the redirect mechanism if the intake hue is detected
                redirect.retract();
            }
        }
        // Wait briefly before the next iteration to prevent excessive polling
        pros::delay(20);
    }
}