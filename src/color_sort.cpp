#include <cstdlib>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "color_sort.h"
#include "devices.h"


// exact values of hues to detect
const int RED_RING_HUE = 0;
const int BLUE_RING_HUE = 210;

// one sided hue range that is considered close enough
const int HUE_RANGE = 10;

const int MIN_RING_DETECTION = 1; // Amount of detections needed to quit loop
const int MAX_RING_DISTANCE = 10; // maximum distance ring is on intake from optical sensor

// global setter for color sort detector
bool isRedAlliance = true;

/*

AUTONOMOUS

*/

bool waitUntilRingDetected(int msecTimeout, int targetHue)
{
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0;           // Counter for consecutive ring detections

    // Define acceptable hue range based on the desired ring color
    int hueMin, hueMax;

    // set target hue to correct team
    if (targetHue == -1)
    {
        // detecting any hue if no team is specified
        hueMin = 0;
        hueMax = 360;
    }
    else
    {
        // detecting specific hue
        hueMin = (targetHue - HUE_RANGE + 360) % 360; // Wrap around to ensure valid range
        hueMax = (targetHue + HUE_RANGE) % 360;       // Wrap around to ensure valid range
    }

    ringSens.set_led_pwm(100); // Set the LED brightness to maximum for better detection

    while (pros::millis() - startTime < msecTimeout && ringDetected < MIN_RING_DETECTION)
    {
        // Calculate elapsed time and check if detection target is met

        int currentHue = ringSens.get_hue();        // Get the current hue value from the sensor
        int currentDist = ringSens.get_proximity(); // Get the current proximity value from the sensor

        // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees

        bool inHueRange;
        if (targetHue == -1)
        {
            inHueRange = true;
        }
        else
        {
            inHueRange = (hueMin <= hueMax) ? (currentHue >= hueMin && currentHue <= hueMax) : (currentHue >= hueMin || currentHue <= hueMax);
        }

        bool inDist = currentDist > 255 - MAX_RING_DISTANCE;
        if ((inHueRange || targetHue == -1) && inDist)
        {
            // Increment the detection counter if the conditions are met
            ringDetected++;
        }
        else
        {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }

        // Print debug information
        pros::lcd::print(2, "Sensor hue %f", ringSens.get_hue());
        pros::lcd::print(3, "Sensor dist: %i", ringSens.get_proximity());
        pros::lcd::print(4, "Detections: %i", ringDetected);

        // pros::lcd::print(4, "Error: %s", strerror(ringSens.get_proximity()));

        pros::delay(20); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    ringSens.set_led_pwm(0);
    if (ringDetected >= MIN_RING_DETECTION)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool waitUntilRedIntake(int timeout)
{
    // Wait until red rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, RED_RING_HUE);
}

bool waitUntilBlueIntake(int timeout)
{
    // Wait until blue rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, BLUE_RING_HUE);
}

bool waitUntilAnyIntake(int timeout)
{
    // Wait until blue rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, -1);
}

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

ColorSort::ColorSort(){}

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

// Display all the information about the colorsort mechanism on the LCD screen on lines 1-7
void color_sort_screen_task(void *param) {
    while (true) {
        pros::lcd::print(1, "Color Sort Active: %s", color_sort.isEnabled() ? "True" : "False");
        pros::lcd::print(2, "Auto Redirect Hue: %f", color_sort.getRedirectHue().getHue());
        pros::lcd::print(3, "Auto Intake Hue: %f", color_sort.getIntakeHue().getHue());
        pros::lcd::print(4, "Current Hue: %f", ringSens.get_hue());
        pros::lcd::print(5, "Current Proximity: %d", ringSens.get_proximity());
        pros::lcd::print(6, "Last Detection: %d", color_sort.getLastDetection(RingColor::any));
        pros::lcd::print(7, "Last Red Detection: %d", color_sort.getLastDetection(RingColor::red));
        pros::delay(100);
    }
}
