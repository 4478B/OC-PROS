#ifndef GOAL_SENSOR_H
#define GOAL_SENSOR_H


namespace Goal {
    extern const int MIN_DETECTION; // Amount of detections needed to quit loop
    extern const int MAX_DISTANCE; // Maximum distance goal is to be counted
}

class AutoClamp {
private:
    static bool isActive;
    static void setActive(bool active);
public:
    static bool isDetected();
    static void waitUntilClamp(int maxDist, int maxTime);
    static bool isGoalClamped();
    static bool isEnabled();
    static void enable();
    static void disable();
};

#endif // GOAL_SENSOR_H