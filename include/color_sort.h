#ifndef COLOR_SORT_H
#define COLOR_SORT_H

#include "pros/rtos.hpp"
class Hue {
public:
    static int lastDetection;
    static int wrapHue(int hue);
    Hue(int hue);
    int getHue() const;
    int min() const;
    int max() const;
    bool inRange(int sensorHue) const;
    static int getLastDetection();
    static void setLastDetection(int time = pros::millis());
    bool equals(const Hue& hue);
private:
    int hue;
};
namespace RingColor {
    extern Hue red;
    extern Hue blue;
    extern Hue any;
}

namespace RingConfig {
    extern const int HUE_RANGE;
    extern const int MIN_RING_DETECTION;
    extern const int MAX_RING_DISTANCE;
}

class ColorSort {
private:
    static bool isActive;
    static Hue autoRedirectHue;
    static Hue autoIntakeHue;
    void setActive(bool active);
public:
    ColorSort();
    int getLastDetection(Hue hue = RingColor::any) const;
    bool isDetected(Hue hue = RingColor::any);
    bool waitUntilDetected(int msecTimeout, Hue hue = RingColor::any);
    bool isEnabled();
    void enable();
    void disable();
    void setAutoRedirect(Hue hue);
    Hue getRedirectHue();
    Hue getIntakeHue();

};

#endif // COLOR_SORT_H
