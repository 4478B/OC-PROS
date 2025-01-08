#ifndef COLOR_SORT_H
#define COLOR_SORT_H

namespace Hue {
    extern const int RED;
    extern const int BLUE;
    extern const int ANY;
}

namespace Ring {
    extern const int HUE_RANGE;
    extern const int MIN_RING_DETECTION;
    extern const int MAX_RING_DISTANCE;
}

class ColorSort {
private:
    int lastRedDetection;
    int lastBlueDetection;
    int lastAnyDetection;
    int wrapHue(int hue);

public:
    static bool isActive;
    static int autoRedirectHue;
    static int autoIntakeHue;
    ColorSort();
    int getLastDetection(int hue = Hue::ANY) const;
    bool isDetected(int hue = Hue::ANY);
    bool waitUntilDetected(int msecTimeout, int hue = Hue::ANY);
    void toggleActive();
    void setAutoRedirect(int hue);
};

#endif // COLOR_SORT_H
