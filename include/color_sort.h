#ifndef COLOR_SORT_H
#define COLOR_SORT_H

namespace Hue {
    extern const int RED;
    extern const int BLUE;
    extern const int ANY;
}
class ColorSort {
public:
    const int HUE_RANGE = 10;
    const int MIN_RING_DETECTION = 1;
    const int MAX_RING_DISTANCE = 10;

private:
    int lastRedDetection;
    int lastBlueDetection;
    int lastAnyDetection;
    int wrapHue(int hue);

public:
    ColorSort();
    int getLastDetection(int hue = Hue::ANY) const;
    bool isDetected(int hue = Hue::ANY);
    bool waitUntilDetected(int msecTimeout, int hue = Hue::ANY);
};

#endif // COLOR_SORT_H
