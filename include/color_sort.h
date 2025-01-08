#ifndef COLOR_SORT_H
#define COLOR_SORT_H

#include "pros/rtos.hpp"

/**
 * @class Hue
 * @brief A class to represent and manipulate hue values.
 */
class Hue {
public:
    /**
     * @brief Constructs a Hue object with a specified hue value.
     * @param hue The hue value to initialize the object with.
     */
    Hue(double hue);

    /**
     * @brief Gets the hue value.
     * @return The hue value.
     */
    double getHue() const;

    /**
     * @brief Gets the minimum hue value.
     * @return The minimum hue value.
     */
    double min() const;

    /**
     * @brief Gets the maximum hue value.
     * @return The maximum hue value.
     */
    double max() const;

    /**
     * @brief Checks if a given sensor hue is within the range of this hue.
     * @param sensorHue The hue value from the sensor to check.
     * @return True if the sensor hue is within range, false otherwise.
     */
    bool inRange(double sensorHue) const;

    /**
     * @brief Checks if this hue is equal to another hue.
     * @param hue The other Hue object to compare with.
     * @return True if the hues are equal, false otherwise.
     */
    bool equals(const Hue& hue);

    /**
     * @brief Wraps a hue value to ensure it is within the valid range.
     * @param hue The hue value to wrap.
     * @return The wrapped hue value.
     */
    static double wrapHue(double hue);

    /**
     * @brief Gets the last detection time of the hue.
     * @return The last detection time in milliseconds.
     */
    int getLastDetection();

    /**
     * @brief Sets the last detection time of the hue.
     * @param time The time to set as the last detection time. Defaults to the current time in milliseconds.
     */
    void setLastDetection(int time = pros::millis());

private:
    double hue; ///< The hue value.
    int lastDetection; ///< The last detection time in milliseconds.
};
namespace RingColor {
    /** @brief The hue value representing the color red. */
    extern Hue red;

    /** @brief The hue value representing the color blue. */
    extern Hue blue;

    /** @brief The hue value representing any color. */
    extern Hue any;
}

namespace RingConfig {
    /** @brief The range of hue values used for detecting rings. */
    extern const int HUE_RANGE;

    /** @brief The minimum threshold for detecting a ring. */
    extern const int MIN_RING_DETECTION;

    /** @brief The maximum distance at which a ring can be detected. */
    extern const int MAX_RING_DISTANCE;
}

/**
 * @class ColorSort
 * @brief A class to handle color sorting operations.
 * 
 * The ColorSort class provides methods to detect colors, enable or disable
 * the color sorting mechanism, and manage auto redirection based on detected hues.
 */
class ColorSort {
public:
    ColorSort();

    /**
     * @brief Get the last detected hue.
     * 
     * @param hue The hue to check for detection. Default is any hue.
     * @return int The last detected hue.
     */
    int getLastDetection(Hue hue = RingColor::any) const;

    /**
     * @brief Check if a specific hue is detected.
     * 
     * @param hue The hue to check for detection. Default is any hue.
     * @return true if the hue is detected.
     * @return false if the hue is not detected.
     */
    bool isDetected(Hue hue = RingColor::any);

    /**
     * @brief Wait until a specific hue is detected or timeout occurs.
     * 
     * @param msecTimeout The timeout in milliseconds.
     * @param hue The hue to wait for detection. Default is any hue.
     * @return true if the hue is detected before timeout.
     * @return false if the timeout occurs before detection.
     */
    bool waitUntilDetected(int msecTimeout, Hue hue = RingColor::any);

    /**
     * @brief Check if the color sort is enabled.
     * 
     * @return true if enabled.
     * @return false if disabled.
     */
    bool isEnabled();

    /**
     * @brief Enable the color sort.
     */
    void enable();

    /**
     * @brief Disable the color sort.
     */
    void disable();

    /**
     * @brief Set the hue for auto redirection.
     * 
     * @param hue The hue to set for auto redirection.
     */
    void setAutoRedirect(Hue hue);

    /**
     * @brief Get the hue set for auto redirection.
     * 
     * @return Hue The hue set for auto redirection.
     */
    Hue getRedirectHue();

    /**
     * @brief Get the hue set for intake.
     * 
     * @return Hue The hue set for intake.
     */
    Hue getIntakeHue();

private:
    void setActive(bool active);

    static bool isActive;
    static Hue autoRedirectHue;
    static Hue autoIntakeHue;
};

#endif // COLOR_SORT_H
