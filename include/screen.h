#ifndef SCREEN_H
#define SCREEN_H

// *** USE THIS TO ADD SCREENS *** //
enum screen_state{
    OVERCLOCK,
    AUTOCLAMP,
    COLORSORT,
    EMPTY,
    TEMP1,
    TEMP2,
    TEMP3
};

namespace Screen {
    extern screen_state current_screen;
    void initializeScreenTask();
}

#endif // SCREEN_H
