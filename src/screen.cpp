#include "main.h"
#include "devices.h"
#include "screen.h"

// Display all the information about the overclock mechanism on the LCD screen on lines 1-7
void oc_screen_task(void *param) {
    while (true) {
        pros::lcd::print(1, "OC Pos: %d", overclock_mech.getTargetPos().getAngle());
        pros::lcd::print(2, "OC Current: %f", overclock_mech.getCurrentPos());
        pros::lcd::print(3, "OC Active: %d", overclock_mech.getIsActive());
        pros::lcd::print(4, "OC Return Low: %d", overclock_mech.getReturnLowAfterMove());
        pros::lcd::print(5, "OC Brake Mode: %d", overclock_mech.getTargetPos().getBrakeMode());
        pros::lcd::print(6, "OC Using PID: %d", overclock_mech.getTargetPos().getIsUsingPID());
        pros::lcd::print(7, "OC Direction: %d", overclock_mech.getTargetPos().getAngularDirection());
        pros::delay(100);
    }
}

// Display all the information about the autoclamp mechanism on the LCD screen on lines 1-7
void auto_clamp_screen_task(void *param) {
    while (true) {
        pros::lcd::print(1, "Goal Detected: %s", auto_clamp.isDetected() ? "True" : "False");
        pros::lcd::print(2, "Goal Clamped: %s", auto_clamp.isGoalClamped() ? "True" : "False");
        pros::lcd::print(3, "Auto Clamp Enabled: %s", auto_clamp.isEnabled() ? "True" : "False");
        pros::lcd::print(4, "Clamp State: %s", clamp.is_extended() ? "Extended" : "Retracted");
        pros::delay(100);
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

struct screen_state_data{
    screen_state state;
    std::function<void(void *param)> func;
};

// *** USE THIS TO ADD SCREENS WITH TASKS *** //
screen_state_data screen_states[] = {
    {OVERCLOCK, oc_screen_task},
    {AUTOCLAMP, auto_clamp_screen_task},
    {COLORSORT, color_sort_screen_task},
};
// *** USE THIS TO CHANGE THE ACTIVE SCREEN *** //
screen_state Screen::current_screen = EMPTY;
// REMEMBER TO ADD NEW SCREENS TO THE ENUM IN screen.h //
void Screen::initializeScreenTask() {
    
    // see if screen_state has associated task
    for(int i = 0; i < sizeof(screen_states)/sizeof(screen_state_data); i++){
        if(screen_states[i].state == Screen::current_screen){
            pros::Task screen_task(screen_states[i].func, "Screen Task");
            break;
        }
    }
}