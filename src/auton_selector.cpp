#include "auton_selector.h"
#include "auton_routes.h"
#include "testing.h"
#include "devices.h"
#include "main.h"
#include <variant>

// Define the AutonRoutine structure
struct AutonRoutine {
    std::string displayName;
    std::variant<std::function<void()>, std::function<void(int)>> func;
    int parameter = 0;
};

// Constructor
AutonSelector::AutonSelector(const AutonRoutine* routinesArray, size_t routineCount, bool combineTesting, const AutonRoutine* extraRoutinesArray, size_t extraCount) {

    // Add main routines
    for (size_t i = 0; i < routineCount; i++) {
        if (routinesArray[i].func.index() == std::variant_npos) {
            pros::lcd::clear_line(3);
            pros::lcd::print(3, "Routine %d has null function", i);
        }
        routines.push_back(routinesArray[i]);
    }

    // Add extra routines if required
    if (combineTesting && extraRoutinesArray != nullptr) {
        for (size_t i = 0; i < extraCount; i++) {
            if (extraRoutinesArray[i].func.index() == std::variant_npos) {
                pros::lcd::clear_line(3);
                pros::lcd::print(3, "Extra routine %d has null function", i);
            }
            routines.push_back(extraRoutinesArray[i]);
        }
    }
    isRedTeam = false;
    competitionSelector.toggleDisplayTeam();
}

// Method implementations
void AutonSelector::displaySelectionBrain() {
    if (currentSelection < 0 || currentSelection >= routines.size()) {
        pros::lcd::clear_line(4);
        pros::lcd::print(4, "Invalid selection: %i", currentSelection);
        return;
    }
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "%s",routines[currentSelection].displayName.c_str());
}

void AutonSelector::prevSelection() {
    currentSelection = (currentSelection - 1) % routines.size();
}

void AutonSelector::nextSelection() {
    currentSelection = (currentSelection + 1) % routines.size();
}
void AutonSelector::toggleDisplayTeam(){
    isRedTeam = !isRedTeam;
    pros::lcd::print(1, "%s", competitionSelector.isRedTeam ? "RED" : "BLUE");
}

bool AutonSelector::setSelection(int newSelection) {
    if (newSelection < 0 || newSelection >= routines.size()) {
        currentSelection = newSelection;
        return true;
    }
    else {
        return false;
    }
}

void AutonSelector::runSelection() {
    if (currentSelection < 0 || currentSelection >= routines.size()) {
        pros::lcd::clear_line(4);
        pros::lcd::print(4, "Invalid selection: %d", currentSelection);
        return;
    }

    const AutonRoutine& selectedRoutine = routines[currentSelection];

    if (auto func = std::get_if<std::function<void()>>(&selectedRoutine.func)) {
        (*func)();
    } else if (auto func = std::get_if<std::function<void(int)>>(&selectedRoutine.func)) {
        (*func)(selectedRoutine.parameter);
    }
}

int AutonSelector::getRoutineCount() const {
    return routines.size();
}

// Global object definitions
const AutonRoutine COMPETITION_ROUTINES[] = {
    {"Prog Skills", progSkills},
    {"Safe AWP Left", safeAWP, 1},
    {"Safe AWP Right", safeAWP, 2},
    {"Ring Rush Alliance", ringRush, 1},
    {"Ring Rush Touch Mid", ringRush, 2},
    {"Ring Rush Face Corner", ringRush, 3},
    {"Ring Rush Place Corner", ringRush, 4}
};

const AutonRoutine TESTING_ROUTINES[] = {
    {"Test Ring Sensor", testRingSens}
};

const bool isTestingCombined = false;

AutonSelector competitionSelector(COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));
AutonSelector testingSelector(TESTING_ROUTINES, sizeof(TESTING_ROUTINES) / sizeof(TESTING_ROUTINES[0]), isTestingCombined, COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));

void on_left_button() {
    competitionSelector.prevSelection();
    competitionSelector.displaySelectionBrain();
}

void on_center_button() {
    competitionSelector.toggleDisplayTeam();
}

void on_right_button() {
    competitionSelector.nextSelection();
    competitionSelector.displaySelectionBrain();
}