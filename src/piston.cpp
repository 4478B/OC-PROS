#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdlib>
#include "devices.h"
#include "piston.h"

Piston::Piston(
    std::uint8_t port,
    bool startExtended,
    pros::controller_digital_e_t button,
    ControlType controlType)
    : pros::adi::Pneumatics(port, startExtended),
      button(button),
      controlType(controlType)
{
}
void Piston::handle(bool printing)
{
    // Get the initial status of the piston (extended or retracted)
    bool initStatus = this->is_extended();
    // Check if the button was newly pressed
    bool buttonNewPress = controller.get_digital_new_press(button);
    // Check if the button is currently pressed
    bool buttonPressed = controller.get_digital(button);

    // Handle control type TOGGLE
    if (controlType == ControlType::TOGGLE && buttonNewPress)
    {
        // Toggle piston state
        initStatus ? this->retract() : this->extend();
    }
    // Handle control type HOLD
    else if (controlType == ControlType::HOLD)
    {
        if (buttonPressed)
        {
            // If button is pressed, set piston to opposite of startExtended state
            startExtended ? this->retract() : this->extend();
        }
        else
        {
            // If button is not pressed, set piston to startExtended state
            startExtended ? this->extend() : this->retract();
        }
    }

    // Get the final status of the piston (extended or retracted)
    bool finalStatus = this->is_extended();

    // If printing is enabled and the piston state has changed
    if (printing && initStatus != finalStatus)
    {
        // Print piston state to controller
        controller.print(1, 1, finalStatus ? "XXXXXXXXXXXXXXXXXXXXXXX" : "                       ");
    }
}
