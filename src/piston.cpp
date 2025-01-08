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
    this->button = button;
    this->controlType = controlType;
}

void Piston::handle(bool printing)
{
    // if corresponding button is pressed
    if (controller.get_digital_new_press(button) && controlType == ControlType::TOGGLE)
    {
        // toggle piston state
        this->is_extended() ? this->retract() : this->extend();
    }
    else if (controller.get_digital(button) && controlType == ControlType::HOLD)
    {
        // extend piston
        this->extend();
    }
    else if (controlType == ControlType::HOLD)
    {
        // retract piston
        this->retract();
    
    }

    // print piston state with name of object to controller
    if (printing)
    {
        pros::lcd::print(1, "Piston %d: %s", this->get_port(), this->is_extended() ? "XXXXXXXXXXXXXXXXXXXXXXX" : "                       ");
    }
}