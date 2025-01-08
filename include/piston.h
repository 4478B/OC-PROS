#ifndef PISTON_H
#define PISTON_H
#include "pros/adi.hpp"
#include "pros/misc.h"


enum ControlType {
    TOGGLE,
    HOLD
};

class Piston : public pros::adi::Pneumatics {
public:
    Piston(
      std::uint8_t port,
      bool startExtended,
      pros::controller_digital_e_t button,
      ControlType controlType = ControlType::TOGGLE);

    // Forward declare methods to add to Pneumatics class
    void handle(bool printing = false);

private:
    pros::controller_digital_e_t button;
    ControlType controlType;
    bool startExtended;
};
#endif // PISTON_H