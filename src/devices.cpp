#include "devices.h"


// controller definition
Controller controller(pros::E_CONTROLLER_MASTER);

// motor definitions
// goes front to back
MotorGroup left_motors({-4, 13, -2}, pros::MotorGearset::blue);
MotorGroup right_motors({3,-12, 7}, pros::MotorGearset::blue);
MotorGroup all_motors({-4,13,-2,3,-12,7},pros::MotorGearset::blue);

Motor intake(1, pros::MotorGearset::blue);
Motor oc_motor(10, pros::MotorGearset::green);

Rotation ocRot(5);
PID ocPID(.7, 0, .4);

Piston oc_piston('E',false, pros::E_CONTROLLER_DIGITAL_DOWN);
Piston clamp('G', false, pros::E_CONTROLLER_DIGITAL_B);
Piston left_doinker('D',false, pros::E_CONTROLLER_DIGITAL_LEFT);
Piston right_doinker('F',false, pros::E_CONTROLLER_DIGITAL_RIGHT);
Piston redirect('H',true, pros::E_CONTROLLER_DIGITAL_L2);

Optical ringSens(0);
Optical goalSens(0);

Imu imu(6);


// * ODOMETRY
// measured from middle of wheels to middle of wheels
//const double TRACK_LENGTH = 11; // 11 inch track width (across )
//const double TRACK_WIDTH = 11.1875; // 11 inch track width (across intake)

//Rotation verticalTrackingWheel(0);




// drivetrain settings
Drivetrain drivetrain(&left_motors,               // left motor group
                      &right_motors,              // right motor group
                      11.1875,                         // 11 inch track width
                      lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                      450,                        // drivetrain rpm is 450
                      8                           // horizontal drift is 8 (center traction wheel drivebase)
);

OdomSensors sensors(nullptr, // vertical tracking wheel 1
                    nullptr, // vertical tracking wheel 2
                    nullptr, // horizontal tracking wheel 1
                    nullptr, // horizontal tracking wheel 2
                    &imu     // inertial sensor
);

// lateral PID controller
ControllerSettings lateral_controller(11,  // proportional gain (kP)
                                      0,   // integral gain (kI)
                                      6,   // derivative gain (kD)
                                      3,   // anti windup
                                      1,   // small error range, in inches
                                      100, // small error range timeout, in milliseconds
                                      2,   // large error range, in inches
                                      800, // large error range timeout, in milliseconds
                                      20   // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2,   // proportional gain (kP)
                                      0,   // integral gain (kI)
                                      10,  // derivative gain (kD)
                                      3,   // anti windup
                                      1,   // small error range, in inches
                                      100, // small error range timeout, in milliseconds
                                      3,   // large error range, in inches
                                      500, // large error range timeout, in milliseconds
                                      0    // maximum acceleration (slew)
);

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                              0,    // minimum output where drivetrain will move out of 127
                              1.019 // expo curve gain
);

// create the chassis
Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,            // odometry sensors
                        &throttle_curve     // log drive
);

// create the color sorter
ColorSort color_sort;
AutoClamp auto_clamp;