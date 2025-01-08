#ifndef DEVICES_H
#define DEVICES_H

// required files for devices
#include "main.h"
#include "piston.h"
#include "lemlib/pid.hpp"
#include "color_sort.h"
#include "auto_clamp.h"
#include "lemlib/chassis/chassis.hpp"

// namespace for declarations
using namespace pros;
using namespace lemlib;

extern Controller controller;

extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern MotorGroup all_motors;

extern Motor intake;
extern Motor oc_motor;

extern Rotation ocRot;
extern PID ocPID;

extern Piston oc_piston;
extern Piston clamp;
extern Piston left_doinker;
extern Piston right_doinker;
extern Piston redirect;

extern Optical ringSens;
extern Optical goalSens;

extern Imu imu;

extern Drivetrain drivetrain;
extern OdomSensors sensors;
extern ControllerSettings lateral_controller;
extern ControllerSettings angular_controller;
extern ExpoDriveCurve throttle_curve;
extern Chassis chassis;
extern ColorSort color_sort;
extern AutoClamp auto_clamp;

#endif // DEVICES_H