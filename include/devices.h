#ifndef DEVICES_H
#define DEVICES_H

// required files for devices
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pid.hpp"
#include "extended_chassis.h"

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

extern adi::Port oc_piston;
extern adi::Port clamp;
extern adi::Port doinker;
extern adi::Port redirect;

extern Optical ringSens;
extern Optical goalSens;

extern Imu imu;

extern Drivetrain drivetrain;
extern OdomSensors sensors;
extern ControllerSettings lateral_controller;
extern ControllerSettings angular_controller;
extern ExpoDriveCurve throttle_curve;
extern ExtendedChassis chassis;

#endif // DEVICES_H