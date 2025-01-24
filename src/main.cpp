#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <cstdlib>
#include "devices.h"
#include "auton_selector.h"
#include "testing.h"

// Global variables needed for oc control
int ocMove = NONE;
const double OC_GOAL_THRESHOLD = 3; // one-sided degree range considered "in goal"
const double OC_POSITION_LOW = 335; // position of low 
const double OC_POSITION_HIGH = 110; // position of high
const double OC_MAX_TORQUE = 999;


// Task function for oc control
// NOTE: as arm moves up, angle decreases!!
void oc_control_task(void* param)
{
    double currentPos;
    double error;
    double nextMovement;
    int goalCount = 0;

    while (true)
    {
        // pid movement for low movement
        if (ocMove == LOW)
        {
            // current position in centidegrees, convert to degrees
            currentPos = ocRot.get_angle() / 100.0;

            // normalize error to [-180,180]
            //currentPos = currentPos - 360 * (currentPos > 240) + 360 * (currentPos < -240);

            // calculate how far oc is from target
            error = currentPos - OC_POSITION_LOW;

            if (fabs(error) < OC_GOAL_THRESHOLD || oc_motor.get_torque() > OC_MAX_TORQUE)
            { // goal has been met

                goalCount++;
                if (goalCount >= 3)
                {
                    // reset PID for next usage
                    ocPID.reset();

                    // stop oc motors in place
                    oc_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    oc_motor.brake();

                    // stop running the PID code
                    ocMove = NONE;
                    goalCount = 0;
                }
            }
            else
            { // goal has not been met
                goalCount = 0;
                // determine how far to move based on PID
                nextMovement = ocPID.update(error);

                // clamps movements to [-200,200] (green gearset)
                nextMovement = std::clamp(nextMovement, -200.0, 200.0);
            }

            // move oc motors based on PID
            oc_motor.move_velocity(nextMovement);
        }
        // linear driving for high movement
        else if(ocMove == HIGH){

            currentPos = ocRot.get_angle() / 100.0;
            // starts out high, approaches 0
            error = currentPos - OC_POSITION_HIGH;

            if(error > 0){
                oc_motor.move(127);
            }
            else{
                oc_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                oc_motor.brake();
                ocMove = NONE;
            }
            
        }
        else if(ocMove == NONE){
            oc_motor.brake();
        }
        // in case there is drift
        /*else if(ocMove == IDLE_LOW){
        }*/
        
        
        //pros::lcd::print(6, "oc State: %s", ocMove != NONE ? ocMove == HIGH ? 
        //                                                   "HIGH" : "LOW" : "NONE");
        //pros::lcd::print(3, "oc Current Pos: %f", (double) currentPos);
        //pros::lcd::print(4, "oc Target Pos: %f", ocMove != NONE ? ocMove == HIGH ? 
        //                                                  OC_POSITION_HIGH : OC_POSITION_LOW : -1);
        //pros::lcd::print(7, "error: %f", error);
        //pros::lcd::print(5, "oc Next Movement: %f", nextMovement);
        
        pros::delay(20);
        
    }
}
// initialize function. Runs on program startup
void initialize()
{

    lcd::initialize();   // initialize the LCD screen on the VEX brain
    chassis.calibrate(); // Calibrates the chassis sensors to ensure accurate readings

    clamp.retract(); // Set the clamp to the high position
    oc_piston.retract(); // Set the oc piston to the low position

    ringSens.set_led_pwm(100); // Set the LED brightness to 100%
    ringSens.set_integration_time(10); // Sets the integration time for the ring sensor to 10ms

    oc_motor.set_brake_mode_all(E_MOTOR_BRAKE_COAST); // Set all motors to coast mode

    // Create a task for controlling the oc motor
    Task oc_task(oc_control_task, nullptr, "oc Control Task");
    // Create a task for outputting motor temps
    if(!competition::is_connected()){
        Task temp_task(motor_temp_task, nullptr, "Motor Temp Task");
    }
    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER); // Set the text alignment to center on the LCD screen

}

void autonomous()
{
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    competitionSelector.runSelection();
    all_motors.brake();
    delay(2000);
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

// this is a failsafe incase testing functions in opcontrol haven't been commented out
bool inCompetition = false;

void competition_initialize()
{

    inCompetition = true;
    // show current route on brain screen
    competitionSelector.displaySelectionBrain();

    // assign buttons to actions in auton selector
    lcd::register_btn0_cb(on_left_button);
    lcd::register_btn1_cb(on_center_button);
    lcd::register_btn2_cb(on_right_button);
}

const double SMOOTHING_DENOMINATOR = 100; // Used to normalize the exponential curve
const double EXPONENTIAL_POWER = 2;       // Controls how aggressive the curve is
// Helper function that makes joystick input more precise for small movements
// while maintaining full power at maximum joystick
double logDriveJoystick(double joystickPCT)
{
    // Get the absolute value for calculation
    double magnitude = fabs(joystickPCT);

    // Calculate the smoothed value
    double smoothedValue = pow(magnitude, EXPONENTIAL_POWER) / SMOOTHING_DENOMINATOR;

    // Restore the original sign (positive or negative)
    return joystickPCT >= 0 ? smoothedValue : -smoothedValue;
}

void handleDriveTrain()
{

    // get left y and right y positions
    double leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // convert to pct
    leftY /= 1.27;
    rightY /= 1.27;

    leftY = logDriveJoystick(leftY);
    rightY = logDriveJoystick(rightY);

    // convert to gearset
    leftY *= 6;
    rightY *= 6;

    left_motors.move_velocity(leftY);
    right_motors.move_velocity(rightY);
}

void handleIntake(pros::controller_digital_e_t buttonUp, pros::controller_digital_e_t buttonDown)
{
    // intake
    if (controller.get_digital(buttonUp))
    {
        intake.move(127);
    }
    // outtake
    else if (controller.get_digital(buttonDown))
    {
        intake.move(-127);
    }
    // no movement without button pressed
    else
    {
        intake.brake();
    }
    
}
bool returnOC = false; // if oc still needs to return

void handleOCMotor(pros::controller_digital_e_t button)
{
    if (controller.get_digital(button))
    {
        ocMove = HIGH;
        returnOC = true;
    }
    else if(returnOC){
        ocMove = LOW;
        returnOC = false;
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

    all_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // loop forever
    while (true)
    {
        if (!inCompetition) { testAuton(); }

        handleDriveTrain();
        handleIntake(pros::E_CONTROLLER_DIGITAL_R2,pros::E_CONTROLLER_DIGITAL_R1);
        handleOCMotor(pros::E_CONTROLLER_DIGITAL_L1);
        
        oc_piston.handle();
        clamp.handle(true);
        left_doinker.handle();
        right_doinker.handle();
        redirect.handle();
        
        // delay to save resources
        pros::delay(20);
    }
}