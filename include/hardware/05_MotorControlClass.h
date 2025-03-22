#ifndef MOTOR_CONTROL_CLASS_H
#define MOTOR_CONTROL_CLASS_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "../core/02_PinDefinitions.h"
#include "../core/04_Config.h"

/**
 * StepperMotor - A class that encapsulates stepper motor functionality
 * 
 * This class provides a clean interface for controlling a stepper motor,
 * including movement, homing, and status reporting.
 */
class StepperMotor {
private:
    AccelStepper motor;
    int homeSwitchPin;
    int dirPin;
    int stepPin;
    int enablePin;
    
    long maxSteps;
    long currentPosition;
    bool isEnabled;
    bool isHomed;
    bool isMoving;
    int motorSpeed;
    int motorAcceleration;
    
public:
    /**
     * Constructor - Initialize a new stepper motor
     * 
     * @param stepPin      The pin that triggers steps
     * @param dirPin       The pin that controls direction
     * @param enablePin    The pin that enables/disables the motor
     * @param homeSwitchPin The pin for the home switch
     * @param maxSteps     Maximum steps from home position
     * @param speed        Default speed in steps per second
     * @param acceleration Acceleration in steps per second squared
     */
    StepperMotor(int stepPin, int dirPin, int enablePin, int homeSwitchPin, 
                 long maxSteps, int speed, int acceleration);
    
    /**
     * Setup the motor with initial configuration
     */
    void setup();
    
    /**
     * Enable the motor
     */
    void enable();
    
    /**
     * Disable the motor
     */
    void disable();
    
    /**
     * Move the motor to a specific position
     * 
     * @param position Target position in steps
     */
    void moveTo(long position);
    
    /**
     * Move the motor by a relative number of steps
     * 
     * @param steps Number of steps to move (positive or negative)
     */
    void moveBy(long steps);
    
    /**
     * Home the motor by moving until the home switch is triggered
     * 
     * @param homingSpeed Speed to use during homing
     * @return True if homing was successful
     */
    bool home(int homingSpeed);
    
    /**
     * Update motor - Call this function in the main loop
     * 
     * @return True if the motor is still moving
     */
    bool update();
    
    /**
     * Check if the home switch is triggered
     * 
     * @return True if home switch is triggered
     */
    bool isHomeSwitch();
    
    /**
     * Check if the motor is at the home position
     * 
     * @return True if the motor is homed
     */
    bool isAtHome();
    
    /**
     * Get the current position of the motor
     * 
     * @return Current position in steps
     */
    long getPosition();
    
    /**
     * Check if the motor is currently moving
     * 
     * @return True if the motor is moving
     */
    bool isMotorMoving();
};

#endif // MOTOR_CONTROL_CLASS_H 