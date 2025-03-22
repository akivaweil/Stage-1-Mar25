#include "../../include/hardware/05_MotorControlClass.h"

// Constructor implementation
StepperMotor::StepperMotor(int stepPin, int dirPin, int enablePin, int homeSwitchPin, 
                           long maxSteps, int speed, int acceleration)
    : motor(AccelStepper::DRIVER, stepPin, dirPin),
      homeSwitchPin(homeSwitchPin),
      dirPin(dirPin),
      stepPin(stepPin),
      enablePin(enablePin),
      maxSteps(maxSteps),
      currentPosition(0),
      isEnabled(false),
      isHomed(false),
      isMoving(false),
      motorSpeed(speed),
      motorAcceleration(acceleration) {
}

// Setup motor with initial configuration
void StepperMotor::setup() {
    // Configure pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(homeSwitchPin, INPUT_PULLUP);
    
    // Disable motor initially for safety
    digitalWrite(enablePin, HIGH);
    
    // Configure motor settings
    motor.setMaxSpeed(motorSpeed);
    motor.setAcceleration(motorAcceleration);
}

// Enable the motor
void StepperMotor::enable() {
    digitalWrite(enablePin, LOW);
    isEnabled = true;
}

// Disable the motor
void StepperMotor::disable() {
    digitalWrite(enablePin, HIGH);
    isEnabled = false;
    isMoving = false;
}

// Move to absolute position
void StepperMotor::moveTo(long position) {
    // Validate position is within range
    if (position < 0) position = 0;
    if (position > maxSteps) position = maxSteps;
    
    // Only move if motor is enabled and homed
    if (isEnabled && isHomed) {
        motor.moveTo(position);
        isMoving = true;
    }
}

// Move by relative steps
void StepperMotor::moveBy(long steps) {
    // Only move if motor is enabled
    if (isEnabled) {
        // Calculate new position
        long newPosition = motor.currentPosition() + steps;
        
        // Validate the new position is within range
        if (newPosition < 0) newPosition = 0;
        if (newPosition > maxSteps) newPosition = maxSteps;
        
        // Set the new target position
        motor.moveTo(newPosition);
        isMoving = true;
    }
}

// Home the motor
bool StepperMotor::home(int homingSpeed) {
    // Only home if motor is enabled
    if (!isEnabled) {
        return false;
    }
    
    // If already at home switch, move away first
    if (isHomeSwitch()) {
        motor.setSpeed(-homingSpeed);
        
        // Move away from switch until it's released
        while (isHomeSwitch()) {
            motor.runSpeed();
            yield(); // Allow ESP32 background tasks to run
        }
        
        // Give extra clearance
        for (int i = 0; i < 100; i++) {
            motor.runSpeed();
            yield();
        }
    }
    
    // Now move toward home switch
    motor.setSpeed(homingSpeed);
    
    // Move until switch is triggered
    unsigned long startTime = millis();
    while (!isHomeSwitch()) {
        motor.runSpeed();
        yield();
        
        // Timeout if homing takes too long
        if (millis() - startTime > HOMING_TIMEOUT) {
            return false;
        }
    }
    
    // Stop the motor
    motor.setSpeed(0);
    motor.runSpeed();
    
    // Reset position to zero
    motor.setCurrentPosition(0);
    currentPosition = 0;
    isHomed = true;
    isMoving = false;
    
    return true;
}

// Update motor - call in main loop
bool StepperMotor::update() {
    if (isEnabled && isMoving) {
        // Run the motor one step if needed
        if (motor.distanceToGo() != 0) {
            motor.run();
            return true;
        } else {
            // Motor has reached target position
            isMoving = false;
            currentPosition = motor.currentPosition();
            return false;
        }
    }
    return false;
}

// Check if home switch is triggered
bool StepperMotor::isHomeSwitch() {
    // Home switch is active LOW (pulled up)
    return digitalRead(homeSwitchPin) == LOW;
}

// Check if motor is at home position
bool StepperMotor::isAtHome() {
    return isHomed && motor.currentPosition() == 0;
}

// Get current position
long StepperMotor::getPosition() {
    return motor.currentPosition();
}

// Check if motor is moving
bool StepperMotor::isMotorMoving() {
    return isMoving;
} 