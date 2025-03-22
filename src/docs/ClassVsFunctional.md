# Using Classes vs Functional Code

## Benefits of Object-Oriented Programming for ESP32 Projects

### Functional Approach (Before)
```
// Global variables
int cutMotorPin;
bool isCutMotorEnabled;

// Functions operating on globals
void initializeCutMotor() {
    pinMode(cutMotorPin, OUTPUT);
    isCutMotorEnabled = false;
}

void enableCutMotor() {
    digitalWrite(cutMotorPin, HIGH);
    isCutMotorEnabled = true;
}

void disableCutMotor() {
    digitalWrite(cutMotorPin, LOW);
    isCutMotorEnabled = false;
}

void moveCutMotor(int steps) {
    if (isCutMotorEnabled) {
        // Move motor code
    }
}
```

### Object-Oriented Approach (After)
```
class StepperMotor {
private:
    int pinStep;
    int pinDir;
    int pinEnable;
    bool isEnabled;
    
public:
    StepperMotor(int stepPin, int dirPin, int enablePin) {
        pinStep = stepPin;
        pinDir = dirPin;
        pinEnable = enablePin;
        isEnabled = false;
    }
    
    void setup() {
        pinMode(pinStep, OUTPUT);
        pinMode(pinDir, OUTPUT);
        pinMode(pinEnable, OUTPUT);
    }
    
    void enable() {
        digitalWrite(pinEnable, HIGH);
        isEnabled = true;
    }
    
    void disable() {
        digitalWrite(pinEnable, LOW);
        isEnabled = false;
    }
    
    void moveSteps(int steps) {
        if (isEnabled) {
            // Move motor code
        }
    }
};

// Usage
StepperMotor cutMotor(CUT_STEP_PIN, CUT_DIR_PIN, CUT_ENABLE_PIN);
StepperMotor positionMotor(POS_STEP_PIN, POS_DIR_PIN, POS_ENABLE_PIN);
```

## Advantages of Using Classes

1. **Encapsulation**
   - Data and functions that operate on that data are bundled together
   - State variables are protected from accidental changes
   - Implementation details are hidden from other parts of the code

2. **Reusability**
   - Class can be instantiated multiple times for similar hardware
   - Same code works for different motors, sensors, etc.
   - Reduces code duplication

3. **Modularity**
   - Classes create clean boundaries between components
   - Makes it easier to understand how components interact
   - Allows for focused testing and debugging

4. **Maintainability**
   - Changes to one component don't affect others
   - Easier to read and understand the code
   - New developers can grasp the system structure more quickly

5. **Object Lifecycle Management**
   - Constructor ensures proper initialization
   - Destructor can handle cleanup (less relevant for Arduino)
   - Class methods ensure state consistency

## Practical Impact

In our project, moving to class-based architecture for hardware components:

1. Replaces scattered global variables with contained class members
2. Organizes related functionality into logical units
3. Makes the relationship between hardware and software clearer
4. Creates a more scalable structure for adding new features

## Beginner Tips

When working with classes:
1. Start by identifying what "things" your system contains (motors, sensors, LEDs)
2. Think about what data each thing needs to keep track of
3. Define what actions each thing can perform
4. Create a class with private member variables and public methods
5. Use descriptive method names that explain what the action does
6. Keep methods small and focused on one task 