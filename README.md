# Concurrent PID Controller Documentation

## Overview
This library provides an automated PID (Proportional-Integral-Derivative) controller for Arduino. It includes two classes:

- `PID`: A standard PID controller for single-input single-output (SISO) control.
- `ConcurrentPID`: A concurrent PID controller that enables multiple PID instances to run simultaneously.

## Features
- Customizable PID parameters (`Kp`, `Ki`, `Kd`)
- Adjustable tolerance for error
- Configurable goal and required time for goal stabilization
- Overridable input and output functions
- Support for concurrent PID execution

---

## `PID` Class

### Methods

### `void setPIDvalues(float kp, float ki, float kd)`
Sets the proportional (`Kp`), integral (`Ki`), and derivative (`Kd`) gains of the PID controller.

### `void setTolerance(float tolerance)`
Sets the acceptable error margin within which the goal is considered reached.

### `void setTimeRequired(long time)`
Defines the duration (in milliseconds) that the error must remain within tolerance for the PID controller to complete.

### `void setGoal(float goal)`
Sets the target value for the PID controller.

### `virtual float inputFunction()`
A function that returns the input value. Can be overridden in a subclass.

### `virtual void outputFunction(float output)`
A function that outputs the calculated PID response. Can be overridden in a subclass.

### `void calculate()`
Computes the PID output based on the current input, goal, and PID parameters.

### `void start()`
Starts a loop that continuously runs the `calculate()` function until the goal is reached.

### `bool goalReached()`
Checks if the current error is within the specified tolerance.

### `bool goalReachedForT()`
Checks if the error has remained within tolerance for the required duration.

### Attributes
- `float kp, ki, kd`: PID gain values
- `float tolerance`: Acceptable error margin
- `float goal`: Target value for the PID
- `float* input`: Pointer to input variable
- `float* output`: Pointer to output variable
- `unsigned long timeRequired`: Time required for goal stability
- `unsigned long timeBelowTolerance`: Time spent within tolerance
- `unsigned long startTime`: Timestamp of PID start
- `unsigned long prevT`: Previous timestamp for calculations
- `float previousProportional`: Previous proportional error
- `float integral`: Accumulated integral error

---

## `ConcurrentPID` Class

### Constructor & Destructor

### `ConcurrentPID(PID* vec[], int numberOfPIDs)`
Initializes a concurrent PID controller with an array of `PID` objects.

### `~ConcurrentPID()`
Destructor to free allocated resources.

### Methods

### `void setPIDvalues(float kp, float ki, float kd)`
Sets PID values (`Kp`, `Ki`, `Kd`) for all PID controllers.

### `void setTolerance(float tolerance)`
Sets the tolerance for all PID controllers.

### `void setTimeRequired(long time)`
Sets the required time for all PID controllers.

### `void setGoal(float goal)`
Sets the goal for all PID controllers.

### `void setPIDvalues(int pidNumber, float kp, float ki, float kd)`
Sets PID values (`Kp`, `Ki`, `Kd`) for a specific PID controller.

### `void setTolerance(int pidNumber, float tolerance)`
Sets the tolerance for a specific PID controller.

### `void setTimeRequired(int pidNumber, long time)`
Sets the required time for a specific PID controller.

### `void setGoal(int pidNumber, float goal)`
Sets the goal for a specific PID controller.

### `void start()`
Starts a loop that enables all PID controllers to run concurrently.

### Attributes
- `PID** PIDmatrix`: Array of pointers to `PID` objects
- `int numPIDs`: Number of PID controllers in the system
- 
---

## Notes
- The `start()` function blocks execution, so input and output functions must be overridden to prevent program halt.
- Ensure correct initialization of PID objects before using them in `ConcurrentPID`.

## License
This library is open-source and can be freely used and modified under the MIT License.

## About
This library was created at the University of Cyprus Robotics CLab by me as a tool in order to be used in a maze solving robot. I thought it would be interesting if i shared I submited it to the library manager of Arduino IDE.
