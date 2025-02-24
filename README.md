# Concurrent PID Controller Documentation

## Overview
This library provides an automated PID (Proportional-Integral-Derivative) controller for Arduino. It includes two classes:

- `PID`: A standard PID controller for single-input single-output (SISO) control.
- `ConcurrentPID`: A concurrent PID controller that enables multiple PID instances to run simultaneously.

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


---

# How to Use

## 1. Create a Custom PID Controller
Create a new controller class for each unique controller inheriting from the PID class and override input and output

```cpp
class Controller : public PID {
public:
    float inputFunction() override {
        return *input;  // Read the current speed from a sensor
    }

    void outputFunction(float outputVal) override {
        *output = outputVal;  // Send the computed output to the motor
    }
};
```
## 2. Create instances of objects
create objects for each PID controller you need to use

```cpp
Controller myController1;
Controller myController2;
Controller myController3;
```
## 3. Create instance of concurrentPID class
create object of the concurrentPID class and give it an array of pointers to each of your controllers

```cpp
PID* array[] = {&myController1, &myController2, &myController3};
concurrentPID multiController(array, 3);
```


## 4. Set variables to controllers
set variables to each of your controllers or the concurrentPID object

```cpp
multiController.setPIDvalues(10, 0.1, 1);

myController1.setPIDvalues(1.0, 0.01, 0.1);
myController1.setGoal(0);
myController1.setTolerance(1.0, 0.01, 0.1);

// Set the time required 
multiController.setTimeRequired(1);

// Set the goal of firct controller to 100
multiController.setGoal(1 ,100);
```

## 5. start() or calculate()
- The calculate function enables the controller to automatically perform the necesary calculations
- The start() function calls the claculate() function untill the goal is reached for a certain amount of time within a set tolerance

Both of these functions are available for both classes

## License
This library is open-source and can be freely used and modified under the MIT License.

## About
This library was created at the University of Cyprus Robotics CLab by me as a tool in order to be used in a maze solving robot. I thought it would be interesting if i shared I submited it to the library manager of Arduino IDE.

Creator: Christodoulos Negkoglou

<div align="center">
  <img src="Picture.jpg" alt="Logo">
</div>