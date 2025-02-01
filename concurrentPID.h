//this is an automated pid controller
//you can use the PID object for a simple pid controller or
//use the concurent pid controller object to create multiple that run simultaniousely
//since the start function stops the program input and output functions for each pid controller must be overriden

#ifndef CONCURRENTPID_H
#define CONCURRENTPID_H

#include <Arduino.h>

class PID {
public:
    
    void setPIDvalues(float kp, float ki, float kd);  //set PID values
    void setTolerance(float tolerance); //set accepted tolarance for error
    void setTimeRequired(long time); //set time that pid controller needs to stay below tolarance in order for it to be completed
    void setGoal(float goal);

    // These methods can be overridden by subclasses
    virtual float inputFunction() { return *input; }
    virtual void outputFunction(float output) { *this->output = output; }

    void calculate(); //use pid equation to calculate output (calls input/output functions)

    void start(); //start a loop that uses calculate function until the pid controller finishes

    bool goalReached(); //check if the error is below tolarance

    bool goalReachedForT(); //check if the error is below tolarance for the required time

    float kp, ki, kd; //PID constants
    

protected:
    float tolerance = 0.0f;
    float goal = 0.0f;
    float* input = nullptr;
    float* output = nullptr;
    unsigned long timeRequired = 0;
    unsigned long timeBelowTolerance = 0;
    unsigned long startTime = millis();
    unsigned long prevT = millis();
    float previousProportional = 0;
    float integral = 0;
};

class ConcurrentPID {
public:
    ConcurrentPID(PID* vec[], int numberOfPIDs);
    ~ConcurrentPID();

    void setPIDvalues(float kp, float ki, float kd);  //for all values
    void setTolerance(float tolarance);
    void setTimeRequired(long time);
    void setGoal(float goal);

    void setPIDvalues(int pidNumber, float kp, float ki, float kd);  //for single object
    void setTolerance(int pidNumber, float tolarance);
    void setTimeRequired(int pidNumber, long time);
    void setGoal(int pidNumber, float goal);

    void start(); //start a loop that enables all pid controllers concurrently

private:
    PID** PIDmatrix = nullptr;  // Array of pointers to PID objects
    int numPIDs;
};

#endif // CONCURRENTPID_H

