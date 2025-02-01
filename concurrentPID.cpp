#include "concurrentPID.h"

//***************************PID class***********************//

void PID::setPIDvalues(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::setTolerance(float tolerance){
    this->tolerance = tolerance;
}

void PID::setTimeRequired(long time){
    this->timeRequired = time;
}

void PID::setGoal(float goal) {
    this->goal = goal;
}

bool PID::goalReached() {
    // Check if the input (current state) is within tolerance of the goal
    return fabs(goal - inputFunction()) <= tolerance;
}

bool PID::goalReachedForT() {
    // Ensure the error is below tolerance for the required time
    return goalReached() && (timeBelowTolerance >= timeRequired);
}

void PID::calculate() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevT) / 1000.0f;

    float input = inputFunction();

    float proportional = goal - input;
    float derivative = (proportional - previousProportional) / dt;
    integral += proportional * dt;

    float output = kp * proportional + kd * derivative + ki * integral;

    outputFunction(output);

    prevT = currentTime;
    previousProportional = proportional;

    // Time tracking logic
    if (goalReached()) {
        if (timeBelowTolerance == 0) {
            startTime = currentTime;
        }
        timeBelowTolerance = currentTime - startTime;
    } else {
        timeBelowTolerance = 0;
    }
}

void PID::start() {
    while (!goalReachedForT()) {
        calculate();
        delay(10); // Small delay to avoid maxing out CPU
    }
}

//******************************ConcurrentPID class***************************//

ConcurrentPID::ConcurrentPID(PID* vec[], int numberOfPIDs) : numPIDs(numberOfPIDs) {
    PIDmatrix = new PID*[numPIDs];
    for (int i = 0; i < numPIDs; i++) {
        PIDmatrix[i] = vec[i];
    }
}

ConcurrentPID::~ConcurrentPID() {
    delete[] PIDmatrix;
}

void ConcurrentPID::setPIDvalues(float kp, float ki, float kd) {
    for (int PIDnum = 0; PIDnum < numPIDs; PIDnum++) {
        PIDmatrix[PIDnum]->setPIDvalues(kp, ki, kd);
    }
}

void ConcurrentPID::setPIDvalues(int PIDnum, float kp, float ki, float kd) {
    if (PIDnum >= 0 && PIDnum < numPIDs) {
        PIDmatrix[PIDnum]->setPIDvalues(kp, ki, kd);
    }
}

void ConcurrentPID::setTolerance(float tolerance) {
    for (int PIDnum = 0; PIDnum < numPIDs; PIDnum++) {
        PIDmatrix[PIDnum]->setTolerance(tolerance);
    }
}

void ConcurrentPID::setTolerance(int PIDnum, float tolerance) {
    if (PIDnum >= 0 && PIDnum < numPIDs) {
        PIDmatrix[PIDnum]->setTolerance(tolerance);
    }
}

void ConcurrentPID::setTimeRequired(long time) {
    for (int PIDnum = 0; PIDnum < numPIDs; PIDnum++) {
        PIDmatrix[PIDnum]->setTimeRequired(time);
    }
}

void ConcurrentPID::setTimeRequired(int PIDnum, long time) {
    if (PIDnum >= 0 && PIDnum < numPIDs) {
        PIDmatrix[PIDnum]->setTimeRequired(time);
    }
}

void ConcurrentPID::setGoal(float goal) {
    for (int PIDnum = 0; PIDnum < numPIDs; PIDnum++) {
        PIDmatrix[PIDnum]->setGoal(goal);
    }
}

void ConcurrentPID::setGoal(int PIDnum, float goal) {
    if (PIDnum >= 0 && PIDnum < numPIDs) {
        PIDmatrix[PIDnum]->setGoal(goal);
    }
}

void ConcurrentPID::start() {
    while (true) {
        bool allDone = true;

        for (int i = 0; i < numPIDs; i++) {
            PIDmatrix[i]->calculate();

            if (!PIDmatrix[i]->goalReachedForT()) {
                allDone = false;
            }
        }

        if (allDone) {
            break;  // Exit if all PIDs are within tolerance
        }

        delay(10);  // Small delay to avoid maxing out CPU
    }
}




