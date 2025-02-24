#include <concurrentPID.h>

// Defining the pins for motor, encoder, led, potentiometer and photoresistor
int ENA = 11;
int IN1 = 10;
int IN2 = 9;
int LEDpin = 6;
int PotPin = A0;
int ENCA = 2;
int ENCB = 3;
int RphotoPin = A1;

int brightness = 0;
volatile int Position = 0;

void interruptFunc(){

  if (digitalRead(ENCB) == 0)
    Position++;
  else
    Position--;
  
}

 // create a speed controller class by inheriting from PID
class speedController : public PID{
  
  // Override input function to use input we want (Encoders)
  float inputFunction() override {
    return Position;
  }

  
  // Override output function to return the output we want (Speed)
  void outputFunction(float output) override {
    
    if (output > 0)
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    else 
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    
    int speed = constrain(abs(output), 0, 255);
    analogWrite(ENA, speed);
  }
  
};

 // create a led controller class by inheriting from PID
class ledController : public PID{
  
  // Override input function to use input we want (Encoders)
  float inputFunction() override {
    return analogRead(RphotoPin);
  }

  
  // Override output function to return the output we want (Speed)
  void outputFunction(float output) override {
    
    brightness+=output;
    brightness = constrain(brightness, 0, 255);
    Serial.println(brightness);
    analogWrite(LEDpin, brightness);
  }
  
};


// Create objects
speedController controller1; 
ledController controller2;

PID* referenceArray[] = {&controller1, &controller2};

ConcurrentPID multiController(referenceArray, 2);

  void setup()
{

  // Declaire pins as input and output
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(PotPin, INPUT);
  pinMode(RphotoPin, INPUT_PULLUP);
  
  // attach interrupt to pin ENCA the function interruptFunc()
  attachInterrupt(digitalPinToInterrupt(ENCA), interruptFunc, RISING);

  // Set variables to pid controllers
  controller1.setPIDvalues(10,0.5,0.1);
  controller2.setPIDvalues(10,0.5,0.1);

  }

void loop()
{
  
  // set the Goal of all controllers to potentiometer output
  multiController.setGoal((float)analogRead(PotPin)/3);

  // all controllers calculate()
  multiController.calculate();

}
