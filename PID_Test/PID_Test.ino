/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/

#include <PID_v1.h>
#include <Servo.h>
#include <Console.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
double power = 20, mass = 5, distance = 0, speed = 0, force, maxForce, constForce = 0;
unsigned long lastUpdate, lastPrint;

double updatePosition(){
  double time = (double)(millis() - lastUpdate) / 1000.0;
  lastUpdate = millis();
  distance += time * speed + .5 * time * time * force / mass;
  speed += time * force / mass;
  return distance;
}

void updateForce(double effort){
  double percentage = (effort - 128)/128;
  //Serial.println(percentage);
  double maxForce;
  if ((percentage > 0) == (speed > 0)){
    maxForce = abs(abs(speed) > 1 ? power/speed : power);
  }
  else{
    maxForce = power;
  }
  //Serial.println(maxForce);
  force = maxForce * percentage + constForce;  
}

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 10;
  lastUpdate = millis();
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop(){
  
  Input = updatePosition();
  if (millis() - lastPrint > 1000){
    Serial.println(Input);
    Serial.println(Output);
    Serial.println(force);
    Serial.println();
    lastPrint = millis();
  }
  //Serial.println("test");
  if (Console.available() > 0){
    Setpoint = Console.read();
  }

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  updateForce(Output);
}
