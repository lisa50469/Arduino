// Shows how to run three Steppers at once with varying speeds
//
// Requires the Adafruit_Motorshield v2 library 
//   https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// And AccelStepper with AFMotor support 
//   https://github.com/adafruit/AccelStepper

// This tutorial is for Adafruit Motorshield v2 only!
// Will not work with v1 shields

//#include <Wire.h>
//#include <AccelStepper.h>
//#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
//#include "Adafruit_PWMServoDriver.h"

//Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMS(0x60); // Default address, no jumpers

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 2);

// Connect one stepper with 200 steps per revolution (1.8 degree)
// to the bottom shield
//Adafruit_StepperMotor *myStepper3 = AFMSbot.getStepper(200, 2);

// you can change these to SINGLE DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  myStepper1->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, DOUBLE);
}



// Now we'll wrap the 3 steppers in an AccelStepper object
//AccelStepper stepper1(forwardstep1, backwardstep1);

void setup()
{  
//  AFMSbot.begin(); // Start the bottom shield
  AFMS.begin(); // Start the top shield
   
  stepper1.setMaxSpeed(1600.0);
  stepper1.setAcceleration(100.0);
  stepper1.moveTo(1000);
    
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
//	stepper1.moveTo(-stepper1.currentPosition());
stepper1.release();
//stepper1.disableOutputs();

    stepper1.run();
}

