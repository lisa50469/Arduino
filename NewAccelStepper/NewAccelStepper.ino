#include <AccelStepper.h>
#define TB6600Disable 


// Define a stepper motor 1 for arduino
// direction Digital 9 (CW), pulses Digital 8 (CLK)
// (Driver, StepPin, DirPin)
AccelStepper stepper(1, 2, 3); // 1=Driver, 2=StepPin, 3=DirPin


void setup()
{
  // Change these to suit your stepper if you want
  stepper.setEnablePin(4);
 // stepper.setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);
  stepper.setPinsInverted(false, false, true); // 3rd item is enable inverted for TB6600
  stepper.enableOutputs();
  stepper.setMaxSpeed(2000);//1100
  stepper.setAcceleration(400);
  stepper.moveTo(4000);
}

void loop()
{
  // If at the end of travel go to the other end
  if (stepper.distanceToGo() == 0) {
    stepper.disableOutputs(); 
    delay(2000);
    stepper.enableOutputs(); 
    stepper.moveTo( -stepper.currentPosition() );
  }

  stepper.run();
}
