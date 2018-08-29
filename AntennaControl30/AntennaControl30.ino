#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <TimeLib.h>

#define MaxSpeedAZ 20000
#define MaxAccelAZ 100
#define MaxSpeedEL 200
#define MaxAccelEL 40

/*-----------------------------------------------------------------------------------
 * Designed by Lisa Nelson - AK7WS
 * Antenna rotator 
 * Actual and commanded coords are in servo counts!  So Degrees passed to/from
 * have to be converted.  This allows you to use gears/reduction simply by putting
 * the Resolution count to that of the final assembly.
 * 
 * I'm using Adafruit Motor Shield V1.2, (updated to motor shield version 2.)
-----------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
const int CalibrationTimeout = 1;  // Minutes between idle calibrations.

//  The following resolutions are determined by the gear ratio and stepper
//  abilities.  For example, the steppers I'm using are 400 steps per revolution.
//  BUT I have gears in there dropping that down.  About a ratio of ~8.6:1 for the AZ
//  and about 20:1 for the EL.  I wrote simple test code to rotate until I hit the 
//  360 degree mark since I didn't have exact info on the scale of the gears.
const int AZResolutionPerTurn = 7990;  // Stepper counts to make one revolution of the Axis.
const int ELResolutionPerTurn = 7990;  // Stepper counts to make one revolution of the Axis.

const int ELhomePin = 17;     // the number of the int pin (18 - 21 work fine)
const int AZhomePin = 16;     // the number of the int pin (18 - 21 work fine) 

float AZCommandPosition = 0;    // Commanded position in Steps.
float ELCommandPosition = 0;    // Commanded position in Steps.
//float AZLastCommandPos = -1;    // For detecting idle state.
//float ELLastCommandPos = -1;    // For detecting idle state.

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean needhomed = true;         // Home when idle for CalibrationTimeout

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS(0x60); // Default address, no jumpers

Adafruit_StepperMotor *AZmotor = AFMS.getStepper(AZResolutionPerTurn, 1);
Adafruit_StepperMotor *ELmotor = AFMS.getStepper(ELResolutionPerTurn, 2);

// you can change these to DOUBLE, SINGLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void AZforward() {  
  AZmotor->onestep(BACKWARD, INTERLEAVE);
}
void AZbackward() {  
  AZmotor->onestep(FORWARD, INTERLEAVE);
}
// wrappers for the second motor!
void ELforward() {  
  ELmotor->onestep(FORWARD, INTERLEAVE);
}
void ELbackward() {  
  ELmotor->onestep(BACKWARD, INTERLEAVE);
}

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
AccelStepper AZstepper(AZforward, AZbackward);
AccelStepper ELstepper(ELforward, ELbackward);

//-----------------------------------------------------------------------------
void AZfindhome() {
  Serial.print("Homing AZ.....");
  AZstepper.runToNewPosition(0); // Will this unwind the cables.
  // First move away from the flag in case it's close.
  for (int i = 0; i <= AZResolutionPerTurn/16; i++) AZmotor->step(1, FORWARD, INTERLEAVE);
  long timeout = 0;
  while (digitalRead(AZhomePin) && (timeout < AZResolutionPerTurn + 5)) {
    timeout++;  // Don't just move forever, stop after a resonable time.
    AZmotor->step(1, BACKWARD, INTERLEAVE);
    }
  AZstepper.setCurrentPosition(0);
  Serial.println("Done.");
  TakeShortCut(AZCommandPosition);
  needhomed = false;  // Home has been done.
}


//-----------------------------------------------------------------------------
void ELfindhome() {
  Serial.print("Homing EL.....");
  // With EL, make sure we don't move forward unless we are seeing the flag
  // or it might hit the mechanical limit of travel.
  if (!digitalRead(ELhomePin))  // Are we at the flag. Move off to find home.
    for (int i = 0; i <= 250; i++) ELmotor->step(1, FORWARD, INTERLEAVE);
  long timeout = 0;
  while (digitalRead(ELhomePin) && (timeout < ELResolutionPerTurn / 4)) {
    timeout++;  // Don't just move forever, stop after a resonable time.
    ELmotor->step(1, BACKWARD, INTERLEAVE); // Move to the flag.
  }
  ELstepper.setCurrentPosition(0);
  Serial.println("Done.");
}

/*-----------------------------------------------------------------------------
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {  // For the USB connection and programming.
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
   }
  }
}

/*-----------------------------------------------------------------------------
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent1() {  // For RS232 between the laptop and rotator.
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

//-----------------------------------------------------------------------------
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // LED off!

  // reserve 500 bytes for the inputString:
  inputString.reserve(500);  // Used for talking to rotctl

  // Sensor pins for HOME sensors
  pinMode(ELhomePin, INPUT);  // Elevation home flag
  pinMode(AZhomePin, INPUT);  // Azuimith home flag

  Serial.begin(9600);         // initialize serial communications with your computer
  Serial1.begin(9600);        // initialize serial communications with your computer

  setTime(DEFAULT_TIME);  // Reset the time, we don't really care, we just use minutes.
  
  AFMS.begin(); // Start the top shield

  AZfindhome();
  AZfindhome();  // Twice to take care of when the first move blocks the flag.
  ELfindhome();
  
  AZstepper.setMaxSpeed(MaxSpeedAZ);
  AZstepper.setAcceleration(MaxAccelAZ);
  ELstepper.setMaxSpeed(MaxSpeedEL);
  ELstepper.setAcceleration(MaxAccelEL);


   //Serial.print("Max AZ Speed is:"); Serial.println( AZstepper.maxSpeed());

}

//-----------------------------------------------------------------------------
void PrintStats(String Mymarker) {
  Serial.print(Mymarker);
      Serial.print("  AZ:"); Serial.print(AZCommandPosition*360/AZResolutionPerTurn);
      Serial.print("  EL:"); Serial.print(ELCommandPosition*360/ELResolutionPerTurn);
      Serial.print("  AZ-Steps:"); Serial.print(AZCommandPosition);
      Serial.print("  EL-Steps:"); Serial.print(ELCommandPosition);
      Serial.print("  Time:"); Serial.print(minute()); Serial.print(":"); Serial.println(second());
}

//-----------------------------------------------------------------------------
void TakeShortCut(int RequestedSteps) {
  if ((AZstepper.currentPosition()-RequestedSteps) > AZResolutionPerTurn/2) {
    RequestedSteps += AZResolutionPerTurn;
    }
  if ((AZstepper.currentPosition()-RequestedSteps) < -AZResolutionPerTurn/2) {
    RequestedSteps -= AZResolutionPerTurn;
    }
  AZCommandPosition = RequestedSteps;  // New goal for the AZ motor.
}

//-----------------------------------------------------------------------------
void ProcessCommands()  { // We got any new commands
  float NewNumber;
  float CurEL;
  float CurAZ;
  int MessageNum;
  
  MessageNum = 0;
  inputString.toUpperCase();
  if (inputString.indexOf("STAT")==0) MessageNum = 1; // Report positions.
  if (inputString.indexOf("HOME")==0) MessageNum = 2; // Home both axis.
  if (inputString.indexOf("AZ EL")==0) MessageNum = 3; // Gpredict is asking...
  if (inputString.indexOf("CW")==0) MessageNum = 4; // Unwind Clockwise one turn...
  if (inputString.indexOf("CCW")==0) MessageNum = 5; // Unwind Clockwise one turn...
  if (inputString.indexOf("RESET")==0) MessageNum = 6; // Home and Reset to zero...


  switch (MessageNum) {
    
    case 1: // Status
      PrintStats("--REQ--");
      break;

    case 2: //Home
      AZfindhome();
      ELfindhome();
      break;

    case 3: // Gpredict is asking...
      CurAZ = AZCommandPosition*360/AZResolutionPerTurn;
      if (CurAZ < 0) CurAZ +=360;
      if (CurAZ > 360) CurAZ -= 360;
      Serial1.print("AZ"); Serial1.print(CurAZ); Serial1.print(" ");
      Serial.print("AZ"); Serial.print(CurAZ); Serial.print(" ");
      CurEL = ELCommandPosition*360/ELResolutionPerTurn;
      Serial1.print("EL"); Serial1.print(CurEL); Serial1.print('\n');
      Serial.print("EL"); Serial.print(CurEL); Serial.print('\n');
      break;

    case 4: //CW
        Serial.println("CW Rotation.");
        AZCommandPosition += AZResolutionPerTurn;  // New goal for the AZ motor.
       break;


    case 5: // CCW
        Serial.println("CCW Rotation.");
        AZCommandPosition -= AZResolutionPerTurn;  // New goal for the AZ motor.
        break;

    case 6: //Reset
        Serial.println("Reset.");
        AZstepper.setCurrentPosition(0);
        AZCommandPosition = 0; // Override the commanded position.
        AZfindhome();
        break;

    default:
      // Assume it's new coordinates.
      Serial.print("Command: "); Serial.println(inputString);
      int divider = inputString.indexOf(' '); // Look for space between AZ123 EL23.
      int terminator = inputString.lastIndexOf('\n');  // End of line character.
      if (divider >= 1) {
        String newcords = "";  // Found the space, must be new coordinates.
        for ( int i=2; i <= divider-1; i++) // Get the AZ info.
            newcords.concat(inputString[i]);
        newcords.concat("\n");
        NewNumber =  newcords.toFloat()*AZResolutionPerTurn/360;
        TakeShortCut(NewNumber);
        needhomed = true;  // Home next time it's idle.
        setTime(DEFAULT_TIME); // Reset the time, we don't really care, we just use minutes.

        newcords = "";  // Start with empty string to get the EL numbers.
        for (int i=divider+3; i <= terminator; i++)
          newcords.concat(inputString[i]);
        newcords.concat("\n");
        float NewEL = newcords.toFloat(); // Get the new Elevation number.
        NewNumber =  newcords.toFloat()*ELResolutionPerTurn/360;

        //Check for the limits of the assembly.  0 to 90 degrees.
        if ((NewEL>=0) && (NewEL <=90)){
          ELCommandPosition = NewNumber;
          needhomed = true;  // Home next time it's idle.
          setTime(DEFAULT_TIME); // Reset the time, we don't really care, we just use minutes.
          }
          
        PrintStats("--NEW-->"); // Display new info on the terminal if it's connected.
        }
      break;
    }
    inputString = "";         // Clear the old command text.
    stringComplete = false;   // Be ready for next command from either port.
}

//-----------------------------------------------------------------------------
void loop() {
  if (stringComplete) ProcessCommands();  // We got some command from a port.
 
  AZstepper.moveTo(AZCommandPosition);  // Tell the steppers to track
  ELstepper.moveTo(ELCommandPosition);

  if (minute() >= CalibrationTimeout) {  // After this many minutes, timout and do calibration and Home.
    /* Using the stepper counts, they can go negative, so driving back to zero will
     * unwind the cords.  I also change the commanded position if it's past one rotation.
     */
     Serial.println("IDLE...");
      if (AZCommandPosition > AZResolutionPerTurn) AZCommandPosition -= AZResolutionPerTurn;
      if (AZCommandPosition < -AZResolutionPerTurn) AZCommandPosition += AZResolutionPerTurn;
     if (needhomed) {
      Serial.print("Need HOMED...");
      //if (AZCommandPosition > AZResolutionPerTurn) AZCommandPosition -= AZResolutionPerTurn;
      //if (AZCommandPosition < -AZResolutionPerTurn) AZCommandPosition += AZResolutionPerTurn;
      AZstepper.runToNewPosition(0);  // Blocks and drives back to the zero position.
      ELstepper.runToNewPosition(0);  // Blocks and drives back to the zero position.
      AZfindhome();   // Re-find home again.
      ELfindhome();   // Re-find home again.
      Serial.println();
     }
     else
     {
          AZmotor->release();
          ELmotor->release();
     }
    setTime(DEFAULT_TIME);          // Reset the time, we don't really care, we just use minutes.
    }
   
  AZstepper.run();  // Enable the steppers.  They will keep driving to commanded positions.
  ELstepper.run();
}
