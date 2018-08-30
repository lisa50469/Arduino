#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <TimeLib.h>

#define MaxSpeedAZ 4000
#define MaxAccelAZ 100
//#define MaxSpeedEL 2000
//#define MaxAccelEL 300
#define MaxSpeedEL 300
#define MaxAccelEL 50

/*-----------------------------------------------------------------------------------
   Designed by Lisa Nelson - AK7WS
   Antenna rotor V2.0
   Actual and commanded coords are in servo counts!  So Degrees passed to/from
   have to be converted.  This allows you to use gears/reduction simply by putting
   the Resolution count to that of the final assembly.

   I'm using 2 X TB6600 stepper drivers.
  -----------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
const int CalibrationTimeout = 2;  // Minutes between idle calibrations.

//  The following resolutions are determined by the gear ratio and stepper
//  abilities.  For example, the steppers I'm using are 400 steps per revolution.
//  BUT I have gears in there dropping that down.  About a ratio of ~8.6:1 for the AZ
//  and about 20:1 for the EL.  I wrote simple test code to rotate until I hit the
//  360 degree mark since I didn't have exact info on the scale of the gears.
const int AZResolutionPerTurn = 15900;  // Stepper counts to make one revolution of the Axis.
//const int ELResolutionPerTurn = 12800;  // Stepper counts to make one revolution of the Axis.
const int ELResolutionPerTurn = 12500;  // Stepper counts to make one revolution of the Axis.

const int ELhomePin = 17;     // the number of the int pin (18 - 21 work fine)
const int AZhomePin = 16;     // the number of the int pin (18 - 21 work fine)

float AZCommandPosition = 0;    // Commanded position in Steps.
float ELCommandPosition = 0;    // Commanded position in Steps.
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean needhomed = true;         // Home when idle for CalibrationTimeout
boolean PowerSaveAZ = true;       // When true, power down motor when idle.
boolean PowerSaveEL = true;       // When true, power down motor when idle.

// New TB6600 Driver is now used.
AccelStepper ELmotor(1, 31, 33); // 1=Driver, 31=StepPin, 33=DirPin
AccelStepper AZmotor(1, 39, 41); // 1=Driver, 39=StepPin, 41=DirPin

//-----------------------------------------------------------------------------
void setup() {
  // New code for using the TB6600 driver
  ELmotor.setEnablePin(35);
  AZmotor.setEnablePin(43);
  // stepper.setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);
  ELmotor.setPinsInverted(false, false, true); // 3rd item is enable inverted for TB6600 enable pin.
  ELmotor.enableOutputs();  // Enable the stepper driver.
  AZmotor.setPinsInverted(false, false, true); // 3rd item is enable inverted for TB6600 enable pin.

  AZmotor.setMaxSpeed(MaxSpeedAZ);
  AZmotor.setAcceleration(MaxAccelAZ);
  ELmotor.setMaxSpeed(MaxSpeedEL);
  ELmotor.setAcceleration(MaxAccelEL);
  ELmotor.enableOutputs();  // Enable the stepper driver.
  AZmotor.enableOutputs();  // Enable the stepper driver.

  // reserve 500 bytes for the inputString:
  inputString.reserve(500);  // Used for talking to rotctl

  // Sensor pins for HOME sensors
  pinMode(ELhomePin, INPUT);  // Elevation home flag
  pinMode(AZhomePin, INPUT);  // Azuimith home flag

  Serial.begin(9600);         // initialize serial communications Mega USB port.
  Serial1.begin(9600);        // initialize serial communications with your computer

  setTime(DEFAULT_TIME);  // Reset the time, we don't really care, we just use minutes.

  AZfindhome();
  //  AZfindhome();  // Twice to take care of when the first move blocks the flag.
  ELfindhome();
}

//-----------------------------------------------------------------------------
void AZfindhome() {
  Serial.print("Homing AZ.....");
  long homecount = 0;
  AZmotor.enableOutputs();
  AZmotor.setMaxSpeed(MaxSpeedAZ);
  AZmotor.setAcceleration(MaxAccelAZ);
  AZmotor.runToNewPosition(0);  // Blocks and drives back to the zero position.

  if (!digitalRead(AZhomePin))
  {
    homecount = -AZResolutionPerTurn / 9; // Move away from home position.
    AZmotor.moveTo(homecount);          // Set the position to move to
    AZmotor.runToNewPosition(homecount);
  }

  AZmotor.setMaxSpeed(MaxSpeedAZ / 5);
  while (digitalRead(AZhomePin))
  { // Make the Stepper move CCW until the switch is activated
    AZmotor.moveTo(homecount);  // Set the position to move to
    homecount++;    // Increase by 1 for next move if needed
    AZmotor.run();  // Start moving the stepper
  }

  AZmotor.setMaxSpeed(MaxSpeedAZ);
  AZmotor.setCurrentPosition(0);
  Serial.println("Done.");

  TakeShortCut(AZCommandPosition);
  needhomed = false;  // Home has been done.
}

//-----------------------------------------------------------------------------
void ELfindhome() {
  Serial.print("Homing EL.....");
  long homecount = ELCommandPosition;
  // With EL, make sure we don't move forward unless we are seeing the flag
  // or it might hit the mechanical limit of travel.

  ELmotor.enableOutputs();
  ELmotor.setMaxSpeed(MaxSpeedEL);
  ELmotor.setAcceleration(MaxAccelEL);

  if (!digitalRead(ELhomePin))
  {
    homecount = -500;
    ELmotor.moveTo(homecount);  // Set the position to move to
    ELmotor.runToNewPosition(homecount);
  }

  ELmotor.setMaxSpeed(MaxSpeedEL);
  while (digitalRead(ELhomePin))
  { // Make the Stepper move CCW until the switch is activated
    ELmotor.moveTo(homecount);  // Set the position to move to
    homecount++;  // Decrease by 1 for next move if needed
    ELmotor.run();  // Start moving the stepper
  }

  ELmotor.setMaxSpeed(MaxSpeedEL);
  ELmotor.setCurrentPosition(0);
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
void PrintStats(String Mymarker) {
  Serial.print(Mymarker);
  Serial.print("  AZ:"); Serial.print(AZCommandPosition * 360 / AZResolutionPerTurn);
  Serial.print("  EL:"); Serial.print(ELCommandPosition * 360 / ELResolutionPerTurn);
  Serial.print("  AZ-Steps:"); Serial.print(AZCommandPosition);
  Serial.print("  EL-Steps:"); Serial.print(ELCommandPosition);
  Serial.print("  Time:"); Serial.print(minute()); Serial.print(":"); Serial.println(second());
}

//-----------------------------------------------------------------------------
void TakeShortCut(int RequestedSteps) {
  int movedistance = (RequestedSteps - AZmotor.currentPosition());

  if (movedistance < -AZResolutionPerTurn / 2) {
    RequestedSteps += AZResolutionPerTurn;
  }

  if (movedistance > AZResolutionPerTurn / 2) {
    RequestedSteps -= AZResolutionPerTurn;
  }

  if (AZCommandPosition > AZResolutionPerTurn) AZCommandPosition -= AZResolutionPerTurn;
  if (AZCommandPosition < -AZResolutionPerTurn) AZCommandPosition += AZResolutionPerTurn;

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
  if (inputString.indexOf("STAT") == 0) MessageNum = 1; // Report positions.
  if (inputString.indexOf("HOME") == 0) MessageNum = 2; // Home both axis.
  if (inputString.indexOf("AZ EL") == 0) MessageNum = 3; // Gpredict is asking...
  if (inputString.indexOf("CW") == 0) MessageNum = 4; // Unwind Clockwise one turn...
  if (inputString.indexOf("CCW") == 0) MessageNum = 5; // Unwind Clockwise one turn...
  if (inputString.indexOf("RESET") == 0) MessageNum = 6; // Home and Reset to zero...
  if (inputString.indexOf("OFF") == 0) MessageNum = 7; // Turn Motors off...
  if (inputString.indexOf("ON") == 0) MessageNum = 8; // Turn Motors on...
  if (inputString.indexOf("TIME") == 0) MessageNum = 9; // Force Idle Timeout...
  if (inputString.indexOf("NOPSAZ") == 0) MessageNum = 10; // Power Save AZ off...
  if (inputString.indexOf("NOPSEL") == 0) MessageNum = 11; // Power Save EL off...
  if (inputString.indexOf("PSAZ") == 0) MessageNum = 12; // Power Save AZ on...
  if (inputString.indexOf("PSEL") == 0) MessageNum = 13; // Power Save EL on...
  if (inputString.indexOf("HELP") == 0) MessageNum = 99; // Show commands...


  switch (MessageNum) {

    case 1: // Status
      PrintStats("--REQ--");
      break;

    case 2: //Home
      AZfindhome();
      ELfindhome();
      break;

    case 3: // Gpredict is asking...
      CurAZ = AZCommandPosition * 360 / AZResolutionPerTurn;
      if (CurAZ < 0) CurAZ += 360;
      if (CurAZ > 360) CurAZ -= 360;
      Serial1.print("AZ"); Serial1.print(CurAZ); Serial1.print(" ");
      Serial.print("AZ"); Serial.print(CurAZ); Serial.print(" ");
      CurEL = ELCommandPosition * 360 / ELResolutionPerTurn;
      Serial1.print("EL"); Serial1.print(CurEL); Serial1.print('\n');
      Serial.print("EL"); Serial.print(CurEL); Serial.print('\n');
      break;

    case 4: //CW
      Serial.println("CW Rotation.");
      AZCommandPosition -= AZResolutionPerTurn;  // New goal for the AZ motor.
      break;

    case 5: // CCW
      Serial.println("CCW Rotation.");
      AZCommandPosition += AZResolutionPerTurn;  // New goal for the AZ motor.
      //AZmotor.runToNewPosition(AZCommandPosition);
      break;

    case 6: //Reset
      Serial.println("Reset.");
      AZmotor.setCurrentPosition(0);
      AZCommandPosition = 0; // Override the commanded position.
      AZfindhome();
      break;

    case 7: //Off
      Serial.println("Motors OFF.");
      AZmotor.disableOutputs();
      ELmotor.disableOutputs();
      break;

    case 8: //On
      Serial.println("Motors ON.");
      AZmotor.enableOutputs();
      ELmotor.enableOutputs();
      break;

    case 9: //Timout
      Serial.println("Forced Timout.");
      setTime(1358041600);
      break;
      
    case 10: //Power save off for AZ
      PowerSaveAZ = false;
      break;

    case 11: //Power save off for EL
      PowerSaveEL = false;
      break;

    case 12: //Power save on for AZ
      PowerSaveAZ = true;
      break;

    case 13: //Power save on for EL
      PowerSaveEL = true;
      break;
      
    case 99: //Help list
      Serial1.println("STAT,HOME,CW,CCW,RESET,OFF,ON,TIME,NOPSAZ,NOPSEL,PSAZ,PSEL,HELP");
      break;

    default:
      // Assume it's new coordinates.
      Serial.print("New Coords: "); Serial.print(inputString);
      int divider = inputString.indexOf(' '); // Look for space between AZ123 EL23.
      int terminator = inputString.lastIndexOf('\n');  // End of line character.
      if (divider >= 1) {
        String newcords = "";  // Found the space, must be new coordinates.
        for ( int i = 2; i <= divider - 1; i++) // Get the AZ info.
          newcords.concat(inputString[i]);
        newcords.concat("\n");
        NewNumber =  newcords.toFloat() * AZResolutionPerTurn / 360;
        NewNumber = -NewNumber;  // Reverse direction of rotation.
        TakeShortCut(NewNumber);
        needhomed = true;  // Home next time it's idle.
        setTime(DEFAULT_TIME); // Reset the time, we don't really care, we just use minutes.

        newcords = "";  // Start with empty string to get the EL numbers.
        for (int i = divider + 3; i <= terminator; i++)
          newcords.concat(inputString[i]);
        newcords.concat("\n");
        float NewEL = newcords.toFloat(); // Get the new Elevation number.
        NewNumber =  newcords.toFloat() * ELResolutionPerTurn / 360;

        //Check for the limits of the assembly.  0 to 90 degrees.
        if ((NewEL >= 0) && (NewEL <= 91))
        {
          ELCommandPosition = -NewNumber;  // Negative to reverse movement
          ELmotor.enableOutputs();
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

  AZmotor.moveTo(AZCommandPosition);  // Tell the steppers to track
  ELmotor.moveTo(ELCommandPosition);

  if (minute() >= CalibrationTimeout) {  // After this many minutes, timout and do calibration and Home.
    /* Using the stepper counts, they can go negative, so driving back to zero will
       unwind the cords.  I also change the commanded position if it's past one rotation.
    */
    Serial.println("IDLE...");
    if (needhomed) {
      Serial.print("Need HOMED...");
      AZfindhome();   // Re-find home again.
      ELfindhome();   // Re-find home again.
      Serial.println();
    }
    setTime(DEFAULT_TIME);          // Reset the time, we don't really care, we just use minutes.
  }

  if (PowerSaveAZ && (AZmotor.distanceToGo() == 0)) AZmotor.disableOutputs(); else AZmotor.enableOutputs();
  if (PowerSaveEL && (ELmotor.distanceToGo() == 0)) ELmotor.disableOutputs(); else ELmotor.enableOutputs();
  AZmotor.run();  // Enable the steppers.  They will keep driving to commanded positions.
  ELmotor.run();  // Enable the steppers.  They will keep driving to commanded positions.
}
