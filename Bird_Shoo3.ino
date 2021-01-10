/*
  This is a bird scarecrow - used to shoo the birds from the windows
  It uses a PIR sensor to detect a bird flying into the scene
  Once detected, the code drives a stepper motor to rotate an arm
  waving a streamer to scare the bird away.
  Once the movement times out, the code disables the motor and 
  waits for another detection.
 Sketch uses the Adafruit Motor Shield for Arduino v2
 PIR sensor conntected to pin 2
 Stepper moter connected to M1 and M2.  Motor wiring:
   RED=M1; GRN=M1; YEL=M2; BLU=M2
   I use the Kysan Stepper 1124090
    V = 4.2V
    I = 1.5A
    Step = 1.8 deg (step = 360/1.8 = 200)

 There is a tri-coloer LED to identify the state:
  - Green = Waiting for a bird
  - Red = Wave the flag
  - Blue = Wait for reset
 
 NOTE:  be careful with the PIR sensor.  It has a a POT to 
 extend the pulse output. If set too long, this could cause the
 code to end up in a continuous loop.  I made the pulse short
 so that it would time out while the entire movement completes.
 Then the code will start looking for another PIR code trigger.
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

const int SensorSignal = 2; // Input pin D2 (for PIR sensor)
const int GreenLED = 3;     // Output pin D3 (waiting for dection)
const int RedLED = 4;       // Output pin D4 (motor movement cycle)
const int BlueLED = 5;      // Output pin D5 (waiting to reset system)
int sensorValue = 0;        // variable to store if the PIR sensor detected.
// We start, assuming no motion detected for sensor


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(SensorSignal, INPUT); // declare sense1 as input
  pinMode(GreenLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(BlueLED, OUTPUT);
  AFMS.begin();                 // create with the default frequency 1.6KHz
  //AFMS.begin(1000);           // OR with a different frequency, say 1KHz
  myMotor->setSpeed(2);         // 2 rpm
}

void loop() 
{
    // Set LED to state Green - waiting for movement
    digitalWrite(GreenLED, HIGH); 
    digitalWrite(RedLED, LOW);      
    digitalWrite(BlueLED, LOW);    
  sensorValue = digitalRead(SensorSignal);  // Read the PIR sensor
    delay(100);
    // Serial.print("sensorValue: ");         //diagnonstics - comment out
    // Serial.println(sensorValue);         
   // Serial.println("No Sensor Detection");

  if (sensorValue == HIGH)
  {
    Serial.println("Sensor Detection");
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    // Set LED to state Red - Move the flag
    digitalWrite(GreenLED, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(RedLED, HIGH);   // turn off LED on (HIGH is the voltage level)
    digitalWrite(BlueLED, LOW);   // turn off LED on (HIGH is the voltage level)


    
    Serial.println("Wave Flag");
    myMotor->step(365, FORWARD, DOUBLE); // SINGLE, DOUBLE, INTERLEAVE or MICROSTEP
    myMotor->step(226, BACKWARD, DOUBLE);
    delay(1000);
    myMotor->step(365, FORWARD, DOUBLE);
    myMotor->step(226, BACKWARD, DOUBLE);     
    // Set LED to state Blue - Reset motor and waiting for reset
    digitalWrite(GreenLED, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(RedLED, LOW);     // turn off LED on (HIGH is the voltage level)
    digitalWrite(BlueLED, HIGH);   // turn off LED on (HIGH is the voltage level)
    myMotor->release(); // Release the motor and wait for next sense
     //Serial.print("sensorValue: ");  // diagnonstics - comment out
     //Serial.println(sensorValue);    
    Serial.println("Shoo Over"); 
    delay(10000);
    Serial.println("Ready for next bird"); 
     
     //Serial.print("sensorValue: ");  // diagnonstics - comment out
     //Serial.println(sensorValue);    
     //digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW

 }
  else 
  {
    myMotor->release(); // Release the motor and wait for next sense
    //digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    // Set LED to state Green - waiting for movement
    digitalWrite(GreenLED, HIGH);
    digitalWrite(RedLED, LOW);
    digitalWrite(BlueLED, LOW);

  } 
}
