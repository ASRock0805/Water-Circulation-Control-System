/*
  Water Circulation Control System w/ Flow Rate Feedback Control

      The program integrates two sensors and pumps, namely, a water pressure sensor and a flow sensor.
  The flow rate and pressure can be read through a personal computer, and the pump speed percentage (PWM)
  can be set through the serial port.

  - Water Pressure Sensor (DFRobot SEN0257)
      The output voltage of the sensor is converted by ADC to obtain the water pressure.

  - Compact Electromagnetic Flow Sensor (Aichi Tokei VN05)
      Use interrupts to calculate the number of unit pulses per interval to convert the water flow rate (mL/minute).

  - Kamoer KDS Peristaltic Pump (KDS-FE-2-S17B)
      12V DC brush motor

  Created 2 Dec. 2020
  by Yi-Xuan Wang

  References:
  https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257
  https://www.aichitokei.net/products/compact-electromagnetic-flow-sensor-vn/
  http://kamoer.com/Products/showproduct.php?id=234
*/

/*--- Preprocessor ---*/
#include <Wire.h>   // Import Wire library for Inter-Integrated Circuit (I^2C)

#define MAIN_CTRL 1 // Address of main controller w/ sensor
#define PUMP_CTRL 2 // Address of pump controller

#define flowPin 2   // Pin location of the sensor (D2) w/ interrupt (INT.1)
#define sigPin A0   // Potentiometer signal pin w/ ADC

#define N 800       // Measurement sampling number for smoothing

/*--- Constants ---*/
const unsigned long baudSpeed = 115200; // Sets the data rate in bits per second (baud) for serial data transmission
const unsigned long period = 1000;      // The value is a number of milliseconds

const byte vIn = 5;                                   // Supply voltage from Arduino
const byte resBits = 10;                              // Resolution of ADC (10 bits)
const float vConv = vIn / (pow(2.0, resBits) - 1.0);  // Voltage of ADC level (2^bits)

// Spec. of water pressure sensor, Range: 0 - 16 MPa, Output: 0.5 - 4.5 V
const float pgMax = 16.0;             // Upper limit of pressure sensor
const float pgMin = 0.0;              // Lower limit of pressure sensor
const float pgVmax = 4.5;             // Maximum output voltage of pressure sensor
const float pgVmin = 0.5;             // Minimum output voltage of pressure sensor
const float offSet = 0.471772766113;  // Norminal value is 0.5 V

const byte tolerance = 10;            // Tolerance of flow rate (%)
const float adjFactor = 0.002;        // Adjustment factor of feedback level

/*--- Global Variables ---*/
unsigned long startTime;            // Start time
unsigned long currentTime;          // Current time
unsigned long timer;                // Stopwatch

volatile byte percent;              // Percentage of pump PWM

float vOut;                         // Output of the ADC
float waterPres;                    // Value of water pressure

volatile unsigned long pulseCount;  // Measuring the falling edges of the signal
static unsigned long cumCount;      // Cumulative count
float flowRate;                     // Value of water flow rate
float flowML;                       // Unit converter (milliliter, mL)
float totalML;                      // Volume of cumulative water

// Target flow rate, and it tolerance
float targetFlow;                   // Manual adjustment
float toleranceFlow;
float flowUL;                       // Upper limit of flow rate (mL/min)
float flowLL;                       // Lower limit of flow rate (mL/min)
volatile float flowDev;

/*--- Function Prototype ---*/
void getCounter(void);
void flushReceive(void);
void i2cTransmit(const byte , byte );
float getwaterPres(float );
void waterPressure(const byte );
void feedbackPWM(float );
void serialEvent(void);
void setup(void);
void loop(void);

/*--- Initialization ---*/
void setup(void) {
  Wire.begin(MAIN_CTRL);          // Initializes Wire and join I2C bus
  Serial.begin(baudSpeed);        // Initializes serial port

  pinMode(sigPin, INPUT);         // Initializes potentiometer pin
  pinMode(flowPin, INPUT_PULLUP); // Initializes interrupt digital pin 2 declared as an input and pull-up resitor enabled

  startTime = millis();           // Initial start time

  // Pump Percentage Initialization
  percent = 100;
  Serial.print("Initial Speed: ");  
  Serial.print(percent);
  Serial.println(" %");
  i2cTransmit(PUMP_CTRL, percent);

  // Water Pressure Sensor Initialization
  vOut = 0.0;
  waterPres = 0.0;

  // Flow Sensor Initialization
  pulseCount = 0;
  cumCount = 0;
  flowRate = 0.0;
  flowML = 0.0;
  attachInterrupt(digitalPinToInterrupt(flowPin), getCounter, FALLING); // The interrupt is attached
}

/*--- Measurement ---*/
void loop(void) {
  // Every second, calculate and print the measured value
  currentTime = millis();                     // Get the current "time"

  if ((currentTime - startTime) >= period) {  // Test whether the period has elapsed
    timer = startTime / period;               // Calculate the period of time

    // Water Pressure Sensor
    waterPressure(sigPin);

    // Flow Sensor
    detachInterrupt(digitalPinToInterrupt(flowPin));  // Clears the function used to attend a specific interrupt
    cumCount = cumCount + pulseCount;                 // Count increment
    flowCal(pulseCount);

    /*--- Sensor prompt ---*/
    Serial.print("Measuring, Speed: ");
    Serial.print(percent);
    Serial.print(" %, ");
/*
    Serial.print(", Voltage: ");
    Serial.print(vOut, 12);
    Serial.print(" V, ");
    // Unit converter for pressure raw unit: MPa
    Serial.print("Pressure: ");
    Serial.print(waterPres * 1000, 1);    // 1 : 1000
    Serial.print(" kPa, ");
    Serial.print(waterPres, 2);           // Raw
    Serial.print(" MPa, ");
    Serial.print(waterPres * 10.1972, 1); // 1 : 10.1972
    Serial.print(" kg/cm^2, "); 

    Serial.print("Cumulative Count: ");
    Serial.print(cumCount);
    Serial.print(", Pulse Count: ");
    Serial.print(pulseCount);
*/
    Serial.print("Current Flow Rate: ");
    Serial.print(flowML);
    Serial.print(" mL/min. (±");
    Serial.print(tolerance);
    Serial.print("%), ");
    Serial.print(timer);
    Serial.println(" sec.");

    // Feedback Condition
    if (targetFlow != 0) {
      feedbackPWM(flowML);
    }

    /*--- System Return ---*/
    startTime = currentTime;                                              // Save the start time of the current state
    pulseCount = 0;                                                       // Set pulseCount to 0 ready for calculations
    attachInterrupt(digitalPinToInterrupt(flowPin), getCounter, FALLING); // Reattach interrupt
  } else {
    return;
  }
}

/*--- Functions Definition ---*/
// Interrupt Service Routine (ISR) for Flow Sensor
void getCounter(void) {         
  pulseCount = pulseCount + 1;  // Every falling edge of the sensor signals to increment the pulse count

  return;
}

// Flush Receive Buffer
void flushReceive(void) {
  while (Serial.available()) { // Serial specific 
    Serial.read();
    Serial.flush();
  }

  while (Wire.available()) {   // Wiring specific 
    Wire.read();
  }

  return;
}

// Implementation of I2C Transmission
void i2cTransmit(const byte address, byte data) { 
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();

  return;
}

// Implementation of Water Pressure Calculation
float getwaterPres(float volt) {
  return ((volt - offSet) * ((pgMax - pgMin) / (pgVmax - pgVmin))) + pgMin;
}

// Water Pressure Sensor
void waterPressure(const byte signalPin) {
  for (unsigned int i = 0; i < N; ++i) {    // Get samples with smooth value
    vOut = vOut + analogRead(signalPin);
    delay(1);                               // delay in between reads for stability
  }
  vOut = (vOut * vConv) / N;                // ADC of voltage meter output voltage

  waterPres = getwaterPres(vOut);           // Calculate water pressure

  // Determines whether a value is an infinity, and a number
  if (isinf(waterPres) || isnan(waterPres)) {
    waterPres = -1;
  } else {
    return;
  }
}

// Implementation of Flow Rate Calculation
void flowCal(unsigned int pulse) { // Estimated Volume: 0.5004 ml/Pulse
  flowRate = abs(((-7.0 * pow(10.0, -18.0)) * sq(pulse)) + (0.5004 * pulse) - (8.0 * pow(10.0, -12.0)));
  flowML = flowRate * 60.0; // Milliliter per pulse converter to milliliter per minute

  // Determines whether a value is an infinity, a number or negative
  if (isinf(flowML) || isnan(flowML) || signbit(flowML)) {
    flowML = -1;
  } else {
    return;
  }
}

// Implementation of Flow Rate Feedback-driven Control
void feedbackPWM(float currentFlow) { // Detect current flow rate, and it comparison
  if ((flowML >= flowLL) && (flowML <= flowUL)) {
    return;
  } else {
    flowDev = ((targetFlow - currentFlow) * tolerance) * adjFactor;
    percent = percent + flowDev;

    Serial.print(percent);
    Serial.print(" %, ");
    Serial.print("Target Flow Rate: ");
    Serial.print(targetFlow);
    Serial.print(" mL/min. (");
    Serial.print(flowLL);
    Serial.print(" - ");
    Serial.print(flowUL);
    Serial.print("), ");
    Serial.print("Deviation: ");
    Serial.println(flowDev);

    i2cTransmit(PUMP_CTRL, percent); // Set percentage of PWM
  }
}

// Input Processing and Output Controls
void serialEvent(void) {
  if (Serial.available()) {
    targetFlow = Serial.parseInt(); // Input the flow rate of target to serial port
    if (targetFlow > 0.0) {
      toleranceFlow = (targetFlow / 100.0) * tolerance;
      flowUL = targetFlow + toleranceFlow; // Upper limit of flow rate (mL/min)
      flowLL = targetFlow - toleranceFlow; // Lower limit of flow rate (mL/min)
  
      Serial.print("Set Target Flow Rate: ");
      Serial.print(targetFlow);
      Serial.print(" mL/min. (");
      Serial.print(flowLL);
      Serial.print(" - ");
      Serial.print(flowUL);
      Serial.println(")");
      flushReceive();
    } else if (targetFlow == -1) {
      percent = 0;

      Serial.print("Return to Zero: ");
      Serial.print(percent);
      Serial.println(" %");
      i2cTransmit(PUMP_CTRL, percent);
    }
  } else {
    return;
  }
}
