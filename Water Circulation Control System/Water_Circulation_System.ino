/*
  Water Circulation System
      Use interrupts to calculate the number of unit pulses per interval to convert the water flow rate (mL/minute).


  Created 28 Aug. 2020
  by Yi-Xuan Wang

  References:
  https://wiki.seeedstudio.com/Water-Flow-Sensor/
*/

/*--- Preprocessor ---*/
#define pumpPin 11    // The pin location of the pump (~D11) w/ PWM
#define flowPin_in 2  // The pin location of the sensor (D2) w/ interrupt (INT.0)
#define flowPin_out 3 // The pin location of the sensor (D3) w/ interrupt (INT.1)

/*--- Constant ---*/
const unsigned long baudSpeed = 115200; // Sets the data rate in bits per second (baud) for serial data transmission
const unsigned long period = 1000;      // The value is a number of milliseconds

const byte pwm = 255;                   // Full speed operation

/*--- Global Variables ---*/
unsigned long startTime;              // Start time
unsigned long currentTime;            // Current time

volatile unsigned int pulseCount_in;  // Measuring the falling edges of the signal
volatile unsigned int pulseCount_out; // Measuring the falling edges of the signal

float flowRate;

/*--- Functions ---
 *  countISR_*(), function measures the rising and falling edge of the hall effect sensors signal
 *  getFlowrate(unsigned int *), calculate the flow rate from liquid flow meter
 */

void countISR_in() { // Insterrupt Service Rountine (ISR) for input interrrupt
    pulseCount_in = pulseCount_in + 1;
}

void countISR_out() { // Insterrupt Service Rountine (ISR) for output interrupt
    pulseCount_out = pulseCount_out + 1;
}

float getFlowrate(unsigned int pulseCount) {
  flowRate = ((pulseCount * 2.25) * 60);      // Each pulse is approximately 2.25 milliliters

  if (isinf(flowRate) || isnan(flowRate) || (flowRate <= 0.0)) {
    flowRate = -1;
  }

  return flowRate;
}

/*--- Initialization ---*/
void setup(void) {
  Serial.begin(baudSpeed);            // Sets serial port baud to 9600 bps
  pinMode(flowPin_in, INPUT_PULLUP);  // Initializes interrupt D2 declared as an input and pull up resitor is enabled
  pinMode(flowPin_out, INPUT_PULLUP); // Initializes interrupt D3 declared as an input and pull up resitor is enabled
  pinMode(pumpPin, OUTPUT);           // initializes digital pin 11 declared as an output

  // Flow Sensor Initialization
  pulseCount_in = 0;
  pulseCount_out = 0;
  flowRate = 0.0;

  analogWrite(pumpPin, pwm);
  Serial.print("Pump start up, PWM = ");
  Serial.println(pwm);

  attachInterrupt(digitalPinToInterrupt(flowPin_in), countISR_in, FALLING);   // D2 interrupt is attached
  attachInterrupt(digitalPinToInterrupt(flowPin_out), countISR_out, FALLING); // D3 interrupt is attached
}

/*--- Operating ---*/
void loop(void) {
  // Every second, calculate and print millilitres/minute
  currentTime = millis();                     // Get the current "time"

  if ((currentTime - startTime) >= period) {  // Test whether the period has elapsed
    // Clears the function used to attend a specific interrupt
    detachInterrupt(digitalPinToInterrupt(pulseCount_in));  
    detachInterrupt(digitalPinToInterrupt(pulseCount_out));

    Serial.print("Input flow rate:");
    Serial.print(getFlowrate(pulseCount_in), 2);  // Prints the number calculated above
    Serial.print(" mL/min\r\t");                  // Prints "mL/minute" and returns a new line
    Serial.print("Output flow rate:");
    Serial.print(getFlowrate(pulseCount_out), 2);
    Serial.println(" mL/min\r");

    // Set pulseCount to 0 ready for calculations
    pulseCount_in = 0;
    pulseCount_out = 0;

    // Reattach interrupt
    attachInterrupt(digitalPinToInterrupt(pulseCount_in), countISR_in, FALLING); 
    attachInterrupt(digitalPinToInterrupt(pulseCount_out), countISR_out, FALLING);
  } else {
    return;
  }
}
