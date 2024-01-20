#include <Arduino.h>
#include <algorithm>
#include <Timer5.h>
#include <api/Print.h>
#include <LiquidCrystal.h>
using namespace std;

#define DIGITAL_WAVE_COUNTER 0
#define ANALOG_ZERO_CROSS 1
#define ANALOG_SAMPLE_PASSTHROUGH 2

// lcd object
LiquidCrystal lcd(8, 7, 5, 4, 3, 2);

// Global variable for the frequency
float frequency;

// Pin definitions
const u_int8_t interruptPin = 0; // Interrupt pin for the frequency counter
const u_int8_t DACPin = A0;      // DAC pin for output
const u_int8_t ADCPin = A1;      // ADC pin for input
const u_int8_t modePin = 6;      // Pin for switching between operating modes
// DFCR pins for frequency alert LEDs
const u_int8_t REDPLUS = A2;
const u_int8_t YELLOWPLUS = A3;
const u_int8_t GREEN = A4;
const u_int8_t YELLOWMINUS = A5;
const u_int8_t REDMINUS = A6;
const u_int8_t BLUE = 1;
const u_int8_t voltageBaseSwitchPin = 9;

// Digital wave counter variables
const u_int8_t avgSampleLength = 1;        // Number og samples to average over in the running average
volatile float timeArray[avgSampleLength]; // Array of measurement intervals
volatile u_int8_t currentIndex = 0;        // Current index in the time stamp array
volatile float lastTime = 0;               // Time of last interrupt
volatile float lastPeriod = 0;             // Length of last full period
const float digitalSampleRate = 10000;      // ADC sample rate for RMS calculation

// Analog zero cross variables
volatile float prevZeroCross = 0;     // Time of previous zero crossing
volatile int counter = 0;             // Counter for number of samples between zero crossings
volatile int lastCounter = 0;         // Last counter for number of samples between zero crossings
const float analogSampleRate = 15000; // Sample rate for the analog zero cross

// Low pass filter variables
const float cutOffFrequency = 1000;  // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
float alpha;                        // Constant for the low pass filter
float sampleTime;                   // Sample time for the low pass filter in seconds
volatile float OldSample = 0;       // Previous sample for the low pass filter

// Operating mode variables
volatile u_int8_t operatingMode;          // Operating mode for the program
volatile unsigned long switchingTime = 0; // Time of last switch between operating modes
volatile float samrate;                   // Sampling rate for MyTimer5

// RMS variables
volatile float analogSquareSum = 0;                    // Value to incriment in interrupt
volatile float lastsquareSum = 0;                      // Flushed RMS value for full period
volatile float lastVoltage = 0;                        // Time of last voltage measurement
volatile float rmsAnalog = 0;                          // RMS analog value ouput
float rmsVoltage = 0;                                  // RMS voltage value output
const float voltageOffset = 1.65;                       // Voltage offset on the frequency generator
const float analogOffset = voltageOffset / 3.3 * 1023; // Voltage offset converted to analog value
volatile bool useGridVoltage = false;                  // Flag for using grid voltage
volatile unsigned long baseSwitchTime = 0;             // Time of last voltage switch for debounce

// write me a wave counting function

// Function declarations
void timeStamp();
float digitalFrequency();
float analogFrequency();
void resetTimeArray();
void AdcBooster();
void waitMillis(unsigned long ms);
void waitMicros(unsigned long us);
void Timer5_analogZeroCross();
void rmsSum(float voltage, float time);
float rmsCalculation(float period);
float analogToBoardVoltage(float analogValue);
float analogToGridVoltage(float analogValue);
void lcdFrequency(float freq);
void lcdVoltage(float voltage);
void lcdReset();
void setupDigitalWaveCounter();
void setupAnalogZeroCross();
void setupAnalogSamplePassthrough();
void setTimer5(float sampleRate, voidFuncPtr callback);
void switchOperatingMode();
void lowPassFilterSetup(float period);
float lowPassFilter(float newSample, float previousSample);
void FreqAlert(float freq);
u_int8_t lcdDigits(float value);
float analogToVoltage(float analogValue);
void voltageBaseSwitch();

// Initializing

void setup()
{
  Serial.begin(9600); // Serial communication rate
  while (!Serial)
    ; // Wait for serial monitor to open

  Serial.println("Setup started!");

  // ADC and DAC setup
  Serial.println("Setting up ADC and DAC");
  analogWriteResolution(10); // We set the resolution of the DAC to 10 bit
  analogReadResolution(10);  // We set the resolution of the ADC to 10 bit
  AdcBooster();              // We boost the ADC clock frequency to 2 MHz
  // pins
  Serial.println("Setting up pins");
  pinMode(interruptPin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(DACPin, OUTPUT);
  pinMode(ADCPin, INPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOWPLUS, OUTPUT);
  pinMode(YELLOWMINUS, OUTPUT);
  pinMode(REDPLUS, OUTPUT);
  pinMode(REDMINUS, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // Timer5.begin() must be called before Timer5.end can be run in operating mode setup functions
  MyTimer5.begin(1);

  // lcd code
  Serial.println("Setting up LCD");
  lcd.begin(16, 2);

  Serial.println("Setting up interrupts");
  setupDigitalWaveCounter();                                                               // Setup default operating mode
  attachInterrupt(digitalPinToInterrupt(modePin), switchOperatingMode, RISING);            // Setup interrupt for switching operating mode
  attachInterrupt(digitalPinToInterrupt(voltageBaseSwitchPin), voltageBaseSwitch, RISING); // Setup interrupt for switching operating mode
  Serial.println("Setup done!");
}

// Main loop
void loop()
{
  // Main switch case for operating modes
  switch (operatingMode)
  {
  case DIGITAL_WAVE_COUNTER:
    /* This mode calculates the frequency using the time
    between digital RISING interrupts on pin 0.
    The frequency is a running average.*/
    frequency = digitalFrequency();

    // Check frequency against DFCR requirements
    FreqAlert(frequency);

    // Calculate RMS voltage from the last full period
    rmsVoltage = analogToVoltage(rmsCalculation(lastPeriod));

    // Print values to the lcd
    lcdFrequency(frequency);
    lcdVoltage(rmsVoltage);

    delay(500);
    break;

  case ANALOG_ZERO_CROSS:
    /* This mode calculates the frequency using the time
    between analog values crossing the offset value as
    set on the frequency generator. Many analog samples are
    compared to find the zero crossing times.
    The frequency variable is set in interrupt.
    */
    // RMS voltage from the last full period, assuming constant time between samples
    rmsVoltage = analogToVoltage(rmsCalculation(lastCounter * sampleTime));
    // The frequency is compared to the DFCR requirements
    FreqAlert(frequency);

    // Print values to the lcd
    lcdFrequency(frequency);
    lcdVoltage(rmsVoltage);

    delay(500);
    break;

  case ANALOG_SAMPLE_PASSTHROUGH:
    /* This mode passes the analog input signal through
    a low pass filter and outputs it to the DAC.
    The frequency is not calculated in this mode.
    */
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOWPLUS, LOW);
    digitalWrite(YELLOWMINUS, LOW);
    digitalWrite(REDPLUS, LOW);
    digitalWrite(REDMINUS, LOW);

    waitMillis(1000);
    break;
  default:
    // Default case to reset for robustness
    setupDigitalWaveCounter();
    break;
  }
}

// ---------------Frequency functions-----------------//
// Function to calculate the frequency in the digital wave counter mode
float digitalFrequency()
{
  // Create a snapshot of the time stamp array to avoid the array being changed during the calculation
  float timeArraySnapshot[avgSampleLength];
  copy(timeArray, timeArray + avgSampleLength, timeArraySnapshot);

  // The sum of the time differences in the array
  float sum = 0;
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    sum += timeArraySnapshot[i];
  }

  // The frequency is calculated as the inverse of the average time difference
  return 1000000.0f * float(avgSampleLength) / sum;
}

// Function to compare frequency to DFCR requirements and turn on LEDs accordingly
void FreqAlert(float frequency)
{
  if (frequency < 49.75)
  {
    // Low restoration range
    digitalWrite(GREEN, LOW);       // Green turns on
    digitalWrite(YELLOWPLUS, LOW);  // Yellow turns off
    digitalWrite(YELLOWMINUS, LOW); // Yellow turns off
    digitalWrite(REDPLUS, LOW);     // Red turns off
    digitalWrite(REDMINUS, HIGH);   // Red turns off
    digitalWrite(BLUE, LOW);        // Blue turns off
  }
  else if (frequency < 49.9)
  {
    // Low disturbance range
    digitalWrite(GREEN, LOW);        // Green turns on
    digitalWrite(YELLOWPLUS, LOW);   // Yellow turns off
    digitalWrite(YELLOWMINUS, HIGH); // Yellow turns off
    digitalWrite(REDPLUS, LOW);      // Red turns off
    digitalWrite(REDMINUS, LOW);     // Red turns off
    digitalWrite(BLUE, LOW);
  }
  else if (frequency < 50.1)
  {
    // Normal range
    digitalWrite(GREEN, HIGH);      // Green turns on
    digitalWrite(YELLOWPLUS, LOW);  // Yellow turns off
    digitalWrite(YELLOWMINUS, LOW); // Yellow turns off
    digitalWrite(REDPLUS, LOW);     // Red turns off
    digitalWrite(REDMINUS, LOW);    // Red turns off
  }
  else if (frequency < 50.25)
  {
    // High disturbance range
    digitalWrite(GREEN, LOW);       // Green turns on
    digitalWrite(YELLOWPLUS, HIGH); // Yellow turns off
    digitalWrite(YELLOWMINUS, LOW); // Yellow turns off
    digitalWrite(REDPLUS, LOW);     // Red turns off
    digitalWrite(REDMINUS, LOW);    // Red turns off
    digitalWrite(BLUE, HIGH);
  }
  else
  {
    // Over restoration range
    digitalWrite(GREEN, LOW);       // Green turns on
    digitalWrite(YELLOWPLUS, LOW);  // Yellow turns off
    digitalWrite(YELLOWMINUS, LOW); // Yellow turns off
    digitalWrite(REDPLUS, HIGH);    // Red turns off
    digitalWrite(REDMINUS, LOW);    // Red turns off
    digitalWrite(BLUE, HIGH);
  }
}

// ---------------Low pass filter functions-----------------//
// Main low pass filter function
float lowPassFilter(float newSample, float previousSample)
{
  float output = newSample * alpha + previousSample * (1 - alpha);
  return output;
}

// Function to calculate the alpha constant for the filter
void lowPassFilterSetup(float period)
{
  float RC = 1 / (2 * pi * cutOffFrequency); // We calculate the time constant for the low pass filter
  alpha = period / (sampleTime + RC);        // We calculate the constant for the low pass filter
}

// ---------------Wait functions as alternative to delay()----------------//
void waitMillis(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
    ;
}
void waitMicros(unsigned long us)
{
  unsigned long start = micros();
  while (micros() - start < us)
    ;
}

// ---------------RMS functions----------------//
/* Function to sum the square of the read analog values
 The sum is a dicrete time integral of the square of the analog values
 Assumes time may differ between measurement*/
void rmsSum(float value, float time)
{
  // Index analog values so offset is 0
  value -= analogOffset;
  // Discrete time integral of the square of the analog values
  analogSquareSum += value * value * time;
}

// Function to calculate the RMS value from the sum of the squares over the time period
float rmsCalculation(float period)
{
  float rms = sqrt(lastsquareSum / period);
  return rms;
}

//--------------------------Analog to voltage--------------------------//
float analogToVoltage(float analogValue)
{
  if (useGridVoltage)
  {
    return analogToGridVoltage(analogValue);
  }
  else
  {
    return analogToBoardVoltage(analogValue);
  }
}

// Converts 0-1023 analog value to 0-3.3V
float analogToBoardVoltage(float analogValue)
{
  return (analogValue / 1023) * 3.3;
}

// Converts 0-1023 analog value to 0-340V
float analogToGridVoltage(float analogValue)
{
  return 680 * ((analogValue / 1023));
}

//--------------------------LCD functions--------------------------//
// Prints frequency valueson first line of LCD
void lcdFrequency(float freq)
{
  lcd.setCursor(6, 0);
  lcd.print(freq, lcdDigits(freq));
}

// Prints voltage values on second line of LCD
void lcdVoltage(float voltage)
{
  lcd.setCursor(6, 1);
  lcd.print(voltage, lcdDigits(voltage));
}

// Resets LCD to titles on both lines
void lcdReset()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  lcd.setCursor(13, 0);
  lcd.print("Hz");
  lcd.setCursor(0, 1);
  lcd.print("RMS: ");
  lcd.setCursor(13, 1);
  lcd.print("V");
}

// Function to calculate number of digits to print on LCD
u_int8_t lcdDigits(float value)
{
  // Base number of digits
  u_int8_t digits = 4;
  while (value >= 10)
  { // Subtract a digit for each order of magnitude
    value /= 10;
    if (--digits == 0)
    { // Break if we reach 0 digits
      break;
    }
  }
  return digits;
}

//--------------------------ISR functions--------------------------//
// ISR to rotate operating mode
void switchOperatingMode()
{

  if (millis() - switchingTime < 1000)
  {
    // debounce
    return;
  }

  switch (operatingMode)
  {
  // Mode switching
  case DIGITAL_WAVE_COUNTER:
    setupAnalogZeroCross();
    break;
  case ANALOG_ZERO_CROSS:
    setupAnalogSamplePassthrough();
    break;
  case ANALOG_SAMPLE_PASSTHROUGH:
    setupDigitalWaveCounter();

  default:
    setupDigitalWaveCounter();
    break;
  }
  switchingTime = millis();
}

// Main ISR for digital wave counter to calculate intervals between interrupts
void timeStamp()
{
  float currentTime = micros();
  timeArray[currentIndex] = currentTime - lastTime;               // Save time since last interrupt
  rmsSum(analogRead(ADCPin), (currentTime - lastVoltage) * 1e6f); // Last voltage measurement before flushing the sum
  lastsquareSum = analogSquareSum;                                // Flush sum of squares of full period for RMS calculation
  lastPeriod = (currentTime - lastTime) * 1e6f;                   // Flush full period time for RMS calculation
  analogSquareSum = 0;                                            // Reset sum of squares for next period
  lastTime = currentTime;                                         // Update last interrupt time
  if (++currentIndex == avgSampleLength)                          // Reset index when the array is full
  {
    currentIndex = 0;
  }
}

// Main ISR for analog zero cross to calculate frequency
void Timer5_analogZeroCross()
{
  float Newtimestamp = micros();
  float unfilteredSample = analogRead(ADCPin);                  // Analog sample without low pass filter
  float newSample = lowPassFilter(unfilteredSample, OldSample); // Analog sample through low pass filter
  rmsSum(unfilteredSample, sampleTime);                         // Send sample to RMS calculation
  counter++;                                                    // Increment counter for number of samples between zero crossings

  // Detects zero crossing if the new sample is above the offset and the old sample is below the offset
  if (newSample > analogOffset && OldSample < analogOffset)
  {

    float zeroCross = Newtimestamp - (newSample / (newSample - OldSample)); // Calculates time stamp for zero crossing
    float interval = zeroCross - prevZeroCross;                             // Time since last zero crossing
    frequency = (1 / interval) * 1e6;                                       // Calculates frequency as inverse of interval
    prevZeroCross = zeroCross;                                              // Updates previous zero crossing

    // Flush RMS variables for full period
    lastsquareSum = analogSquareSum;
    lastCounter = counter;

    // Reset RMS variables for next period
    analogSquareSum = 0;
    counter = 0;
  }
  OldSample = newSample; // Updates old sample
}

// Auxiliary ISR for digital wave counter to calculate RMS voltage
void Timer5_digitalWaveCounter()
{
  float now = micros();
  // Send ADC value and time since last interrupt to RMS calculation
  rmsSum(analogRead(ADCPin), (now - lastVoltage) * 1e6f);
  lastVoltage = now;
}

// Main ISR for analog sample passthrough to pass signal from ADC to DAC
void Timer5_analogSamplePassthrough()
{
  // Sample through low pass filter
  float newSample = lowPassFilter(analogRead(ADCPin), OldSample);
  // Output to DAC
  analogWrite(DACPin, newSample);
  OldSample = newSample;
}

void voltageBaseSwitch()
{
  unsigned long now = millis();
  if (millis() - baseSwitchTime < 500)
  {
    // debounce
    return;
  }
  useGridVoltage = !useGridVoltage;
  baseSwitchTime = now;
}

//--------------------------Mode setup functions--------------------------//
void setupDigitalWaveCounter()
{
  // Setup for digital wave counter operating mode
  Serial.println("Setting up digital wave counter");
  operatingMode = DIGITAL_WAVE_COUNTER;

  Serial.println("Resetting");
  resetTimeArray();
  lcdReset();
  lcd.setCursor(15, 1);
  lcd.print("D");
  // We set our interrupt to trigger the interupt function when value reaches HIGH
  Serial.println("Setting up interrupt");
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING);

  // We set our timer to measure voltage for RMS
  Serial.println("Setting up timer");
  setTimer5(digitalSampleRate, Timer5_digitalWaveCounter);

  Serial.println("Digital wave counter setup done!");
}

void setupAnalogZeroCross()
{
  // Setup for analog zero cross operating mode
  Serial.println("Setting up analog zero cross");
  operatingMode = ANALOG_ZERO_CROSS;
  // Detaching interrupt to avoid disturbance
  Serial.println("Resetting");
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  lcdReset();
  lcd.setCursor(15, 1);
  lcd.print("A");
  // Setting timer to analog sample rate and attaching main ISR
  Serial.println("Setting up timer");
  setTimer5(analogSampleRate, Timer5_analogZeroCross);
  Serial.println("Analog zero cross setup done!");
}

void setupAnalogSamplePassthrough()
{
  // Setup for analog sample passthrough operating mode
  Serial.println("Setting up analog sample passthrough");
  operatingMode = ANALOG_SAMPLE_PASSTHROUGH;

  Serial.println("Resetting");
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("See Oscilloscope");
  lcd.setCursor(15, 1);
  lcd.print("P");

  // Setting timer to analog sample rate and attaching interrupt
  Serial.println("Setting up timer");
  setTimer5(analogSampleRate, Timer5_analogSamplePassthrough);
  Serial.println("Analog sample passthrough setup done!");
}

// Function to reset the timer and set the interrupt
void setTimer5(float sampleRate, voidFuncPtr callback)
{
  // stop the timer so it can be reconfigured
  MyTimer5.end();
  Serial.println("Timer ended");

  // set the sampling rate for low pass filter
  samrate = sampleRate;
  sampleTime = 1.0f / sampleRate;
  lowPassFilterSetup(sampleTime);
  Serial.println("Low pass filter setup done");

  // define frequency of interrupt
  MyTimer5.begin(sampleRate);
  Serial.println("Timer begun");
  // define the interrupt callback function
  MyTimer5.attachInterrupt(callback);
  Serial.println("Interrupt attached");
  // start the timer
  MyTimer5.start();
  Serial.println("Timer started");
}

// Function to reset the time stamp array to 0
void resetTimeArray()
{
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    timeArray[i] = 0;
  }
}

//--------------------------ADC clock booster--------------------------//
void AdcBooster()
{
  ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;                                            // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 |   // Divide Clock by 16.
                   ADC_CTRLB_RESSEL_10BIT;       // Result on 10 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ; // Wait for synchronization
}
