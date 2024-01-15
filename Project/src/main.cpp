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
const int interruptPin = 0; // Interrupt pin for the frequency counter
const int DACPin = A0;      // DAC pin for output
const int ADCPin = A1;      // ADC pin for input
const int modePin = 6;      // Pin for switching between operating modes

// Digital wave counter variables
const unsigned int avgSampleLength = 3;    // Number og samples to average over in the running average
volatile float timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
volatile unsigned int currentIndex = 0;
volatile float lastTime = 0;
volatile float lastPeriod = 0;
const float digitalSampleRate = 1000; // Sample rate for the digital wave counter

// Analog zero cross variables
volatile float prevZeroCross = 0;
volatile int counter = 0;
volatile int lastCounter = 0;
const float analogSampleRate = 10000; // Sample rate for the analog zero cross

// Low pass filter variables
const float cutOffFrequency = 100;  // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
float alpha;                        // Constant for the low pass filter
float sampleTime;                   // Sample time for the low pass filter in seconds
volatile float OldSample = 0;       // Creating val for low pass filter

// Operating mode variables
volatile byte operatingMode;              // Operating mode for the program
volatile unsigned long switchingTime = 0; // Time of last switch between operating modes
volatile float samrate;                   // Sampling rate for MyTimer5

const int GREEN = A2; // Creating val for frequency alert
const int YELLOW = A3;
const int RED = A4;

// RMS variables
volatile float analogSquareSum = 0;                  // Creating val for RMS calculation
volatile float lastsquareSum = 0;                    //
volatile float lastVoltage = 0;                      // Time of last interrupt
volatile float rmsAnalog = 0;                        // RMS analog value ouput
volatile float rmsVoltage = 0;                       // RMS voltage ouput
const float voltageOffset = 1.65;                    // Voltage offset for the board
const int analogOffset = voltageOffset / 3.3 * 1023; // Analog offset for the board

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

// Initializing

void setup()
{
  analogWriteResolution(10); // We set the resolution of the DAC to 10 bit
  analogReadResolution(10);  // We set the resolution of the ADC to 10 bit
  Serial.begin(9600);        // Serial communication rate
  while (!Serial)
    ; // Wait for serial monitor to open

  Serial.println("Setup started!");
  AdcBooster(); // We boost the ADC clock frequency to 2 MHz

  // pins
  Serial.println("Setting up pins");
  pinMode(interruptPin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(DACPin, OUTPUT);
  pinMode(ADCPin, INPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);

  MyTimer5.begin(1);

  // lcd code
  Serial.println("Setting up LCD");
  lcd.begin(16, 2);
  lcdReset();

  Serial.println("Setting up interrupts");
  setupDigitalWaveCounter();                                                    // We set the initial operating mode to digital wave counter
  attachInterrupt(digitalPinToInterrupt(modePin), switchOperatingMode, RISING); // We set the interrupt for switching operating modes
  Serial.println("Setup done!");
}

// Main loop
void loop()
{
  switch (operatingMode)
  {
  case DIGITAL_WAVE_COUNTER:
    // We calculate the frequency using the running average
    frequency = digitalFrequency();
    lcdFrequency(frequency);
    FreqAlert(frequency);
    rmsVoltage = analogToBoardVoltage(rmsCalculation(lastPeriod));

    lcdVoltage(rmsVoltage);
    // FreqAlert(frequency);
    // digitalWrite(kalibrering, LOW); //// skal måske fjernes, ikke testet endnu
    //  We wait 0.5 second before calculating the frequency
    delay(500);
    break;

  case ANALOG_ZERO_CROSS:
    // We calculate the frequency using the running average
    rmsVoltage = analogToBoardVoltage(rmsCalculation(lastCounter * sampleTime));
    // We print the frequency to the serial monitor
    lcdFrequency(frequency);
    lcdVoltage(rmsVoltage);
    FreqAlert(frequency);

    // FreqAlert(frequency);
    // digitalWrite(kalibrering, LOW); //// skal måske fjernes, ikke testet endnu
    delay(500);
    break;
  case ANALOG_SAMPLE_PASSTHROUGH:
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, LOW);
    digitalWrite(RED, LOW);

    // lcdVoltage(OldSample);
    waitMillis(1000);
    break;
  default:
    // Default case to reset for security
    setupDigitalWaveCounter();
    break;
  }
}

// ---------------Frequency calculation functions-----------------//
// Function that calculates the running average of the frequency
float digitalFrequency()
{
  // We create a snapshot of the time stamp array to avoid the array being changed during the calculation
  unsigned long timeArraySnapshot[avgSampleLength];
  copy(timeArray, timeArray + avgSampleLength, timeArraySnapshot);

  // The sum of the time differences in the array
  unsigned long sum = 0;
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    sum += timeArraySnapshot[i];
  }

  // The frequency is calculated as the inverse of the average time difference
  return float(1000000) * avgSampleLength / sum;
}

// ---------------Low pass filter functions-----------------//
float lowPassFilter(float newSample, float previousSample)
{
  float output = newSample * alpha + previousSample * (1 - alpha);
  return output;
}

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
void rmsSum(float value, float time)
{
  value -= analogOffset;
  analogSquareSum += value * value * time;
}

float rmsCalculation(float period)
{
  float rms = sqrt(lastsquareSum / period);
  return rms;
}

//--------------------------Analog to voltage--------------------------//
float analogToBoardVoltage(float analogValue)
{
  return (analogValue / 1023) * 3.3;
}

float analogToGridVoltage(float analogValue)
{
  return 340 * ((analogValue / 1023));
}

//--------------------------LCD functions--------------------------//
void lcdFrequency(float freq)
{
  lcd.setCursor(6, 0);
  lcd.print(freq, 4);
  lcd.setCursor(14, 0);
  lcd.print("Hz");
}

void lcdVoltage(float voltage)
{
  lcd.setCursor(6, 1);
  lcd.print(voltage, 4);
  lcd.setCursor(15, 1);
  lcd.print("V");
}

void lcdReset()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  lcd.setCursor(0, 1);
  lcd.print("RMS: ");
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

// ISR that updates the time stamp array every time the interrupt pin goes HIGH
void timeStamp()
{
  float currentTime = micros();
  timeArray[currentIndex] = currentTime - lastTime;               // Save current time stamp in the array
  rmsSum(analogRead(ADCPin), (currentTime - lastVoltage) * 1e6f); // Last voltage measurement
  lastsquareSum = analogSquareSum;
  lastPeriod = (currentTime - lastTime) * 1e6f;
  analogSquareSum = 0;
  lastTime = currentTime;
  if (++currentIndex == avgSampleLength) // Reset when the array is full
  {
    currentIndex = 0;
  }
}

void Timer5_analogZeroCross()
{
  float Newtimestamp = micros();

  float newSample = lowPassFilter(analogRead(ADCPin), OldSample); // Data through low pass filter
  rmsSum(newSample, sampleTime);
  counter++;

  if (newSample > analogOffset && OldSample < analogOffset)
  { // Detects zero crossing

    float zeroCross = Newtimestamp - (newSample / (newSample - OldSample)); // Calculates time stamp for zero crossing
    float interval = zeroCross - prevZeroCross;
    frequency = (1 / interval) * 1e6; // Calculates frequency
    prevZeroCross = zeroCross;        // Updates previous zero crossing
    lastsquareSum = analogSquareSum;
    lastCounter = counter;
    analogSquareSum = 0;
    counter = 0;
  }
  OldSample = newSample; // Updates old sample
}

void Timer5_digitalWaveCounter()
{
  unsigned long now = micros();
  rmsSum(analogRead(ADCPin), float(now - lastVoltage) * 1e6f);
  lastVoltage = now;
}

void Timer5_analogSamplePassthrough()
{
  volatile float newSample = lowPassFilter(analogRead(ADCPin), OldSample);
  OldSample = newSample;
  analogWrite(DACPin, newSample);
}

//--------------------------Mode setup functions--------------------------//
void setupDigitalWaveCounter()
{
  // Setup for digital wave counter
  operatingMode = DIGITAL_WAVE_COUNTER;
  Serial.println("Resetting arrays");
  resetTimeArray();
  lcdReset();
  // We set our interrupt to trigger the interupt function when value reaches HIGH
  Serial.println("Setting up interrupt");
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING);
  Serial.println("Setting up timer");
  setTimer5(digitalSampleRate, Timer5_digitalWaveCounter);
}

void setupAnalogZeroCross()
{
  // Setup for analog zero cross
  operatingMode = ANALOG_ZERO_CROSS;
  // Detaching interrupt
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  lcdReset();
  setTimer5(analogSampleRate, Timer5_analogZeroCross);
}

void setupAnalogSamplePassthrough()
{
  // Setup for analog sample passthrough
  operatingMode = ANALOG_SAMPLE_PASSTHROUGH;
  // Detaching interrupt and resetting lcd
  detachInterrupt(digitalPinToInterrupt(interruptPin));

  // Setting timer to analog sample rate and attaching interrupt
  setTimer5(analogSampleRate, Timer5_analogSamplePassthrough);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("See Oscilloscope");
}

void setTimer5(float sampleRate, voidFuncPtr callback)
{
  MyTimer5.end();
  Serial.println("Timer ended");
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
// Part 10, frequency alert where the LED turns on and off in an interval
void FreqAlert(float frequency)
{
  if (frequency > 49.9 && frequency < 50.1 && frequency != 0)
  {
    //
    digitalWrite(GREEN, HIGH); // Green turns on
    digitalWrite(YELLOW, LOW); // Yellow turns off
    digitalWrite(RED, LOW);    // Red turns off
  }
  else if ((frequency <= 49.9 && frequency > 49.75) || (frequency >= 50.1 && frequency < 50.25))
  {
    //
    digitalWrite(GREEN, LOW);   // Green turns off
    digitalWrite(YELLOW, HIGH); // Yellow turns on
    digitalWrite(RED, LOW);     // Red turns off
  }
  else if (frequency <= 49.75 || frequency >= 50.25)
  {
    //
    digitalWrite(GREEN, LOW);  // Green turns off
    digitalWrite(YELLOW, LOW); // Yellow turns off
    digitalWrite(RED, HIGH);   // Red turns on
  }
  else
  {
  }
}