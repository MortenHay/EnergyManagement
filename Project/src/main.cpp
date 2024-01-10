#include <Arduino.h>
#include <algorithm>
#include <Timer5.h>
#include <api/Print.h>
#include <LiquidCrystal.h>
using namespace std;

#define DIGITAL_WAVE_COUNTER 0
#define ANALOG_ZERO_CROSS 1

// Tester om lcd kan bruges
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int switchState = 0;

const unsigned int avgSampleLength = 3; // Number og samples to average over in the running average

volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
double frequency;                                  // Global variable for the frequency
volatile unsigned int currentIndex = 0;            // We create the empty time stamp array used for the interrupt
volatile unsigned long lastTime = 0;

const double cutOffFrequency = 100; // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
double alpha;                       // Constant for the low pass filter

const int interruptPin = 0; // Interrupt pin for the frequency counter
const int DACPin = A0;      // DAC pin for output
const int ADCPin = A1;      // ADC pin for input
const int modePin = 6;      // Pin for switching between operating modes
const int switchPin = 6;

volatile double val = 0;              // Creating val for analog read
const float digitalSampleRate = 1000; // Sample rate for the digital wave counter
const float analogSampleRate = 10000; // Sample rate for the analog zero cross
float Samrate;                        // Sampling rate for MyTimer5
volatile double Amplitude = 511;      // Creating val for analog read
volatile double crosstimeN = 50;      // Creating a val for the number of zero crossings bore calculation
double freq = 0;                      // Creating val for frequency
volatile float newSample = 0;         // Creating val for low pass filter
volatile float OldSample = 0;         // Creating val for low pass filter
float sampleTime;                     // Sample time for the low pass filter in seconds
volatile double zerocrosstime = 0;
volatile int counter = 0;

volatile int voltageSquareSum = 0; // Creating val for RMS calculation

volatile byte operatingMode;              // Operating mode for the program
volatile unsigned long switchingTime = 0; // Time of last switch between operating modes
volatile unsigned long lastVoltage = 0;   // Time of last interrupt

// Function declarations
void timeStamp();
double digitalFrequency();
double analogFrequency();
void resetTimeArray();
void AdcBooster();
void waitMillis(unsigned long ms);
void waitMicros(unsigned long us);
void Timer5_analogZeroCross();
void rmsSum(float voltage, unsigned long time);

// Initializing
void setup()
{
  Serial.begin(9600); // Serial communication rate

  AdcBooster();              // We boost the ADC clock frequency to 2 MHz
  analogWriteResolution(10); // We set the resolution of the DAC to 10 bit
  analogReadResolution(10);  // We set the resolution of the ADC to 10 bit

  // pins
  pinMode(interruptPin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(DACPin, OUTPUT);
  pinMode(ADCPin, INPUT);
  pinMode(switchPin, INPUT);

  // lcd code
  lcd.begin(16, 2);
  lcdReset();

  setupDigitalWaveCounter(); // We set the initial operating mode to digital wave counter
  while (!Serial)
    ; // Wait for serial monitor to open

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
    // We print the frequency to the serial monitor
    Serial.println(frequency);
    lcdFrequency(frequency);
    // We wait 0.5 second before calculating the frequency
    waitMillis(500);
    break;

  case ANALOG_ZERO_CROSS:
    // We calculate the frequency using the running average
    if (zerocrosstime >= (crosstimeN - 1))
    {
      frequency = analogFrequency();
      // We print the frequency to the serial monitor
      Serial.println(frequency);
      lcdFrequency(frequency);
    }
    break;

  default:
    // Default case to reset for security
    setupDigitalWaveCounter();
    break;
  }

  // waitMillis(500); // We wait 0.5 second before calculating the frequency
  //  print and calculation of frequency

  // If function for analog calculating frequency
  // 0.9739943508327652 factor for 1000Hz
  // 1.087040767 factor for 10000Hz
  /*   if (zerocrosstime >= (crosstimeN - 1))
    {

      freq = (Samrate * (crosstimeN - 1) / (counter));

      counter = 0;
      zerocrosstime = 0;

      // Serial.println(freq);
      switchState = digitalRead(switchPin);
      if (switchState)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Value: ");
        lcd.setCursor(0, 1);
        lcd.print(freq, 5);
      }
    } */
}

// ---------------Frequency calculation functions-----------------//
// Function that calculates the running average of the frequency
double digitalFrequency()
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
  return double(1000000) * avgSampleLength / sum;
}

double analogFrequency()
{
  counter = 0;
  zerocrosstime = 0;
  return (Samrate * (crosstimeN - 1) / (counter));
}

// ---------------Low pass filter functions-----------------//
float lowPassFilter(float newSample, float previousSample)
{
  float output = newSample * alpha + previousSample * (1 - alpha);
  return output;
}

void lowPassFilterSetup(double period)
{
  double RC = 1 / (2 * pi * cutOffFrequency); // We calculate the time constant for the low pass filter
  alpha = period / (sampleTime + RC);         // We calculate the constant for the low pass filter
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
void rmsSum(float voltage, unsigned long time)
{
  voltageSquareSum += voltage * voltage * time;
}

float rmsCalculation(unsigned long period)
{
  float rms = sqrt(voltageSquareSum / period);
  voltageSquareSum = 0;
  return rms;
}

//--------------------------LCD functions--------------------------//
void lcdFrequency(double freq)
{
  lcd.setCursor(6, 0);
  lcd.print(freq, 5);
}

void lcdVoltage(float voltage)
{
  lcd.setCursor(6, 1);
  lcd.print(voltage, 5);
}

void lcdReset()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  lcd.setCursor(0, 1);
  lcd.print("Volt: ");
}

//--------------------------ISR functions--------------------------//
// ISR to rotate operating mode
void switchOperatingMode()
{

  if (millis() - switchingTime < 500)
  {
    // debounce
    return;
  }

  switch (operatingMode)
  {
  // Mode switching
  case DIGITAL_WAVE_COUNTER:
    operatingMode = ANALOG_ZERO_CROSS;
    break;
  case ANALOG_ZERO_CROSS:
    setupDigitalWaveCounter();
    break;
  default:
    setupDigitalWaveCounter();
    break;
  }
  switchingTime = millis();
}

// ISR that updates the time stamp array every time the interrupt pin goes HIGH
void timeStamp()
{
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  timeArray[currentIndex] = deltaTime; // Save current time stamp in the array
  lastTime = currentTime;
  if (++currentIndex == avgSampleLength) // Reset when the array is full
  {
    currentIndex = 0;
  }
}

void Timer5_analogZeroCross()
{
  newSample = lowPassFilter(analogRead(ADCPin), OldSample);
  rmsSum(newSample, sampleTime);
  counter++;

  if (newSample >= Amplitude / 2 && OldSample < Amplitude / 2)
  {
    zerocrosstime++;
  }

  OldSample = newSample;

  // analogWrite(DACPin,newSample);
}

void Timer5_digitalWaveCounter()
{
  lastVoltage = micros();
  rmsSum(analogRead(ADCPin), sampleTime);
}

//--------------------------Mode setup functions--------------------------//
void setupDigitalWaveCounter()
{
  // Setup for digital wave counter
  operatingMode = DIGITAL_WAVE_COUNTER;
  // The timeArray is reset to 0
  resetTimeArray();
  // We set our interrupt to trigger the interupt function when value reaches HIGH
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING);
  setTimer5(digitalSampleRate, Timer5_digitalWaveCounter);
}

void setupAnalogZeroCross()
{
  // Setup for analog zero cross
  operatingMode = ANALOG_ZERO_CROSS;
  // We set our interrupt to trigger the interupt function when value reaches HIGH
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  counter = 0;
  setTimer5(analogSampleRate, Timer5_analogZeroCross);
}

void setTimer5(float sampleRate, voidFuncPtr callback)
{
  MyTimer5.end();

  Samrate = sampleRate;
  sampleTime = 1 / sampleRate;

  lowPassFilterSetup(sampleTime);

  // define frequency of interrupt
  MyTimer5.begin(sampleRate); // 200=for toggle every 5msec

  // define the interrupt callback function
  MyTimer5.attachInterrupt(Timer5_analogZeroCross);

  // start the timer
  MyTimer5.start();
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