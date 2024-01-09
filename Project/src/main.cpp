#include <Arduino.h>
#include <algorithm>
#include <Timer5.h>
#include <arduinoFFT.h>
using namespace std;

const unsigned int avgSampleLength = 3;            // Number og samples to average over in the running average
volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
double frequency;                                  // Global variable for the frequency
unsigned int currentIndex = 0;                     // We create the empty time stamp array used for the interrupt
const unsigned long ulongMax = 4294967295;         // Maximum value of unsigned long
const unsigned long ulongThreashold = 4000000000;  // Threshold for when the time stamp array should be reset
unsigned long lastTime = 0;

const int sampleTime = 5;
const double cutOffFrequency = 50;  // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
double alpha;                       // Constant for the low pass filter

const int interruptPin = 0; // Interrupt pin for the frequency counter
const int DACPin = A0;      // DAC pin for output
const int ADCPin = A1;      // ADC pin for input

const unsigned long ulongThreasholdTest = 0; // Temp test value

// FFT
arduinoFFT FFT;
const u_int16_t samples = 1048576;        // Must be a power of 2
const double samplingFrequency = 1048576; // Hz
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];

// Function declarations
void timeStamp();
double averageFrequency();
void resetTimeArray();
void AdcBooster();
void waitMillis(unsigned long ms);
void waitMicros(unsigned long us);

void setup()
{

  Serial.begin(9600); // Serial communication rate
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    // We initialize the time stamp array to 0
    timeArray[i] = 0;
  }
  // pins
  pinMode(interruptPin, INPUT);
  pinMode(DACPin, OUTPUT);
  pinMode(ADCPin, INPUT);
  // attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING); // We set our interrupt to trigger the interupt function when value reaches HIGH
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  double RC = 1 / (2 * pi * cutOffFrequency); // We calculate the time constant for the low pass filter
  alpha = sampleTime / (sampleTime + RC);     // We calculate the constant for the low pass filter
  AdcBooster();                               // We boost the ADC clock frequency to 2 MHz

  while (!Serial)
    ; // Wait for serial monitor to open
  Serial.println("Setup done!");
}

void loop()
{
  microseconds = micros();
  for (unsigned int i = 0; i < samples; i++)
  {
    vReal[i] = analogRead(ADCPin);
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us)
      ;
    microseconds += sampling_period_us;
  }

  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);
  // FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  double peak = FFT.MajorPeak();
  Serial.println(peak, 6);
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

// Function that calculates the running average of the frequency
double averageFrequency()
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

// Debug function to reset the time stamp array to 0
void resetTimeArray()
{
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    timeArray[i] = 0;
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING);
}

// Low pass filter function
float lowPassFilter(float newSample, float OldSample)
{
  float output = newSample * alpha + OldSample * (1 - alpha);
  return output;
}

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
} // AdcBooster

// Wait functions as alternative to delay()
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