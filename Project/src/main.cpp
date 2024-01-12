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
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Global variable for the frequency
double frequency;

// Pin definitions
const int interruptPin = 0; // Interrupt pin for the frequency counter
const int DACPin = A0;      // DAC pin for output
const int ADCPin = A1;      // ADC pin for input
const int modePin = 6;      // Pin for switching between operating modes
const int switchPin = 6;

// Digital wave counter variables
const unsigned int avgSampleLength = 10;           // Number og samples to average over in the running average
volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
volatile unsigned int currentIndex = 0;            // We create the empty time stamp array used for the interrupt
volatile unsigned long lastTime = 0;
const float digitalSampleRate = 1000; // Sample rate for the digital wave counter

// Analog zero cross variables
volatile double Amplitude = 511; // Creating val for analog read
volatile double crosstimeN = 50; // Creating a val for the number of zero crossings bore calculation

volatile float OldSample = 0; //Creating val for low pass filter
const float sampleTime = 1/Samrate;   // Sample time for the low pass filter in seconds
volatile double zerocrosstime = 0;
volatile int counter = 0;
const float analogSampleRate = 1000; // Sample rate for the analog zero cross

// Low pass filter variables
const double cutOffFrequency = 100; // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
double alpha;                       // Constant for the low pass filter
float sampleTime;                   // Sample time for the low pass filter in seconds
volatile float OldSample = 0;       // Creating val for low pass filter


// Operating mode variables
volatile byte operatingMode;              // Operating mode for the program
volatile unsigned long switchingTime = 0; // Time of last switch between operating modes
volatile float samrate;                   // Sampling rate for MyTimer5

const int kalibrering = 9;
const int freqAlert = 7; //Creating val for frequency alert



// RMS variables
volatile float analogSquareSum = 0;     // Creating val for RMS calculation
volatile unsigned long lastVoltage = 0; // Time of last interrupt
volatile float rmsAnalog = 0;           // RMS voltage ouput
volatile float rmsVoltage = 0;          // RMS voltage ouput
const float voltageOffset = 1.65;       // Voltage offset for the board

// write me a wave counting function

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
float rmsCalculation(unsigned long period);
float analogToBoardVoltage(float analogValue);
float analogToGridVoltage(float analogValue);
void lcdFrequency(double freq);
void lcdVoltage(float voltage);
void lcdReset();
void setupDigitalWaveCounter();
void setupAnalogZeroCross();
void setupAnalogSamplePassthrough();
void setTimer5(float sampleRate, voidFuncPtr callback);
void switchOperatingMode();

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
  pinMode(freqAlert, OUTPUT);
  pinMode(kalibrering, OUTPUT);

  MyTimer5.begin(1);

  // lcd code
  Serial.println("Setting up LCD");
  lcd.begin(16, 2);
  lcdReset();

  Serial.println("Setting up interrupts");
  setupDigitalWaveCounter(); // We set the initial operating mode to digital wave counter

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
    rmsVoltage = analogToBoardVoltage(rmsAnalog);
    lcdVoltage(rmsVoltage);
    // We wait 0.5 second before calculating the frequency
    waitMillis(500);
    break;

  case ANALOG_ZERO_CROSS:
    // We calculate the frequency using the running average
    if (zerocrosstime >= (crosstimeN - 1))
    {
      frequency = analogFrequency();
      rmsVoltage = analogToGridVoltage(rmsCalculation(counter * sampleTime));
      // We print the frequency to the serial monitor
      Serial.println(frequency);
      lcdFrequency(frequency);
      lcdVoltage(rmsVoltage);
    }
    break;
  case ANALOG_SAMPLE_PASSTHROUGH:
    lcdVoltage(OldSample);
    waitMillis(100);
    break;
  default:
    // Default case to reset for security
    setupDigitalWaveCounter();
    break;

    FreqAlert();
    digitalWrite(kalibrering, LOW); //// skal måske fjernes, ikke testet endnu
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
  double freq = (samrate * (crosstimeN - 1) / (counter));
  counter = 0;
  zerocrosstime = 0;
  return freq;
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
void rmsSum(float value, unsigned long time)
{
  value -= 511;
  analogSquareSum += value * value * time;
}

float rmsCalculation(unsigned long period)
{
  float rms = sqrt(analogSquareSum / float(period));
  analogSquareSum = 0;
  return rms;
}

//--------------------------Analog to voltage--------------------------//
float analogToBoardVoltage(float analogValue)
{
  return (analogValue / 1023) * 3.3 + voltageOffset;
}

float analogToGridVoltage(float analogValue)
{
  return 340 * ((analogValue / 1023));
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
  unsigned long currentTime = micros();
  timeArray[currentIndex] = currentTime - lastTime;              // Save current time stamp in the array
  rmsSum(analogRead(ADCPin), (currentTime - lastVoltage) * 1e6); // Last voltage measurement
  rmsAnalog = rmsCalculation((currentTime - lastTime) * 1e6);
  lastTime = currentTime;
  if (++currentIndex == avgSampleLength) // Reset when the array is full
  {
    currentIndex = 0;
  }
}

void Timer5_analogZeroCross()
{
  float newSample = lowPassFilter(analogRead(ADCPin), OldSample);
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
  unsigned long now = micros();
  rmsSum(analogRead(ADCPin), (now - lastVoltage) * 1e6);
  lastVoltage = now;
}

void Timer5_analogSamplePassthrough()
{
  float newSample = lowPassFilter(analogRead(ADCPin), OldSample);
  OldSample = newSample;
  analogWrite(DACPin, newSample);
}

//--------------------------Mode setup functions--------------------------//
void setupDigitalWaveCounter()
{
  // Setup for digital wave counter
  operatingMode = DIGITAL_WAVE_COUNTER;
  // The timeArray is reset to 0
  Serial.println("Resetting time array");
  resetTimeArray();
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
  counter = 0;
  setTimer5(analogSampleRate, Timer5_analogZeroCross);
}

void setupAnalogSamplePassthrough()
{
  // Setup for analog sample passthrough
  operatingMode = ANALOG_SAMPLE_PASSTHROUGH;
  // Detaching interrupt and resetting lcd
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  lcdReset();
  lcdFrequency(0);

  // Setting timer to analog sample rate and attaching interrupt
  setTimer5(analogSampleRate, Timer5_analogSamplePassthrough);
}

void setTimer5(float sampleRate, voidFuncPtr callback)
{
  MyTimer5.end();
  Serial.println("Timer ended");
  samrate = sampleRate;
  sampleTime = 1 / sampleRate;

  lowPassFilterSetup(sampleTime);
  Serial.println("Low pass filter setup done");

  // define frequency of interrupt
  MyTimer5.begin(sampleRate); // 200=for toggle every 5msec
  Serial.println("Timer begun");
  // define the interrupt callback function
  MyTimer5.attachInterrupt(Timer5_analogZeroCross);
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

  
// Part 10, frequency alert where the LED turns on and off in an interval
void FreqAlert(){
  if (freq <= 49.9) {

    //FCR N skal opjustere
    digitalWrite(freqAlert, LOW);

  } else if (freq >= 50.1) {

    //FCR N skal nedjusteres 
    digitalWrite(freqAlert, HIGH);

  }

}