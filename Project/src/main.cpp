#include <Arduino.h>
#include <algorithm>
#include <Timer5.h>
#include <api/Print.h>
using namespace std;

const unsigned int avgSampleLength = 50;           // Number og samples to average over in the running average
volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
double frequency;                                  // Global variable for the frequency
unsigned int currentIndex = 0;                     // We create the empty time stamp array used for the interrupt
const unsigned long ulongMax = 4294967295;         // Maximum value of unsigned long
const unsigned long ulongThreashold = 4000000000;  // Threshold for when the time stamp array should be reset
unsigned long now;

const int sampleTime = 5;
const double cutOffFrequency = 50;  // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
double alpha;                       // Constant for the low pass filter

const int interruptPin = 0; // Interrupt pin for the frequency counter
const int DACPin = A0;      // DAC pin for output
const int ADCPin = A1;      // ADC pin for input
volatile int val = 0;                //Creating val for analog read

volatile unsigned long Analogarray[avgSampleLength];  //Creating array for analog input

const unsigned long ulongThreasholdTest = 0; // Temp test value

// Function declarations
void timeStamp();
double averageFrequency();
void resetTimeArray();
void AdcBooster();
void waitMillis(unsigned long ms);
void waitMicros(unsigned long us);
void Timer5_IRQ();

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
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING); // We set our interrupt to trigger the interupt function when value reaches HIGH
  double RC = 1 / (2 * pi * cutOffFrequency);                              // We calculate the time constant for the low pass filter
  alpha = sampleTime / (sampleTime + RC);                                  // We calculate the constant for the low pass filter
  // AdcBooster();                                                             // We boost the ADC clock frequency to 2 MHz
  Serial.println("Setup done!");
  
   // define frequency of interrupt
	MyTimer5.begin(200);  // 200=for toggle every 5msec

    // define the interrupt callback function
    MyTimer5.attachInterrupt(Timer5_IRQ);
  
    // start the timer
    MyTimer5.start();
}

void loop()
{
  waitMillis(500); // We wait 0.005 second before calculating the frequency
  frequency = val; // We calculate the running average of the frequency
  // Print out the running average of the frequency
  Serial.print("Value: ");
  Serial.print(frequency);
  Serial.println(" Position");
}

// Interrupt function that updates the time stamp array every time the interrupt pin goes HIGH
void timeStamp()
{
  noInterrupts();
  Serial.println(micros());
  timeArray[currentIndex] = micros();    // Save current time stamp in the array
  if (++currentIndex == avgSampleLength) // Reset when the array is full
  {
    currentIndex = 0;
  }
  interrupts();
}
// Function that calculates the running average of the frequency
double averageFrequency()
{
  // Finds highest and lowest times in timeArray to calculate time to reach avgSampleLength samples

  // Start by making snapshot of timeArray
  unsigned long timeArraySnapshot[avgSampleLength];
  copy(timeArray, timeArray + avgSampleLength, timeArraySnapshot);
  // Find highest and lowest values in snapshot
  unsigned int i = 0;
  while (i < avgSampleLength && timeArraySnapshot[i] < timeArraySnapshot[i + 1])
  {
    /* The highest value will always preceed the lowest value in the array
 This loop finds the index of the highest value*/

    if (timeArraySnapshot[i++] >= ulongThreashold)
    {
      Serial.println("Overflow");
      // Check if micros() is in danger of overflowing
      for (unsigned int j = 0; j < avgSampleLength; j++)
      {
        if (timeArraySnapshot[j] >= ulongThreashold * 0.9)
        {
          // Subtract all values in the array from ulongMax if they are close to overflowing
          timeArraySnapshot[j] = ulongMax - timeArraySnapshot[j];
        }
      }
      // Reset the loop if array was changed
      i = 0;
    }
  }

  unsigned long highest = timeArraySnapshot[i++]; // We save the highest value and increment i to the next value

  if (i == avgSampleLength)
  {
    // If the highest value is the last value in the array, we reset i to 0
    i = 0;
  }
  unsigned long lowest = timeArraySnapshot[i]; // We save the lowest value

  // We calculate the running average of the frequency
  return double(1000000) * (avgSampleLength - 1) / (highest - lowest);
}

void resetTimeArray()
{
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  // We reset the time stamp array
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    timeArray[i] = 0;
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING);
}

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


void Timer5_IRQ() {
    val = analogRead(ADCPin);
    
}



void AnalogArray()
{

for (unsigned int i = 0; i < avgSampleLength; i++){
 
 Analogarray[i]=val;
}



}


