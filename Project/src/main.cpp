#include <Arduino.h>

const unsigned int avgSampleLength = 250;          // Number og samples to average over in the running average
const unsigned int avgPrintoutsPerCycle = 5;       // Number of printouts of running average per cycle of the running average array
unsigned int avgPrintoutSampleFrequency;           // Variable to be used in the interrupt function
volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
volatile double frequency;                         // Global variable for the frequency
unsigned int currentIndex = 0;                     // We create the empty time stamp array used for the interrupt
const double cutOffFrequency = 50;                 // Cut off frequency for the low pass filter
const double pi = 3.141592653589793;
double RC; // Time constant for the low pass filter

int interruptPin = 0; // Interrupt pin for the frequency counter

void setup()
{
  avgPrintoutSampleFrequency = avgSampleLength / avgPrintoutsPerCycle; // Number of samples to skip before printing out the running average
  Serial.begin(9600);                                                  // Serial communication rate
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING); // We set our interrupt to trigger the interupt function when value reaches HIGH
  RC = 1 / (2 * pi * cutOffFrequency);                                     // We calculate the time constant for the low pass filter
}

void loop()
{
}

// Interrupt function that updates the time stamp array every time the interrupt pin goes HIGH
void timeStamp()
{
  noInterrupts();

  timeArray[currentIndex] = millis(); // Save current time stamp in the array

  /* The current index is incremented. If it is a mulriple of the avgPrintoutSampleFrequency, we calculate the frequency and print it out.
     If the current index is equal to the length of the time stamp array, we reset the index and calculate the frequency.
     currentIndex != 1 is used to avoid printing the frequency twice when the index is reset.
  */
  if (currentIndex++ % avgPrintoutSampleFrequency == 0 && currentIndex != 1)
  {
    frequency = double(1000) * double(avgSampleLength - 1) / double(timeArray[currentIndex - 1] - timeArray[currentIndex]);
  }
  else if (currentIndex == avgSampleLength) // Reset when the array is full
  {
    currentIndex = 0;
    frequency = double(1000) * double(avgSampleLength - 1) / double(timeArray[avgSampleLength - 1] - timeArray[0]);
  }
  interrupts();
}

double averageFrequency()
{
  unsigned long lowest = timeArray[0];
  unsigned long highest = timeArray[0];

  for (int i = 1; i < avgSampleLength; i++)
  {
    if (timeArray[i] < lowest)
    {
      lowest = timeArray[i];
    }
    else if (timeArray[i] > highest)
    {
      highest = timeArray[i];
    }
  }
  return (highest - lowest) / (avgSampleLength - 1);
}
