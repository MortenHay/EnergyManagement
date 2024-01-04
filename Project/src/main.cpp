#include <Arduino.h>
#include <algorithm>
using namespace std;

const unsigned int avgSampleLength = 250;          // Number og samples to average over in the running average
volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
volatile double frequency;                         // Global variable for the frequency
unsigned int currentIndex = 0;                     // We create the empty time stamp array used for the interrupt
const double cutOffFrequency = 50;                 // Cut off frequency for the low pass filter
const double pi = 3.141592653589793;               // constant pi
double RC;                                         // Time constant for the low pass filter

int interruptPin = 0; // Interrupt pin for the frequency counter

// Function declarations
void timeStamp();
double averageFrequency();

void setup()
{
  Serial.begin(9600); // Serial communication rate
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    // We initialize the time stamp array to 0
    timeArray[i] = 0;
  }
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING); // We set our interrupt to trigger the interupt function when value reaches HIGH
  // RC = 1 / (2 * pi * cutOffFrequency);                                     // We calculate the time constant for the low pass filter
}

void loop()
{
  frequency = averageFrequency(); // We calculate the running average of the frequency
  // Print out the running average of the frequency
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");
  delay(1000); // We wait 1 second between printing out the running average
}

// Interrupt function that updates the time stamp array every time the interrupt pin goes HIGH
void timeStamp()
{
  noInterrupts();
  timeArray[currentIndex] = millis(); // Save current time stamp in the array

  if (currentIndex == avgSampleLength) // Reset when the array is full
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
  while (i < avgSampleLength && timeArray[i] < timeArray[i + 1])
  {
    /* The highest value will always preceed the lowest value in the array
     This loop finds the index of the highest value*/
    i++;
  }

  unsigned long highest = timeArray[i++]; // We save the highest value and increment i to the next value

  if (i == avgSampleLength)
  {
    // If the highest value is the last value in the array, we reset i to 0
    i = 0;
  }
  unsigned long lowest = timeArray[i]; // We save the lowest value

  // We calculate the running average of the frequency
  return double(1000) * (avgSampleLength - 1) / (highest - lowest);
}
