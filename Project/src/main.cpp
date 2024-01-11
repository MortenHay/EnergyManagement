#include <Arduino.h>
#include <algorithm>
#include <Timer5.h>
#include <api/Print.h>
#include <LiquidCrystal.h>
using namespace std;



// Tester om lcd kan bruges
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
const int switchPin = 6;
int switchState = 0;
int reply;

const unsigned int avgSampleLength = 3;            // Number og samples to average over in the running average

volatile unsigned long timeArray[avgSampleLength]; // We create the empty time stamp array used for the interrupt
double frequency;                                  // Global variable for the frequency
unsigned int currentIndex = 0;                     // We create the empty time stamp array used for the interrupt
const unsigned long ulongMax = 4294967295;         // Maximum value of unsigned long
const unsigned long ulongThreashold = 4000000000;  // Threshold for when the time stamp array should be reset
unsigned long lastTime = 0;


const double cutOffFrequency = 100;  // Cut off frequency for the low pass filter
const float pi = 3.141592653589793; // constant pi
double alpha;                       // Constant for the low pass filter

const int interruptPin = 0; // Interrupt pin for the frequency counter
const int DACPin = A0;      // DAC pin for output
const int ADCPin = A1;      // ADC pin for input
volatile double val = 0;                //Creating val for analog read
const float Samrate = 10000; //Sampling rate for MyTimer5
volatile double Amplitude = 511; //Creating val for analog read
volatile double crosstimeN = 50; // Creating a val for the number of zero crossings bore calculation
volatile double freq = 0; //Creating val for frequency
volatile float newSample = 0; //Creating val for low pass filter 
volatile float OldSample = 0; //Creating val for low pass filter
const float sampleTime = 1/Samrate;   // Sample time for the low pass filter in seconds
volatile double zerocrosstime = 0;
volatile int counter = 0;

const int freqAlert = 7; //Creating val for frequency alert


volatile double Analogarray[avgSampleLength];  //Creating array for analog input

const unsigned long ulongThreasholdTest = 0; // Temp test value

// Function declarations
void timeStamp();
double averageFrequency();
void resetTimeArray();
void AdcBooster();
void waitMillis(unsigned long ms);
void waitMicros(unsigned long us);
void Timer5_IRQ();
void FreqAlert();


//Initializing 
void setup()
{
  analogWriteResolution(10); // We set the resolution of the DAC to 10 bit
  analogReadResolution(10);  // We set the resolution of the ADC to 10 bit  


  Serial.begin(9600); // Serial communication rate
  for (unsigned int i = 0; i < avgSampleLength; i++)
  {
    // We initialize the time stamp array to 0
    timeArray[i] = 0;
  }
  analogWriteResolution(10); // We set the resolution of the DAC to 10 bits
  analogReadResolution(10);  // We set the resolution of the ADC to 10 bits
  // pins
  pinMode(interruptPin, INPUT);
  pinMode(DACPin, OUTPUT);
  pinMode(ADCPin, INPUT);
  pinMode(freqAlert, OUTPUT);

  //attachInterrupt(digitalPinToInterrupt(interruptPin), timeStamp, RISING); // We set our interrupt to trigger the interupt function when value reaches HIGH
  double RC = 1 / (2 * pi * cutOffFrequency);                              // We calculate the time constant for the low pass filter
  alpha = sampleTime / (sampleTime + RC);                                  // We calculate the constant for the low pass filter
  AdcBooster();                                                             // We boost the ADC clock frequency to 2 MHz
  Serial.println("Setup done!");
  
   // define frequency of interrupt
	MyTimer5.begin(Samrate);  // 200=for toggle every 5msec

    // define the interrupt callback function
    MyTimer5.attachInterrupt(Timer5_IRQ);
  
    // start the timer
    MyTimer5.start();




  //lcd kode
  lcd.begin(16,2);
  pinMode(switchPin,INPUT);
  lcd.print("Klar til");
  lcd.setCursor(0,1);
  lcd.print("maaling :)");
}


//Main loop
void loop()
{
  //waitMillis(500); // We wait 0.5 second before calculating the frequency
  // print and calculation of frequency

 // If function for analog calculating frequency
 // 1.0989010989 factor since MyTimer5 runs 91 times at a 100 times rate
 // 0.9739943508327652 factor for 1000Hz
 // 1.086840767 factor for 10000Hz
  if (zerocrosstime >= (crosstimeN)) {

    freq = (Samrate*(crosstimeN)/(counter));

    counter = 0;
    zerocrosstime = 0;

    Serial.println(freq);
    switchState = digitalRead(switchPin);
    if(switchState){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Value: ");
    lcd.setCursor(0,1);
    lcd.print(freq,5);
    }
    //Calls the function where the LED turns on and off in an interval
    FreqAlert();
  }
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
}

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


void Timer5_IRQ() {
  
  newSample = double(analogRead(ADCPin))*alpha + OldSample*(1-alpha); // Data through low pass filter

    if(newSample >= Amplitude/2 && OldSample < Amplitude/2) { // Detects zero crossing

      counter++;
      zerocrosstime++; // Counts zero crossings

    } else {

      counter++; // Counts number of samples (that aren't zero crossings)

    }

   OldSample = newSample; // Updates old sample

  }




void AnalogArray(){
for (unsigned int i = 0; i < avgSampleLength; i++){
Analogarray[i]=val;
Serial.println(Analogarray[i]);
}
}

// Part 10, frequency alert where the LED turns on and off in an interval
void FreqAlert(){
  if (freq <= 49.999) {

    digitalWrite(freqAlert, LOW);

  } else if (freq >= 50.00) {

    digitalWrite(freqAlert, HIGH);

  } else {
  }
}