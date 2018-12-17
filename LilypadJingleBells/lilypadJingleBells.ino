#include "pitches.h"

#define BANDGAPREF 14   // special indicator that we want to measure the bandgap

const int sensorPin = A1;

const int redLed = 9;        
const int blueLed = 10;
const int greenLed = 11;

const int cLedPin = 5;
const int dLedPin = 6;
const int eLedPin = A2;
const int fLedPin = A4;
const int gLedPin = A3;

const int buttonPin = A5;     // the number of the pushbutton pin

int ledPin = 0;
int colouredLed = 0;

// notes in the melody:
int melody[] = {
  NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_G3, NOTE_C3, NOTE_D3, NOTE_E3, NOTE_F3, NOTE_F3, NOTE_F3, NOTE_F3, NOTE_F3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3, NOTE_D3, NOTE_D3, NOTE_E3, NOTE_D3, NOTE_G3
  //NOTE_G3, NOTE_G3, NOTE_G3, NOTE_G3, NOTE_G3, NOTE_G3, NOTE_G3, NOTE_C3, NOTE_G3,
  //NOTE_C3, NOTE_G3,
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
 4, 4, 2, 4, 4, 2, 4, 4, 3, 8, 1, 4, 4, 3, 8, 4, 4, 4, 8, 8, 4, 4, 4, 4, 2, 2
};

void setup() {
  // enable the internal ~1.1volt bandgap reference
  analogReference(INTERNAL);
  
  // Set the temperature sensor pin as an INPUT:
  pinMode(sensorPin, INPUT);

  // declare all led pins to be an output:
  pinMode(redLed, OUTPUT);
  analogWrite(redLed, 255);
  pinMode(blueLed, OUTPUT);
  analogWrite(blueLed, 255);
  pinMode(greenLed, OUTPUT);
  analogWrite(greenLed, 255);
  
  Serial.begin(9600);  //Start the serial connection with the computer
                       //to view the result open the serial monitor 
  delay(500);
}

void loop() {
  readButtonAndTemperature();
}

float readTemperature() {
  float supplyVoltage = readVcc() / 1024;
  Serial.print("Supply Voltage: "); Serial.println(supplyVoltage);
 
  // Read the raw 0-1023 value of temperature into a variable.
  float sensorValue = analogRead(sensorPin);
  Serial.print("Sensor value: "); Serial.println(sensorValue);

  // Calculate the voltage, based on that value.
  float sensorVoltage = sensorValue * (5 / 1024.0);
  Serial.print("Sensor Voltage: "); Serial.println(sensorVoltage);

  // Calculate the celsius temperature, based on that voltage..
  float tempCelcius = (sensorVoltage - 0.5) * 100.0;
  Serial.print("Temp (C): "); Serial.println(tempCelcius);

  Serial.println();
  return tempCelcius;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void readButtonAndTemperature() {
  // read the state of the pushbutton value:
  int buttonState = digitalRead(buttonPin);

  float tempCelcius = readTemperature();

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH && tempCelcius > 24) {
    playJingleBells();
  }
}

void playJingleBells() {
  
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 26; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];

    switch (melody[thisNote])
    {
      case NOTE_C3:
        ledPin = cLedPin;
        break;
      case NOTE_D3:
        ledPin = dLedPin;
        break;
      case NOTE_E3:
        ledPin = eLedPin;
        break;
      case NOTE_F3:
        ledPin = fLedPin;
        break;
      case NOTE_G3:
        ledPin = gLedPin;
        break;
    }
    
    if (thisNote % 2 == 0)
      colouredLed = redLed;
    else
      colouredLed = greenLed;

    tone(7, melody[thisNote], noteDuration);
    
    turnLEDsOn();

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(noteDuration);
    
    turnLEDsOff();
    
    delay(pauseBetweenNotes - noteDuration);
    
    // stop the tone playing:
    noTone(7);
  }
}

void turnLEDsOn() {
  digitalWrite(ledPin, HIGH);
  analogWrite(colouredLed, 0);
}

void turnLEDsOff() {
  digitalWrite(ledPin, LOW);
  analogWrite(colouredLed, 255);
}
