#define BANDGAPREF 14   // special indicator that we want to measure the bandgap

const int sensorPin = A1;
const int ledPin1 = 5;
const int ledPin2 = 6;
const int ledPin3 = A2;
const int ledPin4 = A4;
const int ledPin5 = A3;

float degreesC;

void setup() {
  // enable the internal ~1.1volt bandgap reference
  analogReference(INTERNAL);
  
  // Set the temperature sensor pin as an INPUT:
  pinMode(sensorPin, INPUT);
  
  Serial.begin(9600);  //Start the serial connection with the computer
                       //to view the result open the serial monitor 
  delay(500);
}

void loop() {
  readTemperature();
  ledScale();
}

void readTemperature() {
  float supplyVoltage = readVcc() / 1024;
  Serial.print("Supply Voltage: "); Serial.println(supplyVoltage);
 
  // Read the raw 0-1023 value of temperature into a variable.
  float sensorValue = analogRead(sensorPin);
  Serial.print("Sensor value: "); Serial.println(sensorValue);

  // Calculate the voltage, based on that value.
  float sensorVoltage = sensorValue * (supplyVoltage / 1024.0);
  Serial.print("Sensor Voltage: "); Serial.println(sensorVoltage);

  // Calculate the celsius temperature, based on that voltage..
  degreesC = (sensorVoltage - 0.5) * 100.0;
  Serial.print("Temp (C): "); Serial.println(degreesC);

  Serial.println();
  // Wait 1 second between readings
  delay(1000);  
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

void ledScale() {
  if (degreesC <= 18) {
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    digitalWrite(ledPin5, LOW);
  }
  if (degreesC > 19) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    digitalWrite(ledPin5, LOW);
  }
  if (degreesC > 20) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    digitalWrite(ledPin5, LOW);
  }
  if (degreesC > 21) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, LOW);
    digitalWrite(ledPin5, LOW);
  }
  if (degreesC > 22) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, HIGH);
    digitalWrite(ledPin5, LOW);
  }
  if (degreesC > 24) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, HIGH);
    digitalWrite(ledPin5, HIGH);
  }
}
