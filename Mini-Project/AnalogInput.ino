/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

//int sensorPin = A0;    // select the input pin for the potentiometer
//int outPin = 4;      // select the pin for the LED
//int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the pins as an OUTPUT and make them high:
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  pinMode(7, OUTPUT);
  
  pinMode(8, OUTPUT);
  
  pinMode(9, OUTPUT);
  
  pinMode(10, OUTPUT);
  //pinMode(10, HIGH);

  pinMode(12, INPUT);

 // pinMode(12, HIGH);
  
}

void loop() {
  // read the value from the sensor:
 // sensorValue = analogRead(sensorPin);

 //write the value to the pin
 analogWrite(9,127);
// analogWrite(10,127);


}
