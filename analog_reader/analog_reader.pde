/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13. 
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead(). 
 
 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground
 
 * Note: because most Arduinos have a built-in LED attached 
 to pin 13 on the board, the LED is optional.
 
 
 Created by David Cuartielles
 Modified 4 Sep 2010
 By Tom Igoe
 
 This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/AnalogInput
 
 */
int analogValues[6];

void setup() {
  Serial.begin(9600);
  // declare the ledPin as an OUTPUT:
//  pinMode(ledPin, OUTPUT);  
}

void loop() {
  
  for(int i = 0; i<6; i++){
    // read the value from the sensor:
    analogValues[i] = analogRead(i);
    //print out value
    Serial.print(analogValues[i]);
    //tab delimiter
    Serial.print("\t");
  }
  
  Serial.println();

}
