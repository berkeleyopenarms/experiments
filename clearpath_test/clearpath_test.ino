#include <TimerThree.h>

#define PIN_HLFB 2
#define PIN_B 3
#define PIN_A 4
#define PIN_ENABLE 5
#define PIN_LED 13

void setup() {
  pinMode(PIN_HLFB, INPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
 
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  digitalWrite(PIN_ENABLE, HIGH);

  
  for (int i=0; i < 50; i++) {
    digitalWrite(PIN_ENABLE, HIGH);
    digitalWrite(PIN_LED, HIGH);
    delay(10);
    digitalWrite(PIN_ENABLE, LOW);
    digitalWrite(PIN_LED, LOW);
    delay(10);
  }
  digitalWrite(PIN_ENABLE, HIGH);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  // blah
}
