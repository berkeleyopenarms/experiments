#include <TimerThree.h>
#include <stdint.h>

#include "Clearpath.h"

Clearpath* motors[] = {
  new Clearpath(17, 16, 18),
  new Clearpath(4, 3, 5)
};

void setup() {
  Serial.begin(19200);
  Serial.println("Begin");

  Timer3.initialize(10000);
  Timer3.attachInterrupt(pulse);
}

uint8_t index = 0;
void loop() {
  int32_t val = Serial.parseInt();
  if (val != 0 && val != 9252) { // these values keep popping up
    Serial.print(index);
    Serial.print("\t");
    Serial.println(val);

    motors[index++]->setpoint = val;
    index %= sizeof(motors) / sizeof(motors[0]);
  }
}
void pulse() {
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(motors[0]) ; i++) {
    motors[i]->update();
  }
}
