#include <TimerThree.h>
#include <stdint.h>
#include <Wire.h>

#include "Clearpath.h"

Clearpath* motors[] = {
  //new Clearpath(17, 16, 18),  
  new Clearpath(4, 3, 5)
};

void setup() {
  Serial.begin(250000);
  Serial.println("Begin");

  Wire.begin();

  Serial.println("Begin");
  Serial.println("Begin");
  Serial.println("Begin");
  motors[0]->init_encoders(0, 20, 1100, true);

  Timer3.initialize(10000);
  //Timer3.initialize(1000000);
  Timer3.attachInterrupt(pulse);
}

uint8_t index = 0;
void loop() {
  /*int32_t val = Serial.parseInt();
  if (val != 0 && val != 9252) { // these values keep popping up
    Serial.print(index);
    Serial.print("\t");
    Serial.println(val);

    motors[index++]->absolute_setpoint = val;
    index %= sizeof(motors) / sizeof(motors[0]);
  }*/
  motors[0]->_read_encoder();
  Serial.print("set:");
  Serial.print(motors[0]->absolute_setpoint);
  Serial.print("\t\tlook:");
  Serial.print(motors[0]->_lookahead_position);
  Serial.print("\tpos:");
  Serial.print(motors[0]->_absolute_position);
  Serial.print("\tsoff:");
  Serial.print(motors[0]->step_offset);
  Serial.print("\tdist:");
  Serial.println(motors[0]->distance);
}
void pulse() {
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(motors[0]) ; i++) {
    motors[i]->update();
  }
}
