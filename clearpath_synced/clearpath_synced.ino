#include <TimerOne.h>
#include <stdint.h>
#include <Wire.h>

#include "Clearpath.h"


/*
 * elbow channel 7: 2365-3805 // 2500
 * roll channel 6: 2330-10 // 1780
 * lift channel 5: 150-2000 // 979
 * swing channel 4: 1580 - 2350 // 2220
 */

Clearpath* motors[] = {
  //new Clearpath(17, 16, 18),  
  new Clearpath(22, 24, 26, 28), // roll
  new Clearpath(30, 32, 34, 36), // swing
  new Clearpath(38, 40, 42, 44), // lift
  new Clearpath(46, 48, 50, 52) // lift
};

void setup() {
  Serial.begin(115200);
  Serial.println("Begin");

  Wire.begin();

  Serial.println("Begin");
  Serial.println("Begin");
  Serial.println("Begin");

  //roll
  motors[0]->init_encoders(6, 10, 2330, false);
  motors[0]->absolute_setpoint = 1800;

  //swing
  motors[1]->init_encoders(4, 1580, 2350, true);
  motors[1]->absolute_setpoint = 2200;

  //lift
  motors[2]->init_encoders(5, 150, 2000, true);
  motors[2]->absolute_setpoint = 979;

  //lift
  motors[3]->init_encoders(7, 2365, 3805, true);
  motors[3]->absolute_setpoint = 2500;
  
  Timer1.initialize(10000);

  
  //Timer3.initialize(1000000);
  Timer1.attachInterrupt(pulse);
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
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(motors[0]) ; i++) {
    motors[i]->_read_encoder();
  }
  Serial.print("set:");
  Serial.print(motors[0]->absolute_setpoint);
  Serial.print("\t\tlook:");
  Serial.print(motors[0]->_lookahead_position);
  Serial.print("\tpos:");
  Serial.print(motors[0]->_absolute_position);
  Serial.print("\tsoff:");
  Serial.print(motors[0]->step_offset);
  Serial.print("\tdist:");
  Serial.print(motors[0]->distance);
  Serial.print("\thlfb:");
  Serial.println(motors[0]->hlfb);
}
void pulse() {
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(motors[0]) ; i++) {
    motors[i]->update();
  }
}
