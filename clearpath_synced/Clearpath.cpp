#include "Clearpath.h"

#include <Arduino.h>
#include <Wire.h>

Clearpath::Clearpath(uint8_t hlfb, uint8_t b, uint8_t a, uint8_t enable)
  : pin_hlfb_(hlfb), pin_a_(a), pin_b_(b), pin_enable_(enable)
{
  pinMode(hlfb, INPUT_PULLUP);
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(enable, OUTPUT);

  encoder_channel_ = -1;

  invert_direction_ = false;

  current_a_ = false;
  current_b_ = false;
  current_enable_ = false;
  digitalWrite(a, current_a_);
  digitalWrite(b, current_b_);
  digitalWrite(enable, !current_enable_);
}

bool Clearpath::init_encoders(uint8_t channel, uint16_t soft_min, uint16_t soft_max, bool invert) {
  soft_min_ = soft_min;
  soft_max_ = soft_max;
  encoder_channel_ = channel;
  invert_direction_ = invert;

  read_encoder();

  if(abs((signed)soft_min - (signed)absolute_position) > abs((signed)soft_max - (signed)absolute_position)) {
    // we're closer to the max than the min, so we should calibrate in the negative direction (move towards min)
    current_a_ = true;
  } else {
    current_a_ = false;
  }
    // TODO: refactor direction setting -- "current_a" is too confusing
  digitalWrite(pin_a_, current_a_ ^ invert_direction_);
  delay(10);

  Serial.println("backlash stuff..");
  for (int i = 0; i < 3; i++) {
    // move a bit to shave off any backlash/slop
    digitalWrite(pin_enable_, LOW);
    delay(10);
    digitalWrite(pin_enable_, HIGH);
    delay(10);
  }
  Serial.println("start stuff..");
  int32_t start;
  do {
    //wait for value to stabilize
    start = absolute_position;
    delay(200);
    read_encoder();
  } while(start != absolute_position);
  Serial.println("stop stuff..");
  for (int i = 0; i < 40; i++) {
    // move 20 steps
    digitalWrite(pin_enable_, LOW);
    delay(10);
    digitalWrite(pin_enable_, HIGH);
    delay(10);
  }
  int32_t stop;
  do {
    //wait for value to stabilize
    stop = absolute_position;
    delay(200);
    read_encoder();
  } while(stop != absolute_position);
  encoder_ticks_per_step_ = abs(stop - start) / 20.0;
  Serial.println(encoder_ticks_per_step_);

  absolute_setpoint = absolute_position;
  Serial.println(absolute_setpoint);
  return true;
}

void Clearpath::read_encoder() {
  if (encoder_channel_ == -1) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << encoder_channel_);
  Wire.endTransmission();

  int16_t reading;
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0E));             // sets register pointer
  Wire.endTransmission();    // stop transmitting
  //assume that we can always receiving the correct bytes
  Wire.requestFrom(0x36, 2);         // request 2 bytes from as5601
  reading = Wire.read();             // receive high byte
  reading = reading << 8;            // shift high byte to be high 8 bits
  reading |= Wire.read();            // receive low byte as lower 8 bits

  absolute_position = reading;
}

void Clearpath::update() {
  absolute_position = min(max(soft_min_, absolute_position), soft_max_);

  //hlfb = digitalRead(_pin_hlfb);
  int16_t distance = (signed)absolute_setpoint - (signed)absolute_position;
  int16_t step_offset = abs(distance) / encoder_ticks_per_step_;
  if (absolute_setpoint == -1 || step_offset == 0 || absolute_position < 0 || absolute_position >= 4048) {
    //Serial.println('.');
    current_enable_ = 1;
    digitalWrite(pin_enable_, HIGH);
    return;
  }

  if ((current_a_ && distance > 0)
      || (!current_a_ && distance < 0)) {
    current_a_ = !current_a_;
    digitalWrite(pin_a_, current_a_ ^ invert_direction_);

    current_enable_ = 1;
    digitalWrite(pin_enable_, current_enable_);

    // don't increment immediately if we're changing direction
    return;
  }

  current_enable_ = !current_enable_;
  digitalWrite(pin_enable_, current_enable_);
}
