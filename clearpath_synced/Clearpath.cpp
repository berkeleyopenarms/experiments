#include "Clearpath.h"

#include <Arduino.h>
#include <Wire.h>

Clearpath::Clearpath(uint8_t hlfb, uint8_t b, uint8_t a, uint8_t enable)
  : _pin_a(a), _pin_b(b), _pin_enable(enable)
{
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(enable, OUTPUT);

  _position = 0;
  setpoint = 0;
  _encoder_channel = -1;
  _pulse_queue_length = 2;

  _invert_direction = false;

  _current_a = false;
  _current_b = false;
  _current_enable = false;
  digitalWrite(a, _current_a);
  digitalWrite(b, _current_b);
  digitalWrite(enable, !_current_enable);

  _counter = 0;
}

bool Clearpath::init_encoders(uint8_t channel, uint16_t soft_min, uint16_t soft_max, bool invert) {
  _soft_min = soft_min;
  _soft_max = soft_max;
  _encoder_channel = channel;
  _invert_direction = invert;

  _read_encoder();
  
  if(abs((signed)soft_min - (signed)_absolute_position) > abs((signed)soft_max - (signed)_absolute_position)) {
    // we're closer to the max than the min, so we should calibrate in the negative direction (move towards min)
    _current_a = true;
  } else {
    _current_a = false;  
  }
    // TODO: refactor direction setting -- "current_a" is too confusing
  digitalWrite(_pin_a, _current_a ^ _invert_direction);
  delay(10);
  
  Serial.println("backlash stuff..");
  for (int i = 0; i < 3; i++) {
    // move a bit to shave off any backlash/slop
    digitalWrite(_pin_enable, LOW);
    delay(10);
    digitalWrite(_pin_enable, HIGH);
    delay(10);
  }
  Serial.println("start stuff..");
  int32_t start;
  do {
    //wait for value to stabilize
    start = _absolute_position;
    delay(200);
    _read_encoder();
  } while(start != _absolute_position);
  Serial.println("stop stuff..");
  for (int i = 0; i < 40; i++) {
    // move 20 steps
    digitalWrite(_pin_enable, LOW);
    delay(10);
    digitalWrite(_pin_enable, HIGH);
    delay(10);
  }
  int32_t stop;
  do {
    //wait for value to stabilize
    stop = _absolute_position;
    delay(200);
    _read_encoder();
  } while(stop != _absolute_position);
  _encoder_ticks_per_step = abs(stop - start) / 20.0;
  Serial.println(_encoder_ticks_per_step);

  absolute_setpoint = _absolute_position;
  _lookahead_position = _absolute_position;
  Serial.println(absolute_setpoint);
  return true;
}

void Clearpath::_read_encoder() {
  // TODO: channel selection

  int16_t reading;
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0E));             // sets register pointer
  Wire.endTransmission();    // stop transmitting
  //assume that we can always receiving the correct bytes
  Wire.requestFrom(0x36, 2);         // request 2 bytes from as5601
  reading = Wire.read();             // receive high byte
  reading = reading << 8;            // shift high byte to be high 8 bits
  reading |= Wire.read();            // receive low byte as lower 8 bits

  _absolute_position = reading;
}

void Clearpath::update() {
  distance = (signed)absolute_setpoint - (signed)_lookahead_position;
  step_offset = abs(distance) / _encoder_ticks_per_step;
  if (step_offset == 0) {
    //Serial.println('.');
    _lookahead_position = _absolute_position;
    _current_enable = 1;
    digitalWrite(_pin_enable, HIGH);
    return;    
  }

  if ((_current_a && distance > 0)
      || (!_current_a && distance < 0)) {
    _current_a = !_current_a;
    digitalWrite(_pin_a, _current_a ^ _invert_direction);

    _current_enable = 1;
    digitalWrite(_pin_enable, _current_enable);

    // don't increment immediately if we're changing direction
    return;
  }

  uint8_t queue_length = abs((signed)_lookahead_position - (signed)_absolute_position) / _encoder_ticks_per_step;
  if (_current_enable &&
    queue_length >= _pulse_queue_length) {
    return;
  }

  _lookahead_position += _signum(distance) * _current_enable * _encoder_ticks_per_step;
  _current_enable = !_current_enable;
  digitalWrite(_pin_enable, _current_enable);
  
  
  /*
  //offset = setpoint - position;
  if (setpoint == _position) {
    _current_enable = 1;
    digitalWrite(_pin_enable, HIGH);
    return;
  }
  int64_t error = setpoint - _position;

  // TODO: refactor/abstract direction setting
  // A should be pulled low to move in the position direction
  // or high to move in the negative
  // invert these values if _invert_direction = true
  if ((_current_a && error > 0)
      || (!_current_a && error < 0)) {
    _current_a = !_current_a;
    digitalWrite(_pin_a, _current_a ^ _invert_direction);

    _current_enable = 1;
    digitalWrite(_pin_enable, _current_enable);

    // don't increment immediately if we're changing direction
    return;
  }

  // shift our position one step towards 0, but only if we're
  // about to pull the enable line down low
  _position += _signum(error) * _current_enable;

  _current_enable = !_current_enable;
  digitalWrite(_pin_enable, _current_enable);*/
}

int8_t Clearpath::_signum(int64_t n) {
  if (n < 0) return -1;
  if (n > 0) return 1;
  return 0;
}
