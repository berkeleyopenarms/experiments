#include "Clearpath.h"
#include "Arduino.h"

Clearpath::Clearpath(uint8_t a, uint8_t b, uint8_t enable)
  : _pin_a(a), _pin_b(b), _pin_enable(enable)
{

  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(enable, OUTPUT);

  _position = 0;
  setpoint = 0;

  _current_a = 0;
  _current_b = 0;
  _current_enable = 0;
  digitalWrite(a, _current_a);
  digitalWrite(b, _current_b);
  digitalWrite(enable, !_current_enable);
}


void Clearpath::update() {

  //offset = setpoint - position;
  if (setpoint == _position) {
    _current_enable = 1;
    digitalWrite(_pin_enable, HIGH);
    return;
  }
  int64_t error = setpoint - _position;

  // A should be pulled low to move in the position direction
  // or high to move in the negative
  if ((_current_a && error > 0)
      || (!_current_a && error < 0)) {
    _current_a = !_current_a;
    digitalWrite(_pin_a, _current_a);

    _current_enable = 1;
    digitalWrite(_pin_enable, _current_enable);

    // don't increment immediately if we're changing direction
    return;
  }

  // shift our position one step towards 0, but only if we're
  // about to pull the enable line down low
  _position += _signum(error) * _current_enable;

  _current_enable = !_current_enable;
  digitalWrite(_pin_enable, _current_enable);
}

int8_t Clearpath::_signum(int64_t n) {
  if (n < 0) return -1;
  if (n > 0) return 1;
  return 0;
}
