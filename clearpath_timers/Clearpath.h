// helper class for running the clearpath
// motors in incremental distance mode

#ifndef CLEARPATH_H
#define CLEARPATH_H

#import <Arduino.h>
#include <stdint.h>

class Clearpath
{
  private:
    const uint8_t _pin_a;
    const uint8_t _pin_b;
    const uint8_t _pin_enable;
    bool _current_a;
    bool _current_b;
    bool _current_enable;
    int64_t _position;
    static int8_t _signum(int64_t n);
  public:
    Clearpath(uint8_t a, uint8_t b, uint8_t enable);
    int64_t setpoint;
    void update();
    void test(bool x);
};

#endif
