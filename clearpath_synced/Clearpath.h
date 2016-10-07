// helper class for running the clearpath
// motors in incremental distance mode

#ifndef CLEARPATH_H
#define CLEARPATH_H

#import <Arduino.h>
#include <stdint.h>

class Clearpath
{
  public:
    const uint8_t _pin_a;
    const uint8_t _pin_b;
    const uint8_t _pin_enable;

    int16_t distance;
    uint16_t step_offset;

    uint8_t _counter;
  
    int8_t _encoder_channel;
    float _encoder_ticks_per_step;
    int16_t _soft_min;
    int16_t _soft_max;
    bool _invert_direction;
    uint16_t _absolute_position;
    float _lookahead_position;

    uint8_t _pulse_queue_length;

    bool _current_a;
    bool _current_b;
    bool _current_enable;
    int64_t _position;
    static int8_t _signum(int64_t n);
  //public:
    Clearpath(uint8_t a, uint8_t b, uint8_t enable);
    int64_t setpoint;
    uint16_t absolute_setpoint;
    bool init_encoders(uint8_t channel, uint16_t soft_min, uint16_t soft_max, bool invert);
    void update();
    void test(bool x);
    void _read_encoder();
};

#endif
