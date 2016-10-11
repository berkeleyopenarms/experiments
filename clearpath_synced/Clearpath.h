// helper class for running the clearpath
// motors in incremental distance mode

#ifndef CLEARPATH_H
#define CLEARPATH_H

#import <Arduino.h>
#include <stdint.h>

class Clearpath
{
  private:
    const uint8_t pin_hlfb_;
    const uint8_t pin_a_;
    const uint8_t pin_b_;
    const uint8_t pin_enable_;
  
    int8_t encoder_channel_;
    float encoder_ticks_per_step_;
    int16_t soft_min_;
    int16_t soft_max_;
    bool invert_direction_;

    bool current_a_;
    bool current_b_;
    bool current_enable_;
  public:
    Clearpath(uint8_t hlfb, uint8_t b, uint8_t a, uint8_t enable);
    uint16_t absolute_setpoint;
    uint16_t absolute_position;
    bool init_encoders(uint8_t channel, uint16_t soft_min, uint16_t soft_max, bool invert);
    void update();
    void read_encoder();
};

#endif
