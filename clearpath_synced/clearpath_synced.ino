#include <ros.h>
#include <std_msgs/Int16MultiArray.h> // the input data, an int array with 11 entries [Swing_Gravity(mN), Lifting_gravity(mN), Roll_Gravity(mN), Elbow_gravity, .. the corresponding position]
#include <std_msgs/Int16.h>

#include <TimerOne.h>
#include <stdint.h>
#include <Wire.h>

#include "Clearpath.h"


/*
 * swing channel 4: 1580 - 2350 // 2220
 * lift channel 5: 150-2000 // 979
 * roll channel 6: 2330-10 // 1780
 * elbow channel 7: 2365-3805 // 2500
 */

#define SWING 0
#define LIFT 1
#define ROLL 2
#define ELBOW 3

Clearpath* motors[] = {
  //new Clearpath(17, 16, 18),  
  new Clearpath(30, 32, 34, 36), // swing
  new Clearpath(38, 40, 42, 44), // lift
  new Clearpath(22, 24, 26, 28), // roll
  new Clearpath(46, 48, 50, 52) // elbow
};

uint32_t motor_zeros  [4] = {
  3456,
  1506,
  1276,
  3940
};


int8_t motor_dir  [4] = {
  1,
  1,
  -1,
  -1
};

ros::NodeHandle nh;

std_msgs::Int16 int_msg;
ros::Publisher debug("debug", &int_msg);

void motor_control(const std_msgs::Int16MultiArray& Position) {
    //swing, lift, roll, elbow
    //position.data[0] * 4095 / 1000 / 2 / 3.14159265);

    uint8_t i = ELBOW;
    //for (int i = 0; i  < sizeof(motors) / sizeof(motors[0]); i++) {
      motors[i]->absolute_setpoint = ((motor_zeros[i] + Position.data[i] * motor_dir[i]) % 6283) * 4095 / 6283;
    //}
    i = ROLL;
      motors[i]->absolute_setpoint = ((motor_zeros[i] + Position.data[i] * motor_dir[i]) % 6283) * 4095 / 6283;
    i = LIFT;
    
      motors[i]->absolute_setpoint = ((motor_zeros[i] + Position.data[i] * motor_dir[i]) % 6283) * 4095 / 6283;
    
   int_msg.data = 10;
   debug.publish(&int_msg);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub( "coordinate_input", &motor_control);

// ros::Publisher Joint_Position("joint_Position", &int_array);
// ros::Publisher Sys_info("sys_info", &str_msg);
// ros::Publisher debug("debug", &int_msg);

void setup() {
  nh.initNode();
  nh.advertise(debug);
  nh.subscribe(sub);
  //Serial.begin(250000);
  //Serial.println("Begin");
 
  Wire.begin();
  

  //swing
  //motors[SWING]->init_encoders(4, 1580, 2350, true);
  //motors[SWING]->absolute_setpoint = 2200;

  //lift
  motors[LIFT]->init_encoders(5, 150, 2000, true);
  motors[LIFT]->absolute_setpoint = 979;

  //roll
  motors[ROLL]->init_encoders(6, 10, 2330, false);
  motors[ROLL]->absolute_setpoint = 1800;

  //elbow
  motors[ELBOW]->init_encoders(7, 2365, 3805, true);
  motors[ELBOW]->absolute_setpoint = 2500;
  
  Timer1.initialize(10000);

  
  //Timer3.initialize(1000000);
  Timer1.attachInterrupt(pulse);
  
}

void loop() {
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    motors[i]->_read_encoder();
  }
  nh.spinOnce();
  /*Serial.print("set:");
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
  Serial.println(motors[0]->hlfb);*/
}


void pulse() {
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(motors[0]) ; i++) {
    if (i != SWING) {
      motors[i]->update();
    }
  }
}
