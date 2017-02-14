#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <AccelStepper.h>

ros::NodeHandle nh;


std_msgs::Float32 force_msg;
ros::Publisher force_pub("strain_gauge", &force_msg);

//TODO: test service calls more thoroughly? it doesn't seem to be a thoroughly tested feature of rosserial
//ros::ServiceClient<std_srvs::Trigger::Request, std_srvs::Trigger::Response> zero_client("zero_motor");

#define STEPPER_SPEED 2000
#define STEPS_PER_MM 400 // 200 steps per revolution * 16 microsteps / 8mm per revolution (TODO: fix this)
#define READ_SAMPLES 500
#define STRAIN_GAUGE_IN A0
#define LIMIT_SWITCH_IN 1
#define STEPPER_STEP_OUT 2
#define STEPPER_DIR_OUT 3

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_OUT, STEPPER_DIR_OUT);

void stepperCb( const std_msgs::Float32& position_msg) {
  stepper.moveTo((long) (position_msg.data * STEPS_PER_MM));
}

void zeroCb (const std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  stepper.setSpeed(-1 * STEPPER_SPEED);
  while (digitalRead(LIMIT_SWITCH_IN)) {
    stepper.runSpeed();
  }
}

ros::Subscriber<std_msgs::Float32> sub("stepper_position", &stepperCb );
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> zero_server("zero_motor", &zeroCb);

void setup() {
  pinMode(STRAIN_GAUGE_IN, INPUT);
  pinMode(LIMIT_SWITCH_IN, INPUT);
  analogReadResolution(16);

  stepper.setCurrentPosition(0);

  nh.initNode();
  nh.advertise(force_pub);
  nh.advertiseService(zero_server);
  while (!nh.connected()) nh.spinOnce();
  nh.loginfo("Started up");
}

float avg = 0 ;
float cons = 21128;
float mult = 100.0 / 60.0;
float saved_val = 0;

void loop() {
  float sum = 0;
  for (int i = 0; i < READ_SAMPLES; i++) {
    sum += analogRead(STRAIN_GAUGE_IN);
  }
  avg = avg * 0.93 +  sum * 0.07 / READ_SAMPLES;

  force_msg.data = (avg - cons) * mult;
  force_pub.publish( &force_msg );

  stepper.runSpeed();

  nh.spinOnce();
}
