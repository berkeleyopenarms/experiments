#include <ros.h>`
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <AccelStepper.h>

ros::NodeHandle nh;


std_msgs::Float32 force_msg;
ros::Publisher force_pub("strain_gauge", &force_msg);

//TODO: test service calls more thoroughly? it doesn't seem to be a thoroughly tested feature of rosserial
//ros::ServiceClient<std_srvs::Trigger::Request, std_srvs::Trigger::Response> zero_client("zero_motor");

#define STEPPER_HOME_SPEED 10000
#define STEPPER_MAX_SPEED 15000
#define STEPPER_ACCEL 60000
#define STEPS_PER_MM 800 // 200 steps per revolution * 32 microsteps / 8mm per revolution (TODO: fix this)
#define READ_SAMPLES 500
#define STRAIN_GAUGE_IN A0
#define LIMIT_SWITCH_IN 2
#define STEPPER_STEP_OUT 1
#define STEPPER_DIR_OUT 0

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_OUT, STEPPER_DIR_OUT);

void stepperCb( const std_msgs::Float32& position_msg) {
  long new_pos = (long) (position_msg.data * 1000.0 * STEPS_PER_MM);
  stepper.moveTo(new_pos);

  char str[256];
  sprintf(str, "moving to %ld", new_pos);
  nh.loginfo(str);
}

void zeroCb (const std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  stepper.setSpeed(-1 * STEPPER_HOME_SPEED);
  nh.loginfo("Zeroing...");
  while (digitalRead(LIMIT_SWITCH_IN) == HIGH) {
    stepper.runSpeed();
  }
  stepper.setSpeed(0);
  stepper.setCurrentPosition(0);
  nh.loginfo("Done zeroing, moving to 5mm!");
  stepper.moveTo(5 * STEPS_PER_MM);

  res.success = 1;
}

ros::Subscriber<std_msgs::Float32> sub("stepper_position", &stepperCb );
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> zero_server("zero_motor", &zeroCb);

void setup() {
  pinMode(STRAIN_GAUGE_IN, INPUT);
  pinMode(LIMIT_SWITCH_IN, INPUT_PULLUP);
  analogReadResolution(16);

  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);

  nh.initNode();
  nh.advertise(force_pub);
  nh.advertiseService(zero_server);
  nh.subscribe(sub);
  while (!nh.connected()) nh.spinOnce();
  nh.loginfo("Started up");
}

float avg = 0 ;
float cons = 21128;
float mult = 100.0 / 60.0;
float saved_val = 0;

long count = 0;
void loop() {
  if (stepper.distanceToGo() == 0) {
    float sum = 0;
    for (int i = 0; i < READ_SAMPLES; i++) {
      sum += analogRead(STRAIN_GAUGE_IN);
    }
    avg = avg * 0.93 +  sum * 0.07 / READ_SAMPLES;

    force_msg.data = (avg - cons) * mult;
    force_pub.publish( &force_msg );


    char str[256];
    sprintf(str, "steps left: %ld", stepper.distanceToGo());
    nh.loginfo(str);
  }

  //if (stepper.distanceToGo() != 0)
  stepper.run();

  if (count % 1 == 0)
    nh.spinOnce();

  count++;
}
