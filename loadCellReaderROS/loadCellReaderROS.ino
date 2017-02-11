#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 force_msg;
ros::Publisher force_pub("strain_gauge", &force_msg);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(force_pub);
  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  analogReadResolution(16);
}
float val1;
float val2;
float avg = 0 ;
float cons = 21128;
float mult = 100.0/60.0;
float saved_val = 0;
#define n 500.0

void loop() {
  val1 =0;
  
  for (int i = 0; i<n; i++){
    val1 = val1 + analogRead(A0);
  }
  val1 = val1/n;
  avg = avg*0.93 +  val1*0.07;
  
  force_msg.data = avg;
  force_pub.publish( &force_msg );
  nh.spinOnce();
}
