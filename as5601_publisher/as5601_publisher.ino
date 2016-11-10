#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 msg;
ros::Publisher chatter("chatter", &msg);


void setup()
{
    nh.initNode();
    nh.advertise(chatter);
}

void loop()
{
    int16_t reading;
    Wire.beginTransmission(0x36);       // transmit to device as5601
    Wire.write(byte(0x0E));             // sets register pointer 
    Wire.endTransmission();    // stop transmitting
    //assume that we can always receiving the correct bytes
    Wire.requestFrom(0x36, 2);         // request 2 bytes from as5601
    reading = Wire.read();             // receive high byte 
    reading = reading << 8;            // shift high byte to be high 8 bits
    reading |= Wire.read();            // receive low byte as lower 8 bits

    msg.data = reading;

    chatter.publish( &msg );
    nh.spinOnce();
    delay(1000);
}
