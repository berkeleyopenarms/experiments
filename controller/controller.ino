//hardware: Arduino Mega
//This arduino acts as the main controller for this robot arm, it receives the target position from the computer and 
//upload the current position. It grabe the gravity compensition information and add this info to the pid loop. 

#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h> // the input data, an int array with 11 entries [Swing_Gravity(mN), Lifting_gravity(mN), Roll_Gravity(mN), Elbow_gravity, .. the corresponding position]
#include <std_msgs/Int16.h>
#include <Wire.h>

#define Torque_Limit 1.54 //Nm oeak is 7.73 and we now set 20% torque limit
#define TCAADDR 0x70 //the address of the i2c multiplexer
#define Device_addr 0x36 //the slave address

#define Swing_MotorA 1
#define Swing_MotorB 2 //white cable 
#define Elbow_MotorA 3
#define Elbow_MotorB 4
#define Lift_MotorA 5
#define Lift_MotorB 6
#define Roll_MotorA 7
#define Roll_MotorB 8
#define Switch 2
ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::Int16 int_msg;
std_msgs::Int16MultiArray int_array;
char Maxon_not_in_range[] = "Error: the maxon input coordinate should Greater than 0";
ros::Publisher Joint_Position("Joint_Position", &int_array);
ros::Publisher Sys_info("Sys_info", &str_msg);
ros::Publisher debug("debug", &int_msg);
//int abs_position[4] = {0,0,0,0};
int Current_Position = 0;
int pwm_output = 0;
double Maxon_Position = 0; //unit: degree
int servo1_position = 0;
int servo2_position = 0;
byte rtv;
//initalize the PID algorithm using the library
double  L_Input, L_Output, L_Setpoint, L_Gravity;
double  R_Input, R_Output, R_Setpoint, R_Gravity;
double  S_Input, S_Output, S_Setpoint, S_Gravity;
double  E_Input, E_Output, E_Setpoint, E_Gravity;

//double L_Position, S_Position, R_Position, E_Position;//the encoder's value
PID LiftPID(&L_Input, &L_Output, &L_Setpoint, 0.25,0, 0.03, DIRECT); 
PID RollPID(&R_Input, &R_Output, &R_Setpoint, 0.3,0, 0, DIRECT); 
PID ElbowPID(&E_Input, &E_Output, &E_Setpoint, 0.08,0,0.02, DIRECT); 
PID SwingPID(&S_Input, &S_Output, &S_Setpoint, 0.25,0, 0.06, REVERSE);
void servo_control(int p, int i) {
  selector(2);//select the correct chip
  Wire.beginTransmission(8);
  int k = p & 0xff;
  Wire.write(k);
  k = p >> 8;
  Wire.write(k);
  k = i & 0xff;
  Wire.write(k);
  k = i >> 8;
  Wire.write(k);
  Wire.endTransmission();
  
}
void motor_control(const std_msgs::Int16MultiArray& Position) { //input the absolute angle 
  //check if the input is valid(in the range)
  //update the information
  S_Gravity = Position.data[0];//Nm, scale down by 1000, and then deivde by the torque limit.
  S_Gravity = S_Gravity/1000/Torque_Limit*(-127);
  L_Gravity = Position.data[1]; 
  L_Gravity = L_Gravity/1000/Torque_Limit*(127);
  R_Gravity = Position.data[2];
  R_Gravity = R_Gravity/1000/Torque_Limit;
  E_Gravity = Position.data[3];
  E_Gravity = E_Gravity/1000/Torque_Limit*(127);// the message has been reveived
  
  /*convert radius to the encoder value
   * The input angle is the angle relate to the ZERO position,
 */
 
  S_Setpoint = (-Position.data[4]);//Swing, the ZERO position is the right limit, encoder increase when arm moves left. PC: negative angle
  S_Setpoint = S_Setpoint*2048/3141;
  L_Setpoint = (-Position.data[5]);//Lift, the ZERO position is the uplimit. The encoder value increase when the arm moves dowm. PC: negative angle
  L_Setpoint = L_Setpoint*2048/3141;
  R_Setpoint = (-Position.data[6]);// Roll, when maxon is orthogonal with the encoder. Encoder: decrease and the input angle is positive
  R_Setpoint = R_Setpoint*2.42;
  E_Setpoint = Position.data[7];//Encoder value increase and the PC input also positive
  E_Setpoint = E_Setpoint*2048/3141;
  //constrain the range
  S_Setpoint = constrain(S_Setpoint, 0,800);
  R_Setpoint = constrain(R_Setpoint, -3800,0);
  E_Setpoint = constrain(E_Setpoint, 0,1600);
  L_Setpoint = constrain(L_Setpoint, 0,1200);
  Maxon_Position = Position.data[8];
  Maxon_Position = Maxon_Position*180 /3140;
  int i = constrain(Maxon_Position, 0, 120);
  
  //check if the Maxon position larger than 0, if not publish error message and go back to zero
  if (i < 0 ) {
    str_msg.data = Maxon_not_in_range;
    Sys_info.publish(&str_msg);
    i = 0;
  }
  Maxon_Set_Position(i);
  servo1_position = Position.data[9];//12bits info
  servo2_position = Position.data[10];;
  servo_control(servo1_position, servo2_position);//directly publish the servo position to theservo


}
ros::Subscriber<std_msgs::Int16MultiArray> sub( "Coordinate_input", &motor_control);
int switch_read() {
  if (digitalRead(Switch) == 0){
    return 0;
  } else {
    delay(10);
    if (digitalRead(Switch) == 1) {
      return 1;
    } else {
      return 0;
    }
    return 0; //this case will never happen
  }
}
/*7-> lifting encoder 
 *6-> elbow encoder
 *5-> Shoulder roll encoder
 *4-> Swing encoder
 *3-> arduino for the maxon
 *2-> arduino for the last two dynamixal
 */
void selector(int i) {
  if (i>7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1<<i);
  Wire.endTransmission();
}


//first select the i2c channel and then read the i2c device
int encoder_reader() {
  int reading;
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0E));             // sets register pointer 
  Wire.endTransmission();    // stop transmitting
  //assume that we can always receiving the correct bytes
  Wire.requestFrom(0x36, 2);         // request 2 bytes from as5601
  reading = Wire.read();             // receive high byte 
  reading = reading << 8;            // shift high byte to be high 8 bits
  reading |= Wire.read();            // receive low byte as lower 8 bits
  return reading;
}

//first setting the multiplexer to 3
void Maxon_Set_Position(int i) {
  Wire.beginTransmission(8);
  Wire.write(i);
  Wire.endTransmission();
}

int Maxon_Feedback() {
  Wire.requestFrom(8,1);
  Maxon_Position = Wire.read();
}
void setup()
{
  Wire.begin();
  nh.initNode();
  nh.advertise(Sys_info);
  nh.advertise(debug);
  nh.advertise(Joint_Position);
  nh.subscribe(sub);
  //Initalize the io Pins, enable all the motors.
  pinMode(Swing_MotorA, OUTPUT);
  digitalWrite(Swing_MotorA, HIGH);
  pinMode(Swing_MotorB, OUTPUT);//channel control the torque and channel A disable the motor in emergency
  pinMode(Lift_MotorA, OUTPUT);
  digitalWrite(Lift_MotorA, HIGH);
  pinMode(Lift_MotorB, OUTPUT);
  pinMode(Roll_MotorA, OUTPUT);
  digitalWrite(Roll_MotorA, HIGH);
  pinMode(Roll_MotorB, OUTPUT);
  pinMode(Elbow_MotorA, OUTPUT);
  digitalWrite(Elbow_MotorA, HIGH);
  pinMode(Elbow_MotorB, OUTPUT);
  LiftPID.SetOutputLimits(-125,+125);
  LiftPID.SetMode(AUTOMATIC);
  SwingPID.SetOutputLimits(-125,+125);
  SwingPID.SetMode(AUTOMATIC);
  RollPID.SetOutputLimits(-125,+125);
  RollPID.SetMode(AUTOMATIC);
  ElbowPID.SetOutputLimits(-125,+125);
  ElbowPID.SetMode(AUTOMATIC);
  //Serial.begin(9600);
  //Serial.println("the program start");
  selector(7);//get the current position of the first four joints;
  L_Input = encoder_reader() - 1431 - 100; //encoder increase 
  L_Setpoint  =  L_Input;
  selector(6);
  E_Input = encoder_reader() - 700 - 100; //
  E_Setpoint  =  E_Input;
  selector(5);
  R_Input = encoder_reader() - 4000;//will be negative
  R_Setpoint  =  R_Input;
  selector(4);
  S_Input = encoder_reader() - 1618 - 100;
  S_Setpoint  =  S_Input;
  S_Gravity = 0;
  R_Gravity = 0;
  L_Gravity = 0;
  E_Gravity = 0;
  
}

void loop()
{
  //now read the joint encoder grabe all current position + offset and add some safty value
  selector(7);
  L_Input = encoder_reader() - 2400; //encoder decrease when moving down; 
  selector(6);
  E_Input = encoder_reader()- 700 - 100; //
  selector(5);
  R_Input = encoder_reader() - 4000;//will be negative
  selector(4);
  S_Input = encoder_reader() - 1034 - 100;
 //int encoder_data[]= {S_Input, L_Input, R_Input, E_Input};
 //int_array.layout.dim_length = 1;   ???if add this line, the node can not publish info we need 
   int_array.data_length = 6;
   int_array.data[0] = 0;
   int_array.data[1] = S_Input;
   int_array.data[2] = L_Input;
   int_array.data[3] = R_Input;
   int_array.data[4] = E_Input;
   selector(3);
   int_array.data[5] = Maxon_Feedback(); //the current position of the wrist rolling, in degree;
   Joint_Position.publish(&int_array);

   SwingPID.Compute(); //input positive angle(positive encoder value)
   int pwm = 127+S_Output + S_Gravity;//the torque PID gives
   pwm = constrain(pwm, 2, 253);
   analogWrite(Swing_MotorB, pwm);
   ElbowPID.Compute();
   pwm = 127+E_Output + E_Gravity;//the torque PID gives + the gravity compensition;
   int_msg.data = pwm;
   debug.publish(&int_msg);
   pwm = constrain(pwm, 2, 253);
   analogWrite(Elbow_MotorB, pwm);
   
 
   LiftPID.Compute();
   pwm = 127+L_Output + L_Gravity;//the torque PID gives
   pwm = constrain(pwm, 2, 253);
   analogWrite( Lift_MotorB, pwm);
   RollPID.Compute();
   
   nh.spinOnce();
  /*LiftPID.Compute();
  pwm_output = constrain(pwm_output, 1, 254); 
  int_msg.data = Output;
  debug.publish(&int_msg);
  //analogWrite(Motor_B, pwm_output);*/
  
}
