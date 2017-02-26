# twisted string test setup

All development + testing so far has been done on Ubuntu 14.04 running ROS Indigo.

## Steps for setting up & launching stuff:

1. Source the ROS setup script
   - `source /opt/ros/indigo/setup.bash`
2. Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace); this is where all our code will live
   - `cd ~`
   - `mkdir -p ~/twisted_string_ws/src`
   - `cd ~/twisted_string_ws/src`
   - `catkin_init_workspace`
3. Install some packages -- repos should be cloned into our `twisted_string_ws/src` directory
   - `git clone git@github.com:brentyi/rosserial.git` (for talking to the teensy/arduino)
   - `git clone git@github.com:brentyi/vesc.git` (for talking to the vesc)
   - `sudo apt-get install ros-indigo-ackermann-msgs` (we won't actually use this, but the vesc package won't build without it)
4. Build & source our workspace
   - `cd ~/twisted_string_ws`
   - `catkin_make`
   - `source devel/setup.bash`
5. Physically plug everything in!
   - 2 USB cables (Teensy & VESC)
   - ~12V, >2A power to the motor drivers
6. Launch the ROS nodes, in separate terminal windows:
   - VESC driver: `roslaunch vesc_driver vesc_driver_node.launch port:=/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_301-if00 rotor_position_source:=encoder`
   - Arduino driver: `rosrun rosserial_python serial_node.py /dev/serial/by-id/usb-Teensyduino_USB_Serial_1979910-if00`
   - There's a small possibility that your serial paths will be different from mine; run `ls /dev/serial/by-id` with the cables plugged in and adjust accordingly
7. Example commands for talking to stuff (must source the ROS setup script before running commands):
   - Run motor @ 1A: `rostopic pub -r 10 /commands/motor/current std_msgs/Float64 -- 1`
   - Reverse motor @ 1A: `rostopic pub -r 10 /commands/motor/current std_msgs/Float64 -- -1`
   - Zero stepper:  `rosservice call /zero_motor`
   - Move stepper position to 0.1m: `rostopic pub /stepper_position std_msgs/Float32 -- 0.1`
   - Print out strain gauge information: `rostopic echo /strain_gauge`
   - Print out brushless motor position: `rostopic echo /sensors/rotor_position`
   - Print out misc brushless motor data: `rostopic echo /sensors/core`
   - Launch plotting application: `rosrun rqt_plot rqt_plot`
