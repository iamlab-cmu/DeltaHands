#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;
std_msgs::Int16MultiArray str_msg;
ros::Publisher chatter("slider", &str_msg);

int num_data = 12;

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(chatter);

}

void loop() {
  int readout[num_data];
  readout[0] = analogRead(A0);
  readout[1] = analogRead(A1);
  readout[2] = analogRead(A2);
  readout[3] = analogRead(A3);
  readout[4] = analogRead(A4);
  readout[5] = analogRead(A5);
  readout[6] = analogRead(A8);
  readout[7] = analogRead(A9);
  readout[8] = analogRead(A10);
  readout[9] = analogRead(A11);
  readout[10] = analogRead(A12);
  readout[11] = analogRead(A13);
  str_msg.data = readout;
  str_msg.data_length = num_data;
  chatter.publish(&str_msg);
  nh.spinOnce();
}
