#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

#define USE_USBCON

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>


#define NUM_MOTORS 12
#define MY_ID 0
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

// MC1 (60) - ADC1 (49); MC2(62) - ADC0 (48);
// ################################## Feather MC and ADC Libraries INIT #####################3
Adafruit_MotorShield MC0 = Adafruit_MotorShield(0x62);
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60);
Adafruit_MotorShield MC2 = Adafruit_MotorShield(0x61);

Adafruit_DCMotor *MC0_M1 = MC0.getMotor(1);
Adafruit_DCMotor *MC0_M2 = MC0.getMotor(2);
Adafruit_DCMotor *MC0_M3 = MC0.getMotor(3);
Adafruit_DCMotor *MC0_M4 = MC0.getMotor(4);
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(1);
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(2);
Adafruit_DCMotor *MC1_M3 = MC1.getMotor(3);
Adafruit_DCMotor *MC1_M4 = MC1.getMotor(4);
Adafruit_DCMotor *MC2_M1 = MC2.getMotor(1);
Adafruit_DCMotor *MC2_M2 = MC2.getMotor(2);
Adafruit_DCMotor *MC2_M3 = MC2.getMotor(3);
Adafruit_DCMotor *MC2_M4 = MC2.getMotor(4);

Adafruit_DCMotor* motors[NUM_MOTORS] = {MC0_M1,MC0_M2,MC1_M1,// 1st robot
                                        MC1_M2,MC2_M1,MC2_M2,// 2nd robot
                                        MC2_M3,MC2_M4,MC1_M3,// 3rd robot
                                        MC1_M4,MC0_M3,MC0_M4,// 4th robot
                                        };


Adafruit_ADS1015 ADC2;
Adafruit_ADS1015 ADC1;
Adafruit_ADS1015 ADC0;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC2, &ADC2, &ADC1,//1st robot
                                      &ADC1, &ADC0, &ADC0,//2nd robot
                                      &ADC0, &ADC0, &ADC1,//3rd robot
                                      &ADC1, &ADC2, &ADC2,//4th robot
                                      };

int channels[NUM_MOTORS] = {0,1,0,//1st robot
                            1,0,1,//2nd robot
                            2,3,2,//3rd robot
                            3,2,3,//4th robot
                            };

int motor_val[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float desired_joint_positions[NUM_MOTORS];
float desired_joint_velocities[NUM_MOTORS];

float joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_velocities[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float last_joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//#################################### GLOBAL VARIABLES ##################################//

unsigned long current_arduino_time;
unsigned long last_arduino_time;
float time_elapsed;

int sampleTime = 0;


float position_threshold = 0.0005; //0.15mm threshold
float p = 400; //350;//390.0;
float i_pid = 0.0;//0.1;//0.25;
float d = 0.0;//3.75;// 0.5

float last_joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float total_joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// ### publish joint states with ros
ros::NodeHandle nh;
std_msgs::Float32MultiArray joint_msg;
ros::Publisher chatter("deltaJointState", &joint_msg);

void controlCallback(const std_msgs::Float32MultiArray& control_msg){
  bool reset = control_msg.data[0] > 0.5;
  bool stop_control = control_msg.data[1] > 0.5;
  bool start_control = control_msg.data[2] > 0.5;
  bool position_control = control_msg.data[3] > 0.5;
//
  if (reset){
      resetJoints();
  }
  if (stop_control){
      stopJoints();
  }
  if (start_control){
      startJoints();
  }
  if (position_control){
    for(int j = 0; j < NUM_MOTORS; j++)
    {
      desired_joint_positions[j] = control_msg.data[4+j];
    }
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("deltaControl", controlCallback);

// whether the motors are able to move
bool move_flag = true;

void resetJoints(){
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    desired_joint_positions[i] = 0.001;
  }
}

void stopJoints(){
  move_flag = false;
  // Turn off all motors
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    if(motors[i] != NULL){
      motors[i]->setSpeed(0);
      motors[i]->run(RELEASE);
    }
  }
}

void startJoints(){
  move_flag = true;
}


void updateJointPositions()
{
    for(int i = 0; i < NUM_MOTORS; i++)
    {
        if(motors[i] != NULL)
        {
            motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
            joint_positions[i] = motor_val[i] * 0.003 * 0.02 / 5.0;
        }
    }
}

void setup()
{
//    Serial.begin(57600);

    // set all the base dc motor control pins to outputs
    MC0.begin();
    MC1.begin();
    MC2.begin();
    // start all the ADCs
    ADC2.begin(0x4A);
    ADC1.begin(0x49);
    ADC0.begin(0x48);

    ADC2.setGain(GAIN_ONE);
    ADC1.setGain(GAIN_ONE);
    ADC0.setGain(GAIN_ONE);

    for(int i=0; i<NUM_MOTORS; i++){
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
    }

    updateJointPositions();
    resetJoints();
    moveDeltaPosition();

    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);


}

void moveDeltaPosition()
{
  bool reached_point = true;
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    if(motors[i]!=NULL)
    {
      joint_errors[i] = joint_positions[i] - desired_joint_positions[i];
      if(fabs(joint_errors[i]) > position_threshold)
      {
        reached_point = false;
      }
    }
  }

  if(reached_point==false and move_flag==true)
  {
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      if(motors[i] != NULL){
        if(joint_errors[i] > position_threshold)
        {
          float pid = p * joint_errors[i];
          int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); //change it to use duration input
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(BACKWARD);
          joint_velocities[i] = float(motor_speed);
          if(joint_errors[i] < 0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else if(joint_errors[i] < -position_threshold)
        {
          float pid = p * joint_errors[i];
          int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
          joint_velocities[i] = float(motor_speed);
          if(joint_errors[i] > -0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else
        {
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
          joint_velocities[i] = 0.0;
          total_joint_errors[i] += 0.0;
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      if(motors[i] != NULL){
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        joint_velocities[i] = 0.0;
        total_joint_errors[i] = 0.0;
      }
    }
  }

}

// LOOP CODE
void loop()
{
  updateJointPositions();
  // move to the desired position is it is not finished yet and it is not stopped
  moveDeltaPosition();
  joint_msg.data = joint_positions;
  joint_msg.data_length = NUM_MOTORS;
  chatter.publish(&joint_msg);
  nh.spinOnce();
}
