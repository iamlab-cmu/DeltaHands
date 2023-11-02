#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>

#define NUM_MOTORS 3

// Create the three motor shield object I2C address input
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60);

// Create 3 motor objects
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(1);
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(2);
Adafruit_DCMotor *MC1_M4 = MC1.getMotor(4);

// create array of pointers to motor objects
Adafruit_DCMotor* motors[NUM_MOTORS] = {MC1_M1, MC1_M2, MC1_M4};

// create object to access off-board analog-to-digital converter
Adafruit_ADS1015 ADC0;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC0, &ADC0, &ADC0};
int channels[NUM_MOTORS] = {0,1,2};

// Calculate based on max input size expected for one command
#define MAX_INPUT_SIZE 900

float last_joint_positions[3] = {0.0, 0.0, 0.0};
float joint_positions[3] = {0.0, 0.0, 0.0};
float joint_velocities[3] = {0.0, 0.0, 0.0};
int motor_val[3] = {0, 0, 0};

// STOP COMMAND
// Stop Command Globals
bool stop_flag = false;

// Stop Command
// Immediately halts everything by setting all the motor control pins to LOW and sets the stop_flag to true.
// Remember to publish a false msg to restart the robot.
void stop(bool stop_msg){

  // If the stop_msg contains true, stop all of the motors
  if(stop_msg)
  {
    // set stop_flag to true
    stop_flag = true;

    // Turn off all motors
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      if(motors[i] != NULL){
        motors[i]->run(RELEASE);
      }
    }
  }
  // Otherwise if the stop_msg contains false, set the stop_flag back to false
  else
  {
    stop_flag = false;
  }
}

// Desired Motor Positions and Velocities (hard cap of 10 trajectory points)
float desired_joint_positions[10][NUM_MOTORS];
float desired_joint_velocities[10][NUM_MOTORS];
float durations[10];
float desired_motor_speed[10];

float last_time = 0.0;
int num_trajectory_points = 0;
int current_trajectory_point = 0;
bool position_trajectory = false;
bool velocity_trajectory = false;

// Get next command from Serial (add 1 for final 0)
char inputString[MAX_INPUT_SIZE + 1];

bool newData = false;
int currentStringLength = 0;
int ndx = 0;
bool recvInProgress = false;
char startMarker = '<';
char endMarker = '>';

void recvWithStartEndMarkers() {
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        inputString[ndx] = rc;
        ndx++;
      }
      else {
        inputString[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        currentStringLength = ndx;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// Position Trajectory
// Stores the Position Trajectory to desired_joint_positions
void positionTrajectory()
{
  String received = "p ";

  Serial.println(received);

  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputString,",");      // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);
  Serial.println(num_trajectory_points);
  for(int i = 0; i < num_trajectory_points; i++)
  {
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    durations[i] = atof(strtokIndx);
    received += String(durations[i]) + " ";

    for(int j = 0; j < NUM_MOTORS; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_positions[i][j] = atof(strtokIndx);
      received += String(desired_joint_positions[i][j],4) + " ";
    }

    Serial.println(received);
  }

  position_trajectory = true;
  velocity_trajectory = false;
  current_trajectory_point = 0;
  last_time = 0.0;
}

// Velocity Trajectory
// Stores the Velocity Trajectory to desired_joint_velocities
void velocityTrajectory()
{
  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputString,",");      // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);

  for(int i = 0; i < num_trajectory_points; i++)
  {
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    durations[i] = atof(strtokIndx);
    for(int j = 0; j < NUM_MOTORS; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_velocities[i][j] = atof(strtokIndx);
    }
  }
  position_trajectory = false;
  velocity_trajectory = true;
  current_trajectory_point = 0;
  last_time = 0.0;
}

float max_motor_speed[NUM_MOTORS] = {0.025, 0.025, 0.025};

// RESET COMMAND
// Reset Command Callback
// Resets the robot's desired positions to the default positions.
void resetJoints(){

  num_trajectory_points = 1;
  durations[0] = 4.0;
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    desired_joint_positions[0][i] = 0.001;
  }

  position_trajectory = true;
  velocity_trajectory = false;
  current_trajectory_point = 0;
  last_time = 0.0;
}

unsigned long current_arduino_time;
unsigned long last_arduino_time;
float time_elapsed;
String joint_states;

void updateJointPositions()
{
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    if(motors[i] != NULL){
        motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
        joint_positions[i] = motor_val[i] * 0.003 * 0.02 / 5.0; // 1 bit = 3mV 0.02 mm stroke 5V range
    }
  }
}

// SETUP CODE
void setup()
{
  // set all the base dc motor control pins to outputs
  MC1.begin();

  // start all the ADCs
  ADC0.begin(0x48);

  ADC0.setGain(GAIN_ONE);

  // disable dc motors by setting their enable lines to low
  for(int i=0; i<NUM_MOTORS; i++){
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }

  Serial.begin(57600);           //  setup serial

  updateJointPositions();
  joint_states.reserve(200); // Reserve 200 Bytes for joint_states
  resetJoints();
  moveDeltaPosition();
}


void checkIfDoneMovingDeltas()
{

  if(current_trajectory_point == num_trajectory_points)
  {
    position_trajectory = false;
    velocity_trajectory = false;
    current_trajectory_point = 0;
    num_trajectory_points = 0;
    last_time = 0.0;
    Serial.println("d");
    Serial.println("d");
  }
}

int sampleTime = 0;


float position_threshold = 0.0005; //0.15mm threshold
float p = 350;//390.0;
float i_pid = 0.1;//0.25;
float d = 3.75;// 0.5

float last_joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0};
float joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0};
float total_joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0};

void moveDeltaPosition()
{
  bool reached_point = true;
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    if(motors[i]!=NULL)
    {
      joint_errors[i] = joint_positions[i] - desired_joint_positions[current_trajectory_point][i];
      Serial.print(joint_errors[i],4);
      Serial.print(",");
      if(fabs(joint_errors[i]) > position_threshold)
      {
        reached_point = false;
      }
    }
  }
  Serial.println();
  if(reached_point)
  {
    Serial.println("reached point, onto next trajectory");
    current_trajectory_point += 1;
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      total_joint_errors[i] = 0.0;
    }
  }

  if(current_trajectory_point < num_trajectory_points)
  {
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      if(motors[i] != NULL){
        if(joint_errors[i] > position_threshold)
        {
          float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
          int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); //change it to use duration input
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(BACKWARD);
          joint_velocities[i] = -max_motor_speed[i];
          if(joint_errors[i] < 0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else if(joint_errors[i] < -position_threshold)
        {
          float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
          int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
          joint_velocities[i] = max_motor_speed[i];
          if(joint_errors[i] > -0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else
        {
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
          joint_velocities[i] = 0.0;
          total_joint_errors[i] = 0.0;
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
    Serial.println("end of all available trajectories");
    //position_trajectory = false; //finished moving all commands
  }
}

void moveDeltaVelocity()
{
  if(last_time >= durations[current_trajectory_point])
  {
    current_trajectory_point += 1;
  }
  if(current_trajectory_point < num_trajectory_points)
  {
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      if(motors[i] != NULL){
        joint_velocities[i] = desired_joint_velocities[current_trajectory_point][i];
        if(joint_velocities[i] < 0.0)
        {
          int motor_speed = (int)(min((-joint_velocities[i]/ max_motor_speed[i]), 1.0) * 255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(BACKWARD);
        }
        else if(joint_velocities[i] > 0.0)
        {
          int motor_speed = (int)(min((joint_velocities[i]/ max_motor_speed[i]), 1.0) * 255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
        }
        else
        {
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
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
        if(joint_positions[i] < 0.0)
        {
          joint_positions[i] = 0.0;
        }
      }
    }
  }
}

void publishJointStates()
{
  joint_states = "j,";

  for(int i = 0; i < NUM_MOTORS; i++)
  {
    joint_states += String(joint_positions[i],4) + "," + String(joint_velocities[i],4) + ",";
  }
  Serial.println(joint_states);
}

// LOOP CODE
void loop()
{
  recvWithStartEndMarkers();

  if (newData) {

    Serial.println(inputString);
    switch (inputString[0]) {
      // Stop Command
      case 's':
        stop(inputString[2]-'0');
        break;
      // Reset Command
      case 'r':
        resetJoints();
        break;
      // Position Command
      case 'p':
        positionTrajectory();
        break;
      // Velocity Command
      case 'v':
        velocityTrajectory();
        break;
      default:
        // do nothing
        break;
    }
    newData = false;
  }

  // If the robot is not currently in the stop mode
  if(!stop_flag)
  {
    if(position_trajectory)
    {
      moveDeltaPosition();
    }
    else if(velocity_trajectory)
    {
      moveDeltaVelocity();
    }

    checkIfDoneMovingDeltas();
  }

  if (sampleTime == 0) {
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      last_joint_positions[i] = joint_positions[i];
    }
    updateJointPositions();

    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
    last_time += time_elapsed;
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      joint_velocities[i] = (joint_positions[i] - last_joint_positions[i]) / time_elapsed;
    }
    last_arduino_time = current_arduino_time;

    publishJointStates();
  }
  sampleTime += 1;
  sampleTime = sampleTime % 1;
}
