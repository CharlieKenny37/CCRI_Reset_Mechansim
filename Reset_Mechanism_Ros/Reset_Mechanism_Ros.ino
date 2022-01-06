//necessary libraries
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <infrastructure_msgs/Drawer.h>

// Stepper Motor Pins
#define LINEAR_ACTUATOR_STEPPER_PULSE_PIN 0
#define LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN 0
#define LINEAR_ACTUATOR_STEPPER_ENABLE_PIN 0

#define SPOOL_STEPPER_PULSE_PIN 0
#define SPOOL_STEPPER_DIRECTION_PIN 0
#define SPOOL_STEPPER_ENABLE_PIN 0

#define Z_AXIS_STEPPER_PULSE_PIN 0
#define Z_AXIS_STEPPER_DIRECTION_PIN 0
#define Z_AXIS_STEPPER_ENABLE_PIN 0

#define X_AXIS_STEPPER_PULSE_PIN 0
#define X_AXIS_STEPPER_DIRECTION_PIN 0
#define X_AXIS_STEPPER_ENABLE_PIN 0

#define ARM_STEPPER_PULSE_PIN 0
#define ARM_STEPPER_DIRECTION_PIN 0
#define ARM_STEPPER_ENABLE_PIN 0

// DC Motor Pins
#define TURNTABLE_MOTOR_1_ENABLE_PIN 0
#define TURNTABLE_MOTOR_1_DIRECTION_PIN 0
#define TURNTABLE_MOTOR_2_ENABLE_PIN 0
#define TURNTABLE_MOTOR_2_DIRECTION_PIN 0 

// Limit Switch Pins
#define LINEAR_ACTUATOR_SWITCH_PIN 0
#define Z_AXIS_SWITCH_PIN 0
#define X_AXIS_SWITCH_PIN 0

// Photo Interrupt Pin
#define PHOTO_INTERRUPT_PIN A0

// Init the stepper motors
AccelStepper linear_actuator_motor( AccelStepper::DRIVER, LINEAR_ACTUATOR_STEPPER_PULSE_PIN, LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN);
AccelStepper spool_motor( AccelStepper::DRIVER, SPOOL_STEPPER_PULSE_PIN, SPOOL_STEPPER_DIRECTION_PIN);
AccelStepper z_axis_motor( AccelStepper::DRIVER, Z_AXIS_STEPPER_PULSE_PIN, Z_AXIS_STEPPER_DIRECTION_PIN);
AccelStepper x_axis_motor( AccelStepper::DRIVER, X_AXIS_STEPPER_PULSE_PIN, X_AXIS_STEPPER_DIRECTION_PIN);
AccelStepper arm_motor( AccelStepper::DRIVER, ARM_STEPPER_PULSE_PIN, ARM_STEPPER_DIRECTION_PIN);

// current encoder reading variable
volatile long encoder_reading = 0;
volatile bool turntable_clockwise = true;

//ros variables
ros::NodeHandle n;
infrastructure_msgs::Reset_Mechansim data;
std_msgs::Bool action_status;

ros::Publisher data_pub("Reset_Mechanism/Data", &data);
ros::Publisher status_pub("Reset_Mechanism/Movement_Status", &action_status);

//ros callback functions for reset_current and reset_drawer services
void reset_current_object_callback(const std_msgs::Empty& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  reset_current_object();
  action_status.data = false;
  statuspub.publish(&action_status);
}

void swap_current_object_callback(const std_msgs::UInt8& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  swap_current_object(msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}

ros::Subscriber<std_msgs::Empty> reset_sub("Reset_Mechanism/Reset_Current_Object", &reset_current_object_callback);
ros::Subscriber<std_msgs::UInt8> swap_sub("Reset_Mechanism/Swap_Current_Object", &reset_current_object_callback);


void setup() 
{
  
  //Setup ros pubs and services
  n.initNode();
  n.advertise(data_pub);
  n.advertise(status_pub);
  n.subscribe(reset_sub);
  n.subscribe(swap_sub);

  //Set the baud rate of the serial port
  n.getHardware()->setBaud(250000);

  
  //Configure pin directions
  pinMode(LINEAR_ACTUATOR_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(LINEAR_ACTUATOR_STEPPER_ENABLE_PIN, OUTPUT);
  
  pinMode(SPOOL_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(SPOOL_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(SPOOL_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(Z_AXIS_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(Z_AXIS_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(Z_AXIS_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(X_AXIS_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(X_AXIS_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(X_AXIS_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(ARM_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(ARM_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(ARM_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(TURNTABLE_MOTOR_1_ENABLE_PIN, OUTPUT);
  pinMode(TURNTABLE_MOTOR_1_DIRECTION_PIN, OUTPUT);
  pinMode(TURNTABLE_MOTOR_2_ENABLE_PIN, OUTPUT);
  pinMode(TURNTABLE_MOTOR_2_DIRECTION_PIN, OUTPUT);

  pinMode(LINEAR_ACTUATOR_SWITCH_PIN, INPUT);
  pinMode(Z_AXIS_SWITCH_PIN, INPUT);
  pinMode(X_AXIS_SWITCH_PIN, INPUT);

  pinMode(PHOTO_INTERRUPT_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoder_pin), update_encoder, CHANGE);

  //configure the max speed for the motors
  linear_actuator_motor.setMaxSpeed(800);
  spool_motor.setMaxSpeed(800);
  z_axis_motor.setMaxSpeed(800);
  x_axis_motor.setMaxSpeed(800);
  arm_motor.setMaxSpeed(800);

  //setup the enable pins on the motors
  linear_actuator_motor.setEnablePin(LINEAR_ACTUATOR_STEPPER_ENABLE_PIN);
  spool_motor.setEnablePin(SPOOL_STEPPER_ENABLE_PIN);
  z_axis_motor.setEnablePin(Z_AXIS_STEPPER_ENABLE_PIN);
  x_axis_motor.setEnablePin(X_AXIS_STEPPER_ENABLE_PIN);
  arm_motor.setEnablePin(ARM_STEPPER_ENABLE_PIN);

  //home carriage & linear actuator
  home_motors();
  
}

void update_encoder()
{
  if(turntable_clockwise)
  {
    encoder_reading++;
  }
  else
  {
    encoder_reading--;
  }
}

void loop()
{
  publish_sensor_data();
  n.spinOnce();

  //Loop rate control
  //uncomment if desired
  /* int loop_end_time = millis() + (1/LOOP_RATE_HZ) * 1000;
  while(millis() < loop_end_time); */
}

void home_motors()
{
  //Log that motors are being homed
  n.loginfo("Starting drawer reset");

  //change action status publisher
  action_status.data = true;
  statuspub.publish(&action_status);

  bool motors mov
  while(

  
}

bool get_linear_actuator_switch()
{
  return !digitalRead(LINEAR_ACTUATOR_SWITCH_PIN);
}

bool get_z_axis_switch()
{
  return !digitalRead(Z_AXIS_SWITCH_PIN);
}

bool get_x_axis_switch()
{
  return !digitalRead(X_AXIS_SWITCH_PIN);
}

void set_turntable_motor_1(bool clockwise, uint8_t motor_speed)
{
  digitalWrite(TURNTABLE_MOTOR_1_DIRECTION_PIN, clockwise);
  analogWrite(TURNTABLE_MOTOR_1_ENABLE_PIN, motor_speed);
}

void set_turntable_motor_2(bool clockwise, uint8_t motor_speed)
{
  digitalWrite(TURNTABLE_MOTOR_2_DIRECTION_PIN, clockwise);
  analogWrite(TURNTABLE_MOTOR_2_ENABLE_PIN, motor_speed);
}
