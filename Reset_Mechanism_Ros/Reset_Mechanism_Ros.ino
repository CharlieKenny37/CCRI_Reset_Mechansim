//necessary libraries
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <infrastructure_msgs/Drawer.h>

#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include <TimerInterrupt.h>

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

// PID Coefficients
#define Kp 1
#define Kd 0.5

// Turntable Rotation Tolerance
#define TURNTABLE_TOLERANCE 2

// Init the stepper motors
AccelStepper linear_actuator_motor( AccelStepper::DRIVER, LINEAR_ACTUATOR_STEPPER_PULSE_PIN, LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN);
AccelStepper spool_motor( AccelStepper::DRIVER, SPOOL_STEPPER_PULSE_PIN, SPOOL_STEPPER_DIRECTION_PIN);
AccelStepper z_axis_motor( AccelStepper::DRIVER, Z_AXIS_STEPPER_PULSE_PIN, Z_AXIS_STEPPER_DIRECTION_PIN);
AccelStepper x_axis_motor( AccelStepper::DRIVER, X_AXIS_STEPPER_PULSE_PIN, X_AXIS_STEPPER_DIRECTION_PIN);
AccelStepper arm_motor( AccelStepper::DRIVER, ARM_STEPPER_PULSE_PIN, ARM_STEPPER_DIRECTION_PIN);

// PID Controller Setup
PID_v2 PIDController(Kp, 0, Kd, PID::Direct);

// current encoder reading variable
volatile long encoder_reading = 0;
volatile bool turntable_clockwise = true;

// Current object on the reset mechanism
uint8_t current_object;

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

  //configure the acceleration rates of the stepper motors
  linear_actuator_motor.setAcceleration(200);
  spool_motor.setAcceleration(200);
  z_axis_motor.setAcceleration(200);
  x_axis_motor.setAcceleration(200);
  arm_motor.setAcceleration(200);

  //setup the enable pins on the motors
  linear_actuator_motor.setEnablePin(LINEAR_ACTUATOR_STEPPER_ENABLE_PIN);
  spool_motor.setEnablePin(SPOOL_STEPPER_ENABLE_PIN);
  z_axis_motor.setEnablePin(Z_AXIS_STEPPER_ENABLE_PIN);
  x_axis_motor.setEnablePin(X_AXIS_STEPPER_ENABLE_PIN);
  arm_motor.setEnablePin(ARM_STEPPER_ENABLE_PIN);

  //home carriage & linear actuator
  home_motors();
  
}


// ISRs

//Encoder Pin
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

//Timer
void TimerHandler()
{
  linear_actuator_motor.run();
  spool_motor.run();
  z_axis_motor.run();
  x_axis_motor.run();
  
}

#define TIMER_INTERVAL_MS        1L

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
  n.loginfo("Homing carriage and linear actuator motors");

  //change action status publisher
  action_status.data = true;
  statuspub.publish(&action_status);

  bool x_motor_moving = true;
  bool z_motor_moving = true;
  bool linear_actuator_moving = true;

  //Set the stepper motor speeds for the motors that need to be homed
  linear_actuator_motor.setSpeed(-200);
  z_axis_motor.setSpeed(-200);
  x_axis_motor.setSpeed(-200);

  //Loop while any of the motors that are being homed are moving
  while(linear_actuator_moving || x_motor_moving || z_motor_moving)
  {
    //Check if the motor limit switches have been triggered
    if(linear_actuator_moving)
    {
      if(get_linear_actuator_switch())
      {
        //Stop the motors if the limit switches are triggered
        linear_actuator_motor.setSpeed(0);
        linear_actuator_moving = false;
      }
    }
    
    if(z_motor_moving)
    {
      if(get_z_axis_switch())
      {
        z_axis_motor.setSpeed(0);
        z_motor_moving = false;
      }
    }

    if(x_motor_moving)
    {
      if(get_x_axis_switch())
      {
        x_axis_motor.setSpeed(0);
        x_motor_moving = false;
      }
    }
  }

  //The motors are now in their home positions so reset the current positions on the stepper motors
  linear_actuator_motor.setCurrentPosition(0);
  z_axis_motor.setCurrentPosition(0);
  x_axis_motor.setCurrentPosition(0);

  //Log that motors are finished being homed
  n.loginfo("Finished homing carriage and linear actuator motors");

  //Move the carriage and linear actuator to a good setup location
  // TO DETERMINE
  
}


void reset_current_object()
{
  
}


void swap_current_object(uint8_t new_object)
{
  n.loginfo("Swapping current object on the reset mechansim");
  
  // Check if the new object is already the object on the reset mechanism
  if( new_object == current_object )
  {
    n.loginfo("The desired object is already on the reset mechansim. Stopping swap algorithm");
    return;
  }

  int z_position_of_new_object;
  int x_position_of_new_object;
  
  // Determine where the carraige needs to travel to
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

void publish_sensor_data()
{
  data.turntable_position = encoder_reading;
  
  data.linear_actuator_position = linear_actuator_motor.currentPosition();
  data.spool_position = spool_motor.currentPosition();
  data.x_axis_position = z_axis_motor.currentPosition();
  data.z_axis_position = x_axis_motor.currentPosition();
  data.arm_position = arm_motor.currentPosition();

  data.linear_actuator_switch = get_linear_actuator_switch();
  data.z_axis_switch = get_z_axis_switch();
  data.x_axis_actuator_switch = get_x_axis_switch();

  data.current_object = current_object;
  
  data.header.stamp = n.now(); //gets the time of the device roscore is running on
  datapub.publish(&data);
}

void move_turntable_to_position(int desired_position)
{
  //PID Initialization
  PIDController.Start(encoder_reading,        // input
                      0,                      // current output
                      desired_position);      // setpoint

  // Run the motors while the position of the turntable is not close to the desired position
  while( abs(encoder_reading - desired_position) < TURNTABLE_TOLERANCE) )
  {
    int output = PIDController.Run(encoder_reading);
    if( desired_position < encoder_reading )
    {
      set_turntable_motors(false, output);
    }
    else
    {
      set_turntable_motors(true, output);
    }
  }

  //Once the turntable has rotated to the desired position, ensure that the motors have stopped moving
  set_turntable_motors(true, 0);
  
}

void set_turntable_motors(bool clockwise, uint8_t motor_speed)
{
  //Store the current direction the motor is turning so that the encoder can count in the right direction
  turntable_clockwise = clockwise;

  //Set the voltages on the motor1 output pins to set the desired motor output
  digitalWrite(TURNTABLE_MOTOR_1_DIRECTION_PIN, clockwise);
  analogWrite(TURNTABLE_MOTOR_1_ENABLE_PIN, motor_speed);

  //Set the voltages on the motor2 output pins to set the desired motor output
  digitalWrite(TURNTABLE_MOTOR_2_DIRECTION_PIN, clockwise);
  analogWrite(TURNTABLE_MOTOR_2_ENABLE_PIN, motor_speed);
}
