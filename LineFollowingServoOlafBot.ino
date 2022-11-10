#define IR_SENSOR_RIGHT 11 //assigns right IR sensor to pin 11
#define IR_SENSOR_LEFT 12 //assigns left IR sensor to pin 12
#define MOTOR_SPEED 255 //assigns speed of the robot, please select a value from 180 to 255
#define Direction 1 //variable used in conditional statements
#include <Servo.h>

int pos = 0;  //sets the initial position of the servo to 0 degrees
Servo servo_3; // assigns servo to pin #3 on Arduino 

//Right motor
int enableRightMotor = 6; //assigns PWM value to right motor
int rightMotorPin1 = 7; //assigns H-bridge switch to arduino board
int rightMotorPin2 = 8; //assigns H-bridge switch to arduino board

//Left motor
int enableLeftMotor = 5; //assigns PWM value to left motor
int leftMotorPin1 = 9; //assigns H-bridge switch to arduino board
int leftMotorPin2 = 10; //assigns H-bridge switch to arduino board

void setup()
{
  servo_3.attach(3, 500, 2500);  // assigns servo to pin #3 and initializes position
  
   //TCCR0B = TCCR0B & B11111000 | B00000010 ; //allows for controlled speed to change PWM frequency by user
 
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);      //Configures these variables as an output: provide substantial amount of current
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);      //Configures these variables as an output: provide substantial amount of current

  pinMode(IR_SENSOR_RIGHT, INPUT);    // Configures these varibales as an input: acts as resistor to read sensor, uses small current
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotor(0, 0);
}


void loop()
{

  // sweep the servo from 0 to 180 degrees in steps
  // of 2 degrees
  for (pos = 0; pos <= 180; pos += 2) {
    // tell servo to go to position in variable 'pos'
    servo_3.write(pos);
    // wait 15 ms for servo to reach the position
    delay(15); // Wait for 15 millisecond(s)
  }
  
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);  //Reads the value from a specified digital pin, either HIGH or LOW.
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(Direction, Direction); //conditional statements used to determine motor direction in rotateMotor Function
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
    rotateMotor(-Direction, Direction);//conditional statements used to determine motor direction in rotateMotor Function
  }
  //If left sensor detects black line, then turn left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
    rotateMotor(Direction, -Direction);//conditional statements used to determine motor direction in rotateMotor Function
  }
  //If both the sensors detect black line, then stop
  else
  {
    rotateMotor(0, 0);  //the motor does not spin
  }

  for (pos = 180; pos >= 0; pos -= 2) {
    // tell servo to go to position in variable 'pos'
    servo_3.write(pos);
    // wait 15 ms for servo to reach the position
    delay(15); // Wait for 15 millisecond(s)
  }
 
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) //rotateMotor is a function of right and left motor speed
{

  if (rightMotorSpeed == Direction) //if right motor speed = 1
  {
    digitalWrite(rightMotorPin1, LOW); //opens switch on H-Bridge (breaks the circuit)
    digitalWrite(rightMotorPin2, HIGH); //closes switch on H-Bridge (makes the circuit)
  }
  else if (rightMotorSpeed == -Direction)  //if right motor speed = -1
  {
    digitalWrite(rightMotorPin1, HIGH); //closes switch on H-Bridge (makes the circuit)
    digitalWrite(rightMotorPin2, LOW); //opens switch on H-Bridge (breaks the circuit)
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW); //opens switch on H-Bridge (breaks the circuit)
    digitalWrite(rightMotorPin2, LOW); //opens switch on H-Bridge (breaks the circuit)
  }

  if (leftMotorSpeed == Direction) //left motor speed conditional statments
  {
    digitalWrite(leftMotorPin1, LOW); //opens switch on H-Bridge (breaks the circuit)
    digitalWrite(leftMotorPin2, HIGH);  //closes switch on H-Bridge (makes the circuit)
  }
  else if (leftMotorSpeed == -Direction)
  {
    digitalWrite(leftMotorPin1, HIGH); //closes switch on H-Bridge (makes the circuit)
    digitalWrite(leftMotorPin2, LOW); //opens switch on H-Bridge (breaks the circuit)
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW); //opens switch on H-Bridge (breaks the circuit)
    digitalWrite(leftMotorPin2, LOW); //opens switch on H-Bridge (breaks the circuit)
  }
  analogWrite(enableRightMotor, MOTOR_SPEED); //assigns PWM to right motor
  analogWrite(enableLeftMotor, MOTOR_SPEED);    // assigns PWM to left motor
    
}
