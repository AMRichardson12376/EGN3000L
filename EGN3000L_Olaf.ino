#define IR_SENSOR_RIGHT 11 //assigns right IR sensor to pin 11
#define IR_SENSOR_LEFT 12 //assigns left IR sensor to pin 12
#define MOTOR_SPEED 200 //assigns speed of the robot, please select a value from 180 to 255
#define Direction 1 //variable used in conditional statements
#include <Servo.h> // includes servo library functions
 

//Right motor
int enableRightMotor = 6; //assigns PWM value to right motor
int rightMotorPin1 = 7; //assigns H-bridge switch to arduino board
int rightMotorPin2 = 8; //assigns H-bridge switch to arduino board

//Left motor
int enableLeftMotor = 5; //assigns PWM value to left motor
int leftMotorPin1 = 9; //assigns H-bridge switch to arduino board
int leftMotorPin2 = 10; //assigns H-bridge switch to arduino board

class Sweeper //allows for use of sweeper functions, similar to a servo library
{
  Servo servo; // declares the servo
  int pos; // current servo position placeholder 
  int servoPositionMin; //variable stores servo's minimum position 
  int servoPositionMax; //variable stores servo's maximum position
  int increment; // increment to move for each interval in b/t degrees
  int  updateInterval;// interval between updates
  unsigned long lastUpdate; // last update of position of servo, does not store negative variable values
     
public: //by declairing public, the coding instructions can be seen from anywhere in the code
  Sweeper(int interval, int posMin, int posMax) //Sweeper is a function of the servo's interval and min/max positions to control rotation
  {
    updateInterval = interval; //updates interval after every servo iteration
  servoPositionMin = posMin;  //assigns minimum servo position to minimum position value assigned in setup()
    servoPositionMax = posMax; //assigned maximum servo position value assigned in setup()
    pos = servoPositionMin;
    increment = 1; //increments the servo to move in 1 degree steps
  }  
  void Attach(int pin)
  {
    servo.attach(pin);  //assigns the servo to the pin on the arduino board
  }
  void Detach()
  {
    servo.detach();  //removed servo from assigned pin number
  }   
  void Update()
  {
    if((millis() - lastUpdate) > updateInterval)  // updates step of servo by 1 degree from 0 to 180
    {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      Serial.println(servoPositionMin); //displays servo min and max position on serial monitor
      Serial.println(servoPositionMax);
      if ((pos >= 180) || (pos <= 0)) // end of sweep
      {
        // reverse direction
        increment = -increment;
      }
    }
  }
};

Sweeper sweeper1(15, 45, 55);  //initializes servo position

void setup()
{
    Serial.begin(9600); //opens serial monitor
  sweeper1.Attach(3);  //assigns sweeper funtion to pin number 3
  
  // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
   TCCR0B = TCCR0B & B11111000 | B00000010 ; //allows for controlled speed to change PWM by user while setting appropirate PWM frequency for motor rotation
 
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
   sweeper1.Update(); //updates the servo position in the sweeper code
   
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

}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) //rotateMotor is a function of right and left motor speed
{

  if (rightMotorSpeed == Direction) //if right motor speed = 1
  {
    digitalWrite(rightMotorPin1, LOW); //opens switch on H-Bridge (breaks the circuit)
    digitalWrite(rightMotorPin2, HIGH); //closes switch on H-Bridge (makes the circuit)
     {


  }
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
