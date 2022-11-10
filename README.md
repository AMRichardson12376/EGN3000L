# Group 7: Fro7en 
This respository contains code for the Olaf Line Following Robot


# Code Explanation:
Assign the signal output pins from the IR Sensors to pin numbers on the Arduino board. This will allow the Arduino board to process feedback information from the sensors. 
Define variables to store the pulse width modulated value (PWM) i.e. motor speed, left and right motor speed, as an integer. 
The user will enter an integer from 180 to 255 where the #define MOTOR_SPEED variable is defined. 
Define a direction variable that will store the numerical value 1. Include the servo library for use of servo functions, indicated by the #include <servo.h>. Define the initial position of the servo to be 0 degrees by creating an integer position variable, int pos 1. 
Using the servo library functions, assign the servo to pin ~3 on the Arduino board. 
For the right and left motors, assign switches on the H-bridge for both DC motors and assign the PWM output pin on the H-Bridge for both motors. 
Code in void setup() will only run once, enabling pins on the Arduino board as inputs or outputs as well as initializing the servo position. Code in void loop() runs the coding instructions over an infinite loop. 
Within the loop, there are also directions on how the servo will rotate, the rate at which the servo will rotate in degree steps, and the time it takes for the servo to change directions. 
The loop also processes the IR signals and assigns the Direction values (1 or -1 from the Direction variable) to a rotateMotor variable to change the motor direction in the rotateMotor function. 
For example, signals from 2 IR sensors will be processed and if both the IR sensors do not detect a black line, the output pin defined earlier will be low, 0 voltage will be supplied to the pins, and the rotate motor speed will be a function of the Direction variable 1 and 1, where one value is right motor speed and other is left. 
The code continues with more if-else if-else statements for each IR signal combination. 
The final code instruction is the declaration of the rotateMotor function which is a function of both right and left motor speed. 
Using the Direction values stored in right and left motor speed from the loop prior, conditional statements are used to control motor direction by turning on and off switches located on the H-bridge. 
For example, if the right motor speed equals the Direction value 1, then the switches assigned to the right motor will be opened or closed on the H-bridge. Here, pin 1 was assigned low. 
This means 0 volts are applied to the pin from the Arduino and the switch opens. When the pin is high, 5 volts are applied, and the switch closes, completing the circuit. 
The concept behind this is that the Arduino board is opening and closing switches on the H-Bridge so that the H-Bridge can change the voltage polarity from the power supply to the DC motors, causing a change in the DC motors’ direction. 
If-else if-else statements continue for the remainder of the code for both left and right motors. The final line of code assigns the user’s PWM value to the left and right motors, changing the speed of the motors. 
The concept behind this is, the PWM determines how long voltage will be supplied to the motors. 
If the PWM value is low, the voltage will be supplied to the motors in short time increments, causing the motors to rotate slowly. 
If the PWM assigned by the user is high i.e. 255, the pulses of supplied voltage will longer, allowing the motors to rotate to their maximum speed before the next pulse.  
