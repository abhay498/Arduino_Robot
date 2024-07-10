
#include <Servo.h> // Include Servo library
#include <HC_SR04.h> // Include HC-SR04 library

// Pin definitions
#define TRIG_PIN 3
#define ECHO_PIN 2
#define ECHO_INT 0

// Constants
const int RForward = 140; // Speed of the servo
const int RightLightSensor = 0; // Analog pin for the right light sensor
const int LeftLightSensor = 2; // Analog pin for the left light sensor
const int collisionThreshold = 10; // Threshold for obstacles (in cm)
const int threshold_pot = 4; // Analog pin for the threshold potentiometer
const int closeness = 60; // Value for detecting light source directly ahead
const int RightLEDIndicator = 5; // Digital pin for right LED indicator
const int LeftLEDIndicator = 4; // Digital pin for left LED indicator

// Variables
int SensorLeft, SensorRight, SensorDifference;
int leftDistance, rightDistance;

// Servo objects
Servo panMotor;
Servo leftMotor;
Servo rightMotor;

// Sensor object
HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);

void setup() {
  // Attach motors
  rightMotor.attach(11);
  leftMotor.attach(10);
  panMotor.attach(6);
  panMotor.write(90); // Center the micro servo

  sensor.begin();

  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LeftLightSensor, INPUT);
  pinMode(RightLightSensor, INPUT);
  pinMode(threshold_pot, INPUT);
  pinMode(RightLEDIndicator, OUTPUT);
  pinMode(LeftLEDIndicator, OUTPUT);

  Serial.begin(9600);
}

void loop() 
{
  int threshold = analogRead(threshold_pot);
  int distance_in_front_of_robot = ping(); // Get distance in front of the robot

  // Read sensor values
  SensorLeft = analogRead(LeftLightSensor);
  SensorRight = analogRead(RightLightSensor);
  SensorDifference = abs(SensorLeft - SensorRight);

  // Print sensor values
  Serial.print("Threshold: ");
  Serial.print(threshold);
  Serial.print(" (");
  Serial.print(threshold / 3);
  Serial.print("), Left photoresistor reading=");
  Serial.print(SensorLeft);
  Serial.print(", Right photoresistor reading=");
  Serial.print(SensorRight);
  Serial.print(", Sensor Difference="); 
  Serial.print(SensorDifference);
  Serial.print(", Obstacle: ");
  Serial.print(distance_in_front_of_robot);
  Serial.print(" cm, Move: ");

  if (SensorLeft >= threshold && SensorRight >= threshold && SensorDifference <= closeness)
	  {
		  stopRobot();
     }

  if (distance_in_front_of_robot > collisionThreshold) 
    {
      navigateTowardsLight(threshold);
    } 
  else 
    {
      avoidObstacle();
    }

  Serial.println();
}

void navigateTowardsLight(int threshold) 
    {
      if (SensorLeft <= threshold && SensorRight <= threshold && 
          SensorLeft >= threshold / 3 && SensorRight >= threshold / 3 && 
          SensorDifference <= closeness)
		  {
            moveForward();
          } 
      else if (SensorLeft >= threshold && SensorRight >= threshold && SensorDifference <= closeness) 
	      {
            stopRobot();
          } 
      else if (SensorLeft < SensorRight && SensorRight >= threshold / 3) 
	      {
            turnRight();
          }
	  else if (SensorLeft > SensorRight && SensorLeft >= threshold / 3)
		  {
            turnLeft();
          }
	  else if (SensorLeft < threshold / 3 && SensorRight < threshold / 3)
		  {
            turnAround();
          }
   }

void moveForward() 
{
  int Left_forward = 70;
  int Right_forward = 140;
  Serial.print("Forward");
  leftMotor.write(Left_forward);
  rightMotor.write(Right_forward);
  digitalWrite(LeftLEDIndicator, HIGH);
  digitalWrite(RightLEDIndicator, HIGH);
  delay(300);
}

void stopRobot() 
{
  int Left_neutral = 87;
  int Right_neutral = 120;
  Serial.println("STOP as Sensor difference is less than closeness");
  leftMotor.write(Left_neutral);
  rightMotor.write(Right_neutral);
  digitalWrite(RightLEDIndicator, LOW);
  digitalWrite(LeftLEDIndicator, LOW);
  delay(1000);
  while (1)
	{	
		Serial.println("STOP as Sensor difference is less than closeness");
		delay(500);
	}
}

void turnRight() 
{
  int Left_forward = 70;
  int Right_backward = 100;
  Serial.print("Right (turning)");
  leftMotor.write(Left_forward);
  rightMotor.write(Right_backward);
  digitalWrite(RightLEDIndicator, LOW);
  digitalWrite(LeftLEDIndicator, HIGH);
  delay(80);
}

void turnLeft() 
{
  int Left_backward = 100;
  int Right_forward = 140;
  Serial.print("Left (turning)");
  leftMotor.write(Left_backward);
  rightMotor.write(Right_forward);
  digitalWrite(RightLEDIndicator, HIGH);
  digitalWrite(LeftLEDIndicator, LOW);
  delay(80);
}

void turnAround() 
{
  int Left_backward_half = 95;
  int Right_forward_half = 130;
  Serial.print("LOST");
  leftMotor.write(Left_backward_half);
  rightMotor.write(Right_forward_half);
  delay(300);
  digitalWrite(RightLEDIndicator, LOW);
  digitalWrite(LeftLEDIndicator, LOW);
}

void avoidObstacle() 
{
  int Left_neutral = 87;
  int Right_neutral = 120;
  Serial.println("Blocked");
  leftMotor.write(Left_neutral);
  rightMotor.write(Right_neutral);
  panMotor.write(0);
  delay(400);
  
  leftDistance = ping();
  Serial.print("Left distance picked up from HC SR04 sensor: ");
  Serial.println(leftDistance);
  delay(400);
  panMotor.write(180);
  delay(600);
  
  rightDistance = ping();
  Serial.print(" and Right distance picked up from HC SR04 sensor: ");
  Serial.println(rightDistance);
  delay(400);
  panMotor.write(90);
  delay(400);
  compareDistance();
}

void compareDistance() 
{
  if (leftDistance > rightDistance) 
     {
		    move_left();
     } 
  else if (rightDistance > leftDistance) 
     {
        move_right();
     } 
  else 
     {
        turn_back();
     }
}

void move_left() 
{
  int Left_backward = 100;
  int Left_forward = 70;
  int Right_forward = 140;
  leftMotor.write(Left_backward);
  rightMotor.write(Right_forward);
  Serial.print("Move: Left");
  delay(400);
  leftMotor.write(Left_forward);
  rightMotor.write(Right_forward);
  delay(600);
}

void move_right() 
{
  int Left_forward = 70;
  int Right_backward = 100;
  int Right_forward = 140;
  leftMotor.write(Left_forward);
  rightMotor.write(Right_backward);
  Serial.print("Move: Right");
  delay(400);
  leftMotor.write(Left_forward);
  rightMotor.write(Right_forward);
  delay(600);
}

void turn_back() 
{
  int Left_backward = 100;
  int Right_forward = 140;
  leftMotor.write(Left_backward);
  rightMotor.write(Right_forward);
  Serial.print("Move: Back");
  delay(400);
}

long ping() {
  sensor.start();
  while (!sensor.isFinished()) continue;
  return sensor.getRange();
}
