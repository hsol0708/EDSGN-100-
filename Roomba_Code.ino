/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Simplified Bump Switch Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) driving forward
 * until it hits an object (ie a bump switch is triggered) then it stops.
 *
 * How to run:
 * 1) Push left button on Launchpad to start the demo
 * 2) Robot will drive forward until the bump switches are triggered
 * 3) Once switches are triggered the robot will halt.
 * 4) Push left button again to start demo
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://fcooper.github.io/Robot-Library/
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * created by Franklin Cooper Jr.
 *
 * This example code is in the public domain.
 */

#include "SimpleRSLK.h"

/* Diameter of Romi wheels in inches */
float wheelDiameter = 2.7559055;

/* Number of encoder (rising) pulses every time the wheel turns completely */
int cntPerRevolution = 180;

/* How far in inches for the robot to travel */
int inchesToTravel = 6;

int wheelSpeed = 50; // Default raw pwm speed for both motors.
int wheelSpeed1 = 0; // Default raw pwm speed for right motor.
int wheelSpeed2 = 10; // Default raw pwm speed for left motor.

/* The distance the wheel turns per revolution is equal to the diamter * PI.
 *  The distance the wheel turns per encoder pulse is equal is equal to the above divided 
 *  by the number of pulses per revolution.
 */
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}

int motorSpeed = 50;

void setup() {
  Serial.begin(115200);
  delay(500);
	setupRSLK();
  setupWaitBtn(PUSH1);
}

void loop() {
    bool hitObstacle = false;

     /* Wait until button is pressed to start robot */
   Serial.println("Push left button on Launchpad to start demo");
   waitBtnPressed(PUSH1);

   delay(2000);

for(int i = 0; i<4; i++)
{
   
  /* Enable both motors, set their direction and provide a default speed */
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,motorSpeed);

  /* Keep checking if the robot has hit an object */
  while(!hitObstacle) {
    /* Loop through all bump switchees to see if it has been pressed */
    for(int x = 0;x<TOTAL_BP_SW;x++)
    {
      /* Check if bump switch was pressed */
      if(isBumpSwitchPressed(x) == true) {
        hitObstacle = true;
        break;
      }
    }
  }
	uint16_t total_count = 0; // Total amount of encoder pulses received

  /* Amount of encoder pulses needed to achieve distance */
  uint16_t x = countForDistance(wheelDiameter, cntPerRevolution, inchesToTravel);
  Serial.print("Expected count: ");
  Serial.println(x);

   /* Set the encoder pulses count back to zero */
   resetLeftEncoderCnt();
   resetRightEncoderCnt();

  /* Cause the robot to drive backward */
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);

  /* "Turn on" the motor */
  enableMotor(BOTH_MOTORS);

  /* Set motor speed */
  setMotorSpeed(BOTH_MOTORS, wheelSpeed);
  setMotorSpeed(RIGHT_MOTOR, wheelSpeed1);
  setMotorSpeed(LEFT_MOTOR, wheelSpeed2);

  /* Drive motor until it has received x pulses */
  while(total_count < x)
  {
    total_count = getEncoderLeftCnt();
    Serial.println(total_count);
  }
  
  Serial.println("Collision detected");
	disableMotor(BOTH_MOTORS);

  hitObstacle = false;
}
}
