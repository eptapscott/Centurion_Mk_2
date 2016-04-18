

/*
 * Notes:
 * 1) Minimum pwm to have a single wheel free running = 70
 * 2) Turning on floor at 100 PWM needs delay of 930
 */

//#include <enes100.h>
//#include <enes100_marker.h>
//#include <enes100_rf_client.h>
#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include "enes100.h"

//#include <NewPing.h>

//Constants
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define MAX_DISTANCE 200
//Right
#define E1 3    //PWM
#define M1 2
//Left
#define E2 5    //PWM
#define M2 4

//Steer Factor
#define steerFactor 1.300

//Ultrasound Setup
Servo myservo;
//NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);

//RF Setup
SoftwareSerial sSerial(11,10);
enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

//Variables


  float x;
  float y;

  int cmS;

void advanceToPoint(int pwm, float xPos, float yPos){

}

void advance(int pwm){    //pwm determines speed (-255 -> 255)
  int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
  int dir = (pwm > 0) ? HIGH : LOW;
  analogWrite(E1,pwmR);
  digitalWrite(M1,dir);
  analogWrite(E2,pwm);
  digitalWrite(M2,dir);
}

void halt(void){
  analogWrite(E1,0);
  digitalWrite(M1,LOW);
  analogWrite(E2,0);
  digitalWrite(M2,LOW);
}

// Turning will be based on theta found from RF communication
void left(int pwm, float dest){  //counterclockwise
	float theta;
	int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
	if(rf.receiveMarker(&marker, 112)){   //Need to find actual number of marker
		theta = marker.theta;
		dest = theta + dest;
	  }
	while(theta > dest){
		delay(100);
		analogWrite(E1,pwmR);
		digitalWrite(M1,HIGH);
		analogWrite(E2,pwm);
		digitalWrite(M2,LOW);
    if(rf.receiveMarker(&marker, 112))
    {theta = marker.theta;}
  }
}

void right(int pwm){ //clockwise
  float theta;
  float dest;
  int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
  if(rf.receiveMarker(&marker, 112)){   //Need to find actual number of marker
    theta = marker.theta;
    dest = theta - (3.14/2.0);
  }
  while(theta < dest){
    delay(100);
    analogWrite(E1,pwmR);
    digitalWrite(M1,LOW);
    analogWrite(E2,pwm);
    digitalWrite(M2,HIGH);
    if(rf.receiveMarker(&marker, 112))
    {theta = marker.theta;}
  }
}

// Utility Functions

float toDegrees(float radians){
	return (radians < 0) ? radians + 6.2831853072 * (180 / 6.2831853072) : radians * (180 / 6.2831853072);
}

float toRadians(float degrees){
	return (degrees > 180) ? degrees - 360 * (6.2831853072 / 180) : degrees * (6.2831853072 / 180);
}

void setup() {
 //Initialize the software and hardware serial
 sSerial.begin(9600);
 Serial.begin(9600);
 //Initialize pins
 pinMode(2,OUTPUT);
 pinMode(3,OUTPUT);
 pinMode(4,OUTPUT);
 pinMode(5,OUTPUT);
 digitalWrite(M1,LOW);
 digitalWrite(M2,LOW);

 delay(500);

 //Reset the state of the server, and send a message indicating our status.
  rf.resetServer();
  rf.sendMessage("EarthBenders Connected.");
  Serial.print("EarthBenders Connected");
}

void loop() {
//	delay(1000);
//	advance(-85);
//	 while(cmS < 70){
//	//  cmS = sonar.ping_cm();
//	  Serial.print("Distance: ");
//	  Serial.println(cmS);
//	  delay(100);
//	 }
//	delay(1000);
//	halt();

	x = 5;
	y = 355;
	Serial.print(x);
	Serial.print(" ");
	Serial.println(y);
	x = toRadians(x);
	y = toRadians(y);
	Serial.print(x);
	Serial.print(" ");
	Serial.println(y);
	x = toDegrees(x);
	y = toDegrees(y);
	Serial.print(x);
	Serial.print(" ");
	Serial.println(y);
	x = 2;
	y = -2;
	x = toDegrees(x);
	y = toDegrees(y);
	Serial.print(x);
	Serial.print(" ");
	Serial.println(y);
	x = toRadians(x);
	y = toRadians(y);
	Serial.print(x);
	Serial.print(" ");
	Serial.println(y);

	delay(2000);
}
