/*
 * Notes:
 * 1) Minimum pwm to have a single wheel free running = 70
 * 2) Turning on floor at 100 PWM needs delay of 930
 */

#include <enes100.h>
#include <enes100_marker.h>
#include <enes100_rf_client.h>
#include <SoftwareSerial.h>
#include "enes100.h"
#include <Servo.h>
#include <NewPing.h>

//Constants
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define MAX_DISTANCE 200

//Ultrasound Setup
Servo myservo;
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);

//RF Setup
SoftwareSerial sSerial(21,20);
enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

//Variables
  //Right
  int E1 = 3;    //PWM
  int M1 = 2;    
  //Left
  int E2 = 5;    //PWM
  int M2 = 4;
  
  float x;
  float y;

  int cmS;
  
void advance(int pwm){    //pwm determines speed (0 -> 255)
  int pwmR = pwm + 24;    //Compensate for right biased oversteer
  analogWrite(E1,pwmR);
  digitalWrite(M1,HIGH);
  analogWrite(E2,pwm);
  digitalWrite(M2,HIGH);
}
void halt(void){
  analogWrite(E1,0);
  digitalWrite(M1,LOW);
  analogWrite(E2,0);
  digitalWrite(M2,LOW);
}
// Turning will be based on theta found from RF communication
void left(int pwm){  //counterclockwise 
  float theta;
  float dest;
  if(rf.receiveMarker(&marker, 5)){   //Need to find actual number of marker
    theta = marker.theta;
    dest = theta + (3.14/2.0);
  }
  while(theta > dest){
    analogWrite(E1,pwm);
    digitalWrite(M1,HIGH);
    analogWrite(E2,pwm);
    digitalWrite(M2,LOW);
    if(rf.receiveMarker(&marker, 5))
    {theta = marker.theta;}
  }
}
void right(int pwm){ //clockwise
  float theta;
  float dest;
  if(rf.receiveMarker(&marker, 5)){   //Need to find actual number of marker
    theta = marker.theta;
    dest = theta - (3.14/2.0);
  }
  while(theta < dest){
    delay(100);
    analogWrite(E1,pwm);
    digitalWrite(M1,LOW);
    analogWrite(E2,pwm);
    digitalWrite(M2,HIGH);
    if(rf.receiveMarker(&marker, 5))
    {theta = marker.theta;}
  }
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
  rf.sendMessage("Team One Connected.");
}

void loop() {
 advance(65);
 while(cmS < 70){
  cmS = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.println(cmS);
  delay(100);
 }
 halt();
}
