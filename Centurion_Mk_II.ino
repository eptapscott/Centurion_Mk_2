#include <enes100.h>
#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>
//#include "enes100.h"
#include <NewPing.h>

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

//Servo Setup
Servo myservo;

//Ultrasound Setup
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);

//RF Setup
SoftwareSerial sSerial(11,10);
enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

//Variables
  float x;
  float y;
  float theta;
  
  float mS; //sonar measurement in meters


// Utility Functions

float toDegrees(float radians){
  return (radians < 0) ? (radians + 6.2831853072) * (360 / 6.2831853072) : radians * (360 / 6.2831853072);
}

//float toRadians(float degrees){
//  return (degrees > 180) ? (degrees - 360) * (6.2831853072 / 360) : degrees * (6.2831853072 / 360);
//}

//Functions
void advance(int pwm){    //pwm determines speed (-255 -> 255)
  rf.sendMessage("Advancing.");
  int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
  int dir = (pwm > 0) ? HIGH : LOW;
  analogWrite(E1,pwmR);
  digitalWrite(M1,dir);
  analogWrite(E2,pwm);
  digitalWrite(M2,dir);

  //Update current position info
  if(rf.receiveMarker(&marker, 112)){   
    theta = toDegrees(marker.theta);
    x = marker.x;
    y = marker.y;
  }
}

void halt(void){
  analogWrite(E1,0);
  digitalWrite(M1,LOW);
  analogWrite(E2,0);
  digitalWrite(M2,LOW);

  //Update current position info
  if(rf.receiveMarker(&marker, 112)){   
    theta = toDegrees(marker.theta);
    x = marker.x;
    y = marker.y;
  }
}

// Turning will be based on theta found from RF communication
// Error: if theta is 15 degrees and we tell it to t
void counterClockwise(int pwm, float dest){  //left
  rf.sendMessage("Turn left.");
  int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
  if(rf.receiveMarker(&marker, 112)){   
    theta = toDegrees(marker.theta);
    }
  while( (theta > dest-5) && (theta < dest+5) ){
    delay(100);
    analogWrite(E1,pwmR);
    digitalWrite(M1,HIGH);
    analogWrite(E2,pwm);
    digitalWrite(M2,LOW);
    if(rf.receiveMarker(&marker, 112))
    {theta = toDegrees(marker.theta);}
  }
  //Stop
  halt();
  delay(100);
}

void clockwise(int pwm, float dest){ //right
  rf.sendMessage("Turning right.");
  int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
  if(rf.receiveMarker(&marker, 112)){   //Need to find actual number of marker
    theta = toDegrees(marker.theta);
  }
  while( !((theta > dest-5) && (theta < dest+5)) ){
    analogWrite(E1,pwmR);
    digitalWrite(M1,LOW);
    analogWrite(E2,pwm);
    digitalWrite(M2,HIGH);
    if(rf.receiveMarker(&marker, 112))
    {theta = toDegrees(marker.theta);}
  }
  //Stop
  halt();
  delay(100);
}

//Function that will move vehicle along x and then y axis in order to reach a predesignated position
void advanceToPoint(int pwm, float xPos, float yPos){
  int pwmR = pwm * steerFactor;    //Compensate for right biased oversteer
  int dir = (pwm > 0) ? HIGH : LOW;
  if(rf.receiveMarker(&marker, 112)){   
    y = marker.y;
    x = marker.x;
    theta = toDegrees(marker.theta);
  }
  while( !((x>xPos-0.1)&&(x<xPos+0.1)) ){ // x-axis movement
    if(theta < 180){ //NORTH
      if(x > xPos){//OSV east of xPos
        counterClockwise(100,180);
        while(x > xPos){//go
          advance(150);
        }
      }else{//OSV west of xPos
        clockwise(100,0);
        while(x < xPos){
          advance(150);
        }
      }
    }else{//(theta > 180) SOUTH
      if(x > xPos){//OSV east of xPos
        clockwise(100,180);
        while(x > xPos){
          advance(150);
        }
      }else{//OSV west of xPos
        counterClockwise(100,0);
        while(x < xPos){
          advance(150);
        }
      }
    }
  }
  halt();
  
  while( !((y>yPos-0.1)&&(y<yPos+0.1)) ){// y-axis movement
    //Turning
    if( (theta < 90) || (theta > 270) ){ //EAST
      if(y > yPos){//OSV north of yPos
        clockwise(100,270);
        while(y > yPos){//go
          advance(150);
        }
      }else{//OSV south of yPos
        counterClockwise(100,90);
        while(y < yPos){
          advance(150);
        }
      }
    }else{// WEST
      if(y > yPos){//OSV north of yPos
        counterClockwise(100,180);
        while(y > yPos){
          advance(150);
        }
      }else{//OSV south of xPos
        clockwise(100,90);
        while(y < yPos){
          advance(150);
        }
      }
    }
  } 
  halt(); 
}

//sonarMean: function will ping sonar numerous times and return average to eliminate outliers
// mS = sonar.ping_cm()/100;
float sonarMean(void){
  float M = 20;
  int i, outliers;
  float total = 0, mean, sigma = 0, sd, newtot = 0, newmean;
  float dataArr[20]; 

  for(i=0;i<M;i++){
    mS = sonar.ping_cm()/100;
    dataArr[i] = mS;
    total += mS;
  }
  mean = total/M;
  for(i=0;i<M;i++){
    sigma += pow(dataArr[i] - mean,2);
  }
  sd = pow(sigma/M,0.5);
  for(i=0;i<M;i++){
    if( (dataArr[i]<mean - sd)||(dataArr[i]>mean + sd) ){
      dataArr[i] = 0;
      outliers++;
    }
  }
  for(i=0;i<M;i++)
    newtot += dataArr[i];
  newmean = newtot/(M - outliers); 
  return newmean;
}

//sonarNav: function that will navigate past obstacles or perform the drive-by measurement
void sonarNav(int stage){
  float travDist,initY;
  switch(stage){
    case 1:
      //Obstacle avoidance
      if( !((x > 1.15) && (y < 0.05) && (theta < 95) && (theta > 85)) ){
        rf.sendMessage("Not in position for stage 1 obstacle avoidance\n");
      }else{
        rf.sendMessage("Beginning obstacle avoidance\n");
        while(travDist < 15){
          initY = marker.y;
          advance(90);
          while(sonarMean() > 0.10){//measuring a gap
            advance(90);
          }
          travDist = marker.y - initY;
        }
        halt();
        clockwise(100,0);
      }
      break;
    case 2:
      //Drive-by measurement
      
      break;
    default:
      rf.sendMessage("Invalid stage\n");  
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
  rf.sendMessage("EarthBenders Connected.\n");
  Serial.print("EarthBenders Connected\n");
}

void loop() {
//  delay(1000);
//  advance(-85);
//   while(cmS < 70){
//    cmS = sonar.ping_cm();
//    Serial.print("Distance: ");
//    Serial.println(cmS);
//    delay(100);
//   }
//  delay(1000);
//  halt();

  //In Transit: Variable object avoidance
  //Position South

  //Testing
  if(rf.receiveMarker(&marker, 112)){
    x = marker.x;
    y = marker.y;
    theta = toDegrees(marker.theta);
    Serial.print("x = ");
    Serial.println(x);
    Serial.print("y = ");
    Serial.println(y);
    Serial.print("theta = ");
    Serial.println(theta);
  }else{
    Serial.println("Not connected");
  }
  //Test
  clockwise(90,180);
  delay(2000);
  
}
