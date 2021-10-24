#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library
#include <Wire.h>
#include <SharpIR.h>
 int turn_speed = 250;    // MAX 255 
  int turn_delay = 2;
//Turning PWM Values
//Straight
int vSpeedr = 140, vSpeedl = 135;
//Turning
int vTurnf = 50, vTurns=10;
//our L298N control pins
const int LeftMotorForward = 6;
const int LeftMotorBackward = 7;
const int RightMotorForward = 4;
const int RightMotorBackward = 5;
#define Rightmotorspeed 3 //controls speed of right wheel
#define Leftmotorspeed 11 //controls speed of left wheel

//Sensor Connection
  const int left_sensor_pin = 8;//for black tape we get 1 and for surface we get 0
  const int right_sensor_pin = 2;

  
  
  int left_sensor_state = 0;
  int right_sensor_state = 0;
//Wire functions
byte x = 0x20;
//sensor pins
#define trig_pin A0
#define echo_pin A1

#define maximum_distance 200
//Sensor pins - Sharp
#define SharpPinL A2
#define SharpPinR A3
#define model 1080
boolean goesForward = false;
int cm = 100;
int cmL = 100;
int cmR = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
//Creating Sharp
SharpIR SharpL = SharpIR(SharpPinL, model);
SharpIR SharpR = SharpIR(SharpPinR, model);


void setup(){
  Serial.begin(115200);
  Wire.begin(0x08);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(Leftmotorspeed, OUTPUT);
  pinMode(Rightmotorspeed, OUTPUT);
  Wire.onReceive(receiveEvent);
  analogWrite(Leftmotorspeed, vSpeedl);
  analogWrite(Rightmotorspeed, vSpeedr);
  pinMode(left_sensor_pin,INPUT);
  pinMode(right_sensor_pin,INPUT);
  
 delay(2000);
  readPing();
  delay(100);
  readPing();
  delay(100);
  readPing();
  delay(100);
  readPing();
  delay(100);
}

void loop(){
 if(x==0x20||0x10)
 obstacleAvoidance();
 if(x==0x30)
  lineTransition();
}
void lineTransition(){
  Serial.println("line Transition");
  
  if (cm <= 20||cmL<=10||cmR<=10){
    Serial.print("obstacle detected");
    moveStop();
    delay(300);
    moveBackward();
    delay(1000);
    moveStop();
    delay(300);
    readPing();

    if (cm >= cmL){
      turnRight();
      delay(300);
      moveStop();
    }
    else{
      turnLeft();
      delay(300);
      moveStop();
    }
  }
  else{
    moveForward(); 
  }
    readPing();
  
  if(digitalRead(left_sensor_pin) == 1 || digitalRead(right_sensor_pin) ==1){
    moveStop();
    Serial.println("line detected");
    delay(500);
    while((digitalRead(right_sensor_pin) == 0 && digitalRead(left_sensor_pin) == 0)||(digitalRead(right_sensor_pin) == 1 && digitalRead(left_sensor_pin) == 1)){
          turnLeft();
          delay(100);
          Serial.println("aligning");
          }
          while(x==0x40){
left_sensor_state = digitalRead(left_sensor_pin);
right_sensor_state = digitalRead(right_sensor_pin);
Serial.print(left_sensor_state);
Serial.print(" ");
Serial.println(right_sensor_state);

if(right_sensor_state == 1 && left_sensor_state == 0)
{
  Serial.println("turning right");

  digitalWrite(LeftMotorForward,HIGH);
  digitalWrite(LeftMotorBackward,LOW);                       
  digitalWrite(RightMotorForward,LOW);
  digitalWrite(RightMotorBackward,LOW);

  analogWrite(Leftmotorspeed, turn_speed/2);
  analogWrite(Rightmotorspeed, turn_speed/4);
  delay(turn_delay);
  }
 if(right_sensor_state == 0 && left_sensor_state == 1)
{
  Serial.println("turning left");
  
  digitalWrite(LeftMotorForward,LOW);
  digitalWrite(LeftMotorBackward,LOW);                       
  digitalWrite(RightMotorForward,HIGH);
  digitalWrite(RightMotorBackward,LOW);

  analogWrite(Leftmotorspeed, turn_speed/4);
  analogWrite(Rightmotorspeed, turn_speed/2);

  delay(turn_delay);
  }

 if(right_sensor_state == 0 && left_sensor_state == 0)
{
  Serial.println("going forward");

  digitalWrite(LeftMotorForward,HIGH);
  digitalWrite(LeftMotorBackward,LOW);                       
  digitalWrite(RightMotorForward,HIGH);
  digitalWrite(RightMotorBackward,LOW);

  analogWrite(Leftmotorspeed, vSpeedl);
  analogWrite(Rightmotorspeed, vSpeedr);

  //delay(turn_delay);
  
}

 if(right_sensor_state == 1 && left_sensor_state == 1)
{ 
  Serial.println("stop");
  digitalWrite(LeftMotorForward,LOW);
  digitalWrite(LeftMotorBackward,LOW);                       
  digitalWrite(RightMotorForward,LOW);
  digitalWrite(RightMotorBackward,LOW);
  analogWrite(Leftmotorspeed, 0);
  analogWrite(Rightmotorspeed, 0);
  //delay(turn_delay);

}
  }
}
}
void obstacleAvoidance(){
  readPing();
  delay(50);
  if(x==0x20){
    Serial.println("normal mode");
    if (cm <= 20||cmL<=10||cmR<=10||(digitalRead(left_sensor_pin)==1||digitalRead(right_sensor_pin)==1)){
    Serial.print("obstacle detected");
    moveStop();
    delay(300);
    moveBackward();
    delay(1000);
    moveStop();
    delay(300);
    readPing();

    if (cm >= cmL){
      turnRight();
      delay(300);
      moveStop();
    }
    else{
      turnLeft();
      delay(300);
      moveStop();
    }
  }
  else{
    moveForward(); 
  }
    readPing();
}
else if(x==0x10){
moveStop();
Serial.println("off");
}
}
/*int lookRight(){  
  servo_motor.write(10);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90);
  return distance;
  delay(100);
}*/

void readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  cmL = SharpL.distance();
  if (cmL==0){
    cmL=250;
  }
  cmR = SharpR.distance();
  if(cmR==0){
    cmR=250;
  }
  /*Serial.print("cm = ");
  Serial.println(cm);
  Serial.print("cmL = ");
  Serial.println(cmL);
  Serial.print("cmR = ");
  Serial.println(cmR);*/
  }

void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(){

  if(!goesForward){

    goesForward=true;
    analogWrite(Leftmotorspeed, vSpeedl);
  analogWrite(Rightmotorspeed, vSpeedr);
    
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  }
}

void moveBackward(){

  goesForward=false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
}

void turnRight(){

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  delay(500);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
 
  
  
}

void turnLeft(){

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(500);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
void turnLeftTransition()
{
  analogWrite(Leftmotorspeed, 100);
  analogWrite(Rightmotorspeed, 25); 
    //run right motors clockwise
digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(100);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
void receiveEvent(int howmany)
{
  x = Wire.read();
  Serial.println(x,HEX);
  
}
