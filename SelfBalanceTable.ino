#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu(Wire);

double calib=0;

#define loopTimer 10

#define BT Serial

#define Kp 20 //20
#define Ki 1.20625  //0.8
#define Kd 25 //25

#define ML1 3
#define ML2 4
#define EnL 9

#define MR1 5
#define MR2 6
#define EnR 10


char dir='s';

double setpoint=0;
double curr_error=0, last_error=0, sum_error=0;


void setup() 
{
  pinMode(ML1,OUTPUT);
  pinMode(ML2,OUTPUT);
  pinMode(MR1,OUTPUT);
  pinMode(MR2,OUTPUT);
  pinMode(EnL,OUTPUT);
  pinMode(EnR,OUTPUT);

  //BT.begin(115200);
  
  Wire.begin();
  mpu.begin();
  //mpu.setGyroOffsets(-2.74, 1.55, 0.08);
  mpu.setGyroOffsets(-2.71, 1.51, 0.13);  //Old

  //calib=3.45;  
  calib=3.65; //Old
  
}

void loop() 
{
  long timer=millis();
  
  switch(dir)
  {
    case 'f': setpoint=5;  break;
    case 'b': setpoint=-5;  break;
    case 'l': setpoint=0;  break;
    case 'r': setpoint=0;  break;
    case 's': setpoint=0;  break;
  }

  curr_error=setpoint-sensorAngle();
  double change_error=curr_error-last_error;
  sum_error+=curr_error;
  last_error=curr_error;

  double output = Kp*curr_error + Ki*sum_error + Kd*change_error;

  if(output>250)
    output=250;
  else if(output<-250)
    output=-250;
  
  for_back(-output);

  //Serial.println(curr_error);
  
  while((millis()-timer)<loopTimer);
}

double sensorAngle()
{
  mpu.update();
  double k=mpu.getAngleX() - calib;
  //Serial.println(k);
  return k;
}
void for_back(int v)
{
  if(v>0)  //forward
  {
    digitalWrite(ML1,HIGH);
    digitalWrite(ML2,LOW);
    digitalWrite(MR1,HIGH);
    digitalWrite(MR2,LOW);
    analogWrite(EnL,v);
    analogWrite(EnR,v);
  }
  else if(v==0)
  {
    digitalWrite(ML1,LOW);
    digitalWrite(ML2,LOW);
    digitalWrite(MR1,LOW);
    digitalWrite(MR2,LOW);
    analogWrite(EnL,0);
    analogWrite(EnR,0);    
  }
  else  //backward
  {
    digitalWrite(ML1,LOW);
    digitalWrite(ML2,HIGH);
    digitalWrite(MR1,LOW);
    digitalWrite(MR2,HIGH);
    analogWrite(EnL,-v);
    analogWrite(EnR,-v);
  }
}
