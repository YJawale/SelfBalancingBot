#include <MPU6050_tockn.h>
MPU6050 mpu(Wire);
const int enA=9;
const int enB=10;
const int in1=4;
const int in2=5;
const int in3=7;
const int in4=6;
float prevError=0;
float integral=0;
int county=0;
//best workign kp,ki,kd are 30,0.05,80
int kp= 20;
int ki= 0.8;
int kd=00;


void setup() 
{
   Serial.begin(9600);
  Wire.begin();
  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  mpu.begin();
  mpu.calcGyroOffsets(true);
}


float pidval( float setpoint, float currentAngle, float *prevError, float *integral) {
    float error = setpoint - currentAngle;
    *integral += error;
    float derivative = error - (*prevError);

    // Compute PID output
    float output = kp * error + ki*(*integral) + kd* derivative;
    *prevError = error;
    // Set motor speed based on PID output
    if(output<255){
        return output;
    }
    else{
      return 255;
    }
    
}

void pidcontrol(float speedA) //normal balance control code 
{
  if(speedA>0)
    {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    analogWrite(enA,speedA);
    analogWrite(enB,speedA);
    }
    else if (speedA<0)
    {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enA,-speedA);
    analogWrite(enB,-speedA);
    }
}

void loop() 
{
mpu.update();
float inpoot = mpu.getAngleY();
inpoot=inpoot+8.59;
float speedA=pidval(0, inpoot,&prevError,&integral);
pidcontrol(speedA);
}
