#include "ECE3.h"
////////////////////////////////////
//////Defining Pins for Wheels//////
////////////////////////////////////

//nslpl turns motor on and off depending on HIGH or LOW state
//direction pin adjusts whether wheel rotates one way or the other
//pwm adjusts the rate at which the wheel spins from 0 - 255

const int right_nslpl_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int left_nslpl_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int LED = 76;

////////////////////////////////////
/////////Globals Variables//////////
////////////////////////////////////

//IR sensor array
uint16_t sensorValues[8];

//For calibrating the sensors
int rawCombined;
int prevRawCombined;
int fused;
int prevFused;
int prevRawPD;
int error;
int rawSum;
int prevRawSum;

unsigned long time_before_moving;

//Default speed if there is no error. Different wheel speeds to calibrate car to run in straight line
int leftDefaultSpeed = 75;
int rightDefaultSpeed = 75;

//Keeps track of what the car is doing
int carPath;

void setup() {
    Serial.begin(9600);
    ECE3_Init();

    //Defining pins
    pinMode(left_nslpl_pin,OUTPUT);
    pinMode(left_dir_pin,OUTPUT);
    pinMode(left_pwm_pin,OUTPUT);
    pinMode(right_nslpl_pin,OUTPUT);
    pinMode(right_dir_pin,OUTPUT);
    pinMode(right_pwm_pin,OUTPUT);

    pinMode(LED, OUTPUT);

    //Car is on and travelling forward
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(left_nslpl_pin,HIGH);
    digitalWrite(right_dir_pin,LOW);
    digitalWrite(right_nslpl_pin,HIGH);

    digitalWrite(LED,LOW);
    
    //Car starts at "followPath()"
    carPath = 0;

    //Initial previous fused value
    prevFused = 0;
    prevRawPD = 0;

    resetEncoderCount_left();
    resetEncoderCount_right();

    delay(3000);
    time_before_moving = micros();
    ChangeBaseSpeed(0, leftDefaultSpeed);
}

void loop() {  

    int time = micros() - time_before_moving; // light up every 3000 ms
    int seconds = micros()/1000000;
    if(((seconds)%3) == 0){
      digitalWrite(LED,HIGH);
    } else {
      digitalWrite(LED,LOW);
    }
    
    sensorFusion();
    
    //carPath = 0: away from start, carPath = 2: return to start
    if(carPath == 0 || carPath == 2)
        followPath();

    //carPath = 1: do a doughnut
    if(carPath == 1)
        doughnut();
   
    //carPath = 3: finish
    if(carPath == 3)
        finish();
}

////////////////////////////////////
///////////Sensor Fusion////////////
////////////////////////////////////

void sensorFusion() {
    
  //Read raw sensor values
  ECE3_read_IR(sensorValues);

  //Normalize all sensors to be between (0-1000)
  rawSum = 0;
  for(int i = 0; i < 8; i++){
    rawSum += sensorValues[i];
  }
  int normal0 = ((sensorValues[0]-644)*1000)/1856;
  int normal1 = ((sensorValues[1]-574)*1000)/1921;
  int normal2 = ((sensorValues[2]-574)*1000)/1664;
  int normal3 = ((sensorValues[3]-620)*1000)/1662;
  int normal4 = ((sensorValues[4]-528.2)*1000)/1158.8;
  int normal5 = ((sensorValues[5]-414)*1000)/1700;
  int normal6 = ((sensorValues[6]-574)*1000)/1735.8;
  int normal7 = ((sensorValues[7]-621)*1000)/1879;

  //Produce one fused value for all sensors
  fused = ((-8*normal0)-(4*normal1)-(2*normal2)-(normal3)+(normal4)+(2*normal5)+(4*normal6)+(8*normal7));  
}

////////////////////////////////////
///////////Follow Path//////////////
////////////////////////////////////

void followPath() {  
    
    int leftSpeed;
    int rightSpeed;
    int PDcontrollerRaw;
    int PDcontrollerConstrain;
    int steer;

    //Arbitrary constants tuned for this system; Kp = 2 & Kd = 36 seem to work well.
    const int Kp = 2;
    const int Kd = 14;
    //Error is current fused value minus previous fused value
    error = fused - prevFused;

    //PD Controller
    PDcontrollerRaw = (Kp*fused)+(Kd*error);    
//    Serial.println(PDcontroller);
    PDcontrollerConstrain = constrain(PDcontrollerRaw,-10000,10000);

    //Translate PD control value into readable value to steer the car
    //map proportionally translates from a fraction of over 10,000 to 100. It maps from a large range to smaller range but same proportion
    steer = map(PDcontrollerConstrain,-10000,10000,-100,100);

    //For debugging purposes
    //Serial.println(speed);
    int speed = leftDefaultSpeed;
    if(average() < 250){
      speed = 40;
    }
    //Define left and right speed based on default speed and steer value
    leftSpeed = (speed - steer);
    rightSpeed = (speed + steer);


    leftSpeed = constrain(leftSpeed,0,255);
    rightSpeed = constrain(rightSpeed,0,255);
    //Drive
    analogWrite(left_pwm_pin,leftSpeed);
    analogWrite(right_pwm_pin,rightSpeed);

    //Use this to troubleshoot and see if the IR sensors are seeing the black line
    
//    Serial.println(fused);
    
    //Serial.println(PDcontrollerRaw);

    //If the car hits a black line, engage "doughnut" or "finish"
    if(rawSum > 18000 && prevRawSum > 18000)
        carPath++;

    //Set current fused value as "prev" for next iteration
    prevFused = fused;
    prevRawSum = rawSum;
}

////////////////////////////////////
//////////////Doughnut//////////////
////////////////////////////////////

void doughnut() {  
    
    //Change direction of left wheel
    /*
    digitalWrite(left_dir_pin,HIGH);

    //Donut for 225 ms
    analogWrite(left_pwm_pin,255);
    analogWrite(right_pwm_pin,255);
    delay(250);

    //Change direction of left wheel and drive forward for 75 ms as a transitional buffer
    digitalWrite(left_dir_pin,LOW);
    analogWrite(left_pwm_pin,leftDefaultSpeed);
    analogWrite(right_pwm_pin,rightDefaultSpeed);
    delay(300);
    */
      //car turn, adjust delay timer

    resetEncoderCount_left();
    resetEncoderCount_right();
    digitalWrite(left_dir_pin,HIGH);
    analogWrite(left_pwm_pin,leftDefaultSpeed);
    analogWrite(right_pwm_pin,rightDefaultSpeed);
    ChangeBaseSpeed(leftDefaultSpeed, 70);
    while(average() < 300) ;
    ChangeBaseSpeed(70, 0);
  
    //go past black bar
    digitalWrite(left_dir_pin,LOW);
    ChangeBaseSpeed(0, leftDefaultSpeed);
    delay(200);

    //Return to "followPath"
    carPath++;
}

////////////////////////////////////
////////////////Finish//////////////
////////////////////////////////////

void finish() {  
    ChangeBaseSpeed(leftDefaultSpeed, 0);
    //Stop car
    digitalWrite(left_nslpl_pin,LOW);
    digitalWrite(right_nslpl_pin,LOW);
}

//used for encoders
int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}

void  ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
/*  
 *   This functin changes the car base speed gradually (in about 300 ms) from
 *   initialspeed to final speed. This non-instantaneous speed change reduces the 
 *   load on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int numSteps = 5;
  int pwmLeftVal = initialBaseSpd; // initialize left wheel speed 
  int pwmRightVal = initialBaseSpd;  // initialize right wheel speed 
  int deltaLeft = (finalBaseSpd-initialBaseSpd)/numSteps; // left in(de)crement
  int deltaRight = (finalBaseSpd-initialBaseSpd)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(60);   
  } // end for int k
} // end void  ChangeBaseSpeed
