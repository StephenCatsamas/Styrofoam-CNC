#include <Wire.h>
#include <L298N.h>

#define ENR 9
#define IN1R 40
#define IN2R 41

#define ENZ 10
#define IN1Z 50
#define IN2Z 51

L298N motorA(ENR, IN1R, IN2R);  //create motors on R and Z axis
L298N motorB(ENZ, IN1Z, IN2Z);
L298N* motor[] = { &motorA, &motorB };

 
// indexes for encoder R and Z
const uint8_t R = 0;
const uint8_t Z = 1;



const uint8_t encPhaseA[] = {2, 3};        // R-ARIS  encoder 1 on pins 2 and 4
const uint8_t encPhaseB[] = {4, 5};
int8_t encPhaseALast[] = {LOW, LOW};
bool reached =false;
bool dir[] = {0,0};                     //0 is forward 1 is backward
volatile int16_t pos[] = {0, 0};

const uint16_t front[] = {1300, 22100};            // Right most encoder boundary
const uint8_t back[] = {0, 0};                   // Right most encoder boundary
int16_t PID[] = {0,0};
const float coP[] = {0.045,0.045};
const float coI[] = {0,0};
const float coD[] = {1.2,0.01};
float Perr[] = {0,0};
float Ierr[] = {0,0};
float Derr[] = {0,0};
const int8_t len = 30;                             //sets the average period for which the derivative occurs
volatile int32_t time[len];
volatile int32_t errFncR[len];
volatile int32_t errFncZ[len];

uint8_t spd[] = {220,220};                    // Carriage speed from 0-255
int8_t i;

volatile int16_t tgt[] = {0, 0};                    // Taget position for carriage
uint8_t count = 0;



void setup() {
  Serial.begin(115200);
  
  zeroAxis(R);
//  zeroAxis(Z);
  
  pinMode(encPhaseA[R], INPUT_PULLUP);
  pinMode(encPhaseA[Z], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encPhaseA[R]), encoderTriggerR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encPhaseA[Z]), encoderTriggerZ, CHANGE); 
}

void loop() {
    
  tgt[R] = random(100,1200);
  go_to_tgt(R);
  delay(500);
  
}

       //motor[axis]->setSpeed(spd[axis]);
       //motor[axis]->backward();
       //motor[axis]->forward();
       //motor[axis]->stop();

void go_to_tgt(uint8_t axis) {
  if ((count) == 20){
      zeroAxis[axis];
      count = 0;
  }
  count++;
  
  for(i=0; i<len; ++i){
      errFncR[i] = tgt[axis] - pos[axis]; // maybe consider offset
  }
  for(i=0; i<len; ++i){
      errFncZ[i] = tgt[axis] - pos[axis];
  }
  for(i=0; i<len; ++i){
      time[i] = millis();
  }
  reached = false;
  
  while (reached == false){
     
      PID[axis] = (int16_t)doPID(axis);
      Serial.print(PID[axis]);
      Serial.print(" ");
      Serial.print(Perr[axis]*coP[axis]);
      Serial.print(" ");
      Serial.print(Ierr[axis]*coI[axis]);
      Serial.print(" ");
      Serial.print(Derr[axis]*coD[axis]);
      Serial.print(" ");
      Serial.println(tgt[R] - pos[R]);
      
      if(PID[axis] < 0){
         //  if (dir[axis] == 0){
           //    motor[axis]->stop();
         //      delay(10);
         //  }
          if(215 - PID[axis] > 250){
              motor[axis]->setSpeed(250);
          }else{
              motor[axis]->setSpeed(215 - PID[axis]);
          }
          motor[axis]->forward();
          dir[axis] = 0;
          
      }
      
      if(PID[axis] > 0){
           //if (dir[axis] == 0){
           //    motor[axis]->stop();
          //    delay(10);
         //  }
          if(215 + PID[axis] > 250){
              motor[axis]->setSpeed(250);
          }else{
              motor[axis]->setSpeed(215 + PID[axis]);
          }
          motor[axis]->backward();
          dir[axis] = 1;
          
      }
      
      if(pos[axis] == tgt[axis]){
          reached = true;
          motor[axis]->stop();
          delay(10);
      }
      
  }
  delay(1);
  
    
}

float doPID(uint8_t axis){
    float resultPID = 0;
    
   listShift(errFncR);
   listShift(errFncZ);
   listShift(time);

    errFncR[0] = tgt[R] - pos[R];
    errFncZ[0] = tgt[Z] - pos[Z];
    time[0] = millis();

    if (axis == R){
    Perr[R] = errFncR[0];
    Ierr[R] = Ierr[R] + errFncR[0]*(time[0]-time[1]); // milli pos second
    Derr[R] = (float)(errFncR[0] - errFncR[len-1])/(time[0]-time[len-1]); // milli pos per second
    }
    if (axis == Z){
    Perr[Z] = errFncZ[0];
    Ierr[Z] = Ierr[Z] + errFncZ[0]*(time[0]-time[1]); // milli pos second
    Derr[Z] = (float)(errFncZ[0] - errFncZ[len-1])/(time[0]-time[len-1]); // milli pos per second
    }
    
   return resultPID = coP[axis]*Perr[axis] + coI[axis]*Ierr[axis] + coD[axis]*Derr[axis];
}

void zeroAxis(uint8_t axis){
  motor[axis]->setSpeed(250); // an integer between 0 and 255
  motor[axis]->forward(); // attempt to bring back to start
  delay(1000); 
  motor[axis]->stop(); // an integer between 0 and 255
  delay(500); 
    
}

void encoderTriggerR() {
  doEncoder(R);
}

void encoderTriggerZ() {
  doEncoder(Z);
}

void doEncoder(uint8_t axis) { 
  uint8_t n = digitalRead(encPhaseA[axis]);
  if ((encPhaseALast[axis] == LOW) && (n == HIGH)) {
    if (digitalRead(encPhaseB[axis]) == LOW) {
      pos[axis] = pos[axis] + 1 - 2 * axis; // dodgy hack cuz 
    } else { 
      pos[axis] = pos[axis] - 1 + 2 * axis; 
    }
  }
  encPhaseALast[axis] = n;
  
  cli();
}

 void listShift(uint32_t list[len]){
    for (i=0; i<len; ++i){
        list[(len-1)-i] = list[(len-2)-i];
    }
}