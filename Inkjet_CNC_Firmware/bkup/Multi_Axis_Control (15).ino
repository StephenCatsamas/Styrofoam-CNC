//idea, have mode where you can manually position the arm to determine coordiates and measure things.

/*coordiante system works like this, the origin is the top of the Z carrage and the back of the R carrage, ie fully retracted
front view
Z axis
<----------------------o
                       |
                       |
                       |
                       |
                       |
                       |
                       |
                       V R axis 
*/

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

const uint16_t front[] = {1350, 22100};            // Right most encoder boundary //track length on R is 240 mm
const uint8_t back[] = {0, 0};                   // Right most encoder boundary
const float uconv[] = {5.625,62.5};              // converstion between pos and millimeters units: pos / mm
int32_t PID[] = {0,0};
const float coP[] = {0.1,0.5};
const float coI[] = {0,0.0};
const float coD[] = {-2,0.00};
float Perr[] = {0,0};
float Ierr[] = {0,0};
float Derr[] = {0,0};
const int16_t off[] = {0,0};
const int8_t len = 30;                             //sets the average period for which the derivative occurs
volatile int32_t time[len];
volatile int32_t errFncR[len];
volatile int32_t errFncZ[len];
volatile uint32_t timetgt[len];
int32_t point_0[] = {0,0};
int32_t point_1[] = {0,0};
int8_t surface;

uint8_t spd[] = {220,220};                    // Carriage speed from 0-255
int8_t i;
int32_t loopvar[] = {0,0};

volatile int16_t tgt[] = {0, 0};                    // Taget position for carriage
uint8_t count = 0;
int16_t v;



void setup() {
  Serial.begin(2000000);
  
  zeroAxis(R);
  //zeroAxis(Z);
  tgt[R] = 15;
  pinMode(encPhaseA[R], INPUT_PULLUP);
  pinMode(encPhaseA[Z], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encPhaseA[R]), encoderTriggerR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encPhaseA[Z]), encoderTriggerZ, CHANGE); 
}

void loop() {
    
  //tgt[R] = (tgt[R] + 29) % 1340;
  //tgt[R] = random(60,1300);
  //tgt[Z] = random(2000,10000);
  //Serial.println("new tgt:");
  //Serial.println(tgt[R]);
  //Serial.println(tgt[Z]);
  //for(v=0;v<20000;v += 10){
      
    //  tgt[Z] = v + 100;
      //Serial.println(pos[Z]);
      //Serial.println("new tgt:");
        //Serial.println(tgt[Z]);
        //go_to_tgt(Z);
      //delay(300);
  //}
  //go_to_tgt(R);
  //go_to_tgt(Z);
  delay(50);
  cut_cylinder(30, 100, 3);
  //cut_sphere( 50,  1, 0, 12);

  
}

       //motor[axis]->setSpeed(spd[axis]);
       //motor[axis]->backward();
       //motor[axis]->forward();
       //motor[axis]->stop();

void all_to_tgt(int16_t Rtgt, uint16_t Ztgt){ //use this method for automatic positioning
    Serial.println("all new tgt: (R,Z)");
    Serial.print (Rtgt);
    Serial.print (" ");
    Serial.println(Ztgt);
    
    if (Ztgt == tgt[Z]){
        tgt[R] = Rtgt;
        go_to_tgt(R);
    }else{ 
        tgt[R] =  Rtgt;
        tgt[Z] =  Ztgt;
        zeroAxis(R);
        go_to_tgt(Z);
        go_to_tgt(R);
    }
 //   delay(1000);
}       
       
void go_to_tgt(uint8_t axis) { //R axis positioning is only prescise on extension //only use this method for manual i-know-what-i-am-doing positioning.
  if (count == 20){
      
      zeroAxis(R);
      count = 0;
  }
  count++;
   
  for(i=0; i<len; ++i){
      errFncR[i] = tgt[axis] - pos[axis] +off[axis]; // maybe consider offset
  }
  for(i=0; i<len; ++i){
      errFncZ[i] = tgt[axis] - pos[axis] +off[axis];
  }
  Ierr[R] = 0;
  Ierr[Z] = 0;
  for(i=0; i<len; ++i){
      time[i] = millis();
  }
  
  reached = false;
  if (errFncR == 0 && axis == R){
      reached = true;
  }
  if (errFncZ == 0 && axis == Z){
      reached = true;
  }
  
  if (reached == false){
  if(tgt[axis] - pos[axis] < 40 && tgt[axis] - pos[axis] > -40 && axis == R){       //if too close it moves slightly backward
           motor[axis]->stop();
           motor[axis]->setSpeed(250); 
           motor[axis]->forward(); 
           delay(70); 
           motor[axis]->stop();
           Serial.println("Small Jump");
           delay(50);
   }
   
   timetgt[axis] = millis();
  }
  while (reached == false){
  if(axis == R){
      
      PID[axis] = (int32_t)doPID(axis);  
        
      if(PID[axis] < 0){
          if(220 - PID[axis] > 235){
              motor[axis]->setSpeed(235);
          }else{
              motor[axis]->setSpeed(220 - PID[axis]);
          }
          motor[axis]->forward();
          dir[axis] = 0;
          
      }
      
      if(PID[axis] > 0){
          if(220 + PID[axis] > 235){
              motor[axis]->setSpeed(235);
          }else{
              motor[axis]->setSpeed(220 + PID[axis]);
          }
          motor[axis]->backward();
          dir[axis] = 1;
          
      }
      
      if(tgt[axis]-pos[axis] < 20 && dir[axis] == 1 && millis() - timetgt[axis] > 100){
          if(axis == R && tgt[R] < 1000){
          motor[axis]->setSpeed(180);
          motor[axis]->backward();
          delay(1);
          }
      }
      if(tgt[axis]-pos[axis] > -20 && dir[axis] == 0 && millis() - timetgt[axis] > 100){
          if(axis == R && tgt[R] < 1000){
          motor[axis]->setSpeed(150);
          motor[axis]->forward();
          delay(1);
          }
      }
          
      
      if(pos[axis] - tgt[axis] <= 1 && pos[axis] - tgt[axis] >= -1){
          reached = true;
          motor[axis]->stop();
          delay(50);
      }
   }
   
   if(axis == Z){
      PID[axis] = (int32_t)doPID(axis);
      //Serial.print(PID[Z]);
      //Serial.print(" ");
      //Serial.println(pos[Z]);
      // if(tgt[axis]-pos[axis] < -30){
          // Serial.println("help!! below target");
          // Serial.println(pos[Z]);
          // motor[axis]->stop();
          // delay(300);
          // dir[axis]= 0;
      // }
      
      if(PID[axis] > 0){
          if(240 + PID[axis] > 255){
              motor[axis]->setSpeed(255);
          }else{
              motor[axis]->setSpeed(240 + PID[axis]);
          }
          motor[axis]->forward();
          dir[axis] = 1;
          
      }
      if(PID[axis] < 0){
          if(240 - PID[axis] > 255){
              motor[axis]->setSpeed(255);
          }else{
              motor[axis]->setSpeed(240 - PID[axis]);
          }
          motor[axis]->backward();
          dir[axis] = 0;
      }
      
     if(tgt[axis]-pos[axis] < 20 && dir[axis] == 1){
         if(axis == Z){
         motor[axis]->setSpeed(100);
         motor[axis]->forward();
         delay(1);
         }
     }
     if(tgt[axis]-pos[axis] > -20 && dir[axis] == 0){
         if(axis == Z){
         motor[axis]->setSpeed(100);
         motor[axis]->backward();
         delay(1);
         }
     }
     //code in case it goes below target and needs manual lifting
          
      
      if(pos[axis] - tgt[axis] <= 10 && pos[axis] - tgt[axis] >= 10){
          reached = true;
          
        //  motor[axis]->setSpeed(255);
         // motor[axis]->forward();
         // delay(10);
          motor[axis]->stop();
          delay(50);
      }
   }
      if(millis() - timetgt[axis] > 1500 && axis == R){ //if taking too long it get a jolt
          motor[axis]->stop();
          motor[axis]->setSpeed(255); 
          motor[axis]->forward(); 
          delay(100); 
          motor[axis]->stop();
          Serial.println("Jolt");
          delay(100);
          timetgt[axis] = millis();
          count++;
      }
    // if(millis() - timetgt[axis] > 300 && axis == Z){ //if taking too long it get a jolt
          // motor[axis]->setSpeed(0);
          // motor[axis]->forward();
          // Serial.print("Jolt: ");
          // Serial.print(tgt[axis]-pos[axis]);
          // Serial.print(" ");
          // Serial.println(PID[axis]);
          // delay(200);
          // timetgt[axis] = millis();
      // }      
  }
  
    
}

float doPID(uint8_t axis){
    float resultPID = 0;
   
   
   listShift(errFncR);
   listShift(errFncZ);
   listShift(time);

    errFncR[0] = tgt[R] - pos[R] +off[R];
    errFncZ[0] = tgt[Z] - pos[Z] +off[Z];
    time[0] = millis();

    if (axis == R){
    Perr[R] = errFncR[0];
    Ierr[R] = Ierr[R] + errFncR[0]*(time[0]-time[1]); // milli pos second
    Derr[R] = (float)(errFncR[0] - errFncR[len-1])/(time[0]-time[len-1]); // milli pos per second
    }
    if (axis == Z){
    Perr[Z] = errFncZ[0];
    //Ierr[Z] = Ierr[Z] + errFncZ[0]*(time[0]-time[1]); // milli pos second
    //Derr[Z] = (float)(errFncZ[0] - errFncZ[len-1])/(time[0]-time[len-1]); // milli pos per second
    Ierr[Z] = Ierr[Z] + (time[0]-time[1]);
    if (errFncZ > 0){
    return resultPID = Ierr[Z]*coI[Z] + (Perr[Z]-0)*coP[Z];
    } else{
        return resultPID = 0;
    }   
    }
    
   return resultPID = coP[axis]*Perr[axis] + coI[axis]*Ierr[axis] + coD[axis]*Derr[axis];
}

void zeroAxis(uint8_t axis){
  Serial.println("Zero Axis");
  motor[axis]->setSpeed(240); // an integer between 0 and 255
  motor[axis]->forward(); // attempt to bring back to start
  delay(1000); 
  motor[axis]->stop();  // an integer between 0 and 255
  delay(500); 
  pos[axis] = 0;
    
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
      pos[axis] = pos[axis] + 1; //- 2 * axis; // dodgy hack cuz 
    } else { 
      pos[axis] = pos[axis] - 1; //+ 2 * axis; // this means the z axis is inverted
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

void cut_cylinder(float radius, float height, float start_Z){      //in millimeters //start_Z is an offest downwards
    point_0[R] = (int32_t)(radius * uconv[R]);
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);
    
    for(loopvar[Z]=(int32_t)(start_Z* uconv[Z]); loopvar[Z]<point_0[Z]; loopvar[Z] += 50){
        for(loopvar[R]=100; loopvar[R]<point_0[R]; loopvar[R] += 5){
            all_to_tgt(loopvar[R],loopvar[Z]);
    }
    }
    
}

void cut_sphere(float radius, int8_t surface, float center_R, float center_Z){      //in millimeters // if surface is 1 then the inner surface is cut if -1 then the outer surface 
    point_1[R] = (int32_t)(center_R * uconv[R]);
    if (center_Z - radius > 0){
    point_0[Z] = (int32_t)((center_Z - radius) * uconv[Z]);
    }else{
        point_0[Z]=0;
    }
    point_1[Z] = (int32_t)((center_Z + radius) * uconv[Z]);
    
    for(loopvar[Z]=point_0[Z]; loopvar[Z]<point_1[Z]; loopvar[Z] += 50){
        point_0[R] = point_1[R] + surface*(int32_t)(sqrt((float)(radius*radius) + (float)(loopvar[Z]-center_Z*uconv[Z])*(loopvar[Z]-center_Z*uconv[Z])/(float)(uconv[Z]*uconv[Z]))* uconv[R]);
            for(loopvar[R]=100; loopvar[R]<point_0[R]; loopvar[R] += 5){
            all_to_tgt(loopvar[R],loopvar[Z]);
            }
    }
    
}

void cut_cone( float radius,  float gradient,  float height,  float start_Z){      //in millimeters //radius is the radius at the top and should be the smallest (largest pos value) // gradient is R(mm)/Z(mm)
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);
    
    for(loopvar[Z]=start_Z; loopvar[Z]<point_0[Z]; loopvar[Z] += 50){
        point_0[R] = (int32_t)((radius - (float)(gradient * loopvar[Z])/(float)uconv[Z])*uconv[R]);
        for(loopvar[R]=0; loopvar[R]<point_0[R]; loopvar[R] += 5){
            all_to_tgt(loopvar[R],loopvar[Z]);
    }
    }
    
}

void cut_layer(float radius, float height, float start_Z){      //in millimeters // for trimming layers off a blank. // only do a small cut each time
    point_0[R] = (int32_t)(radius * uconv[R]);
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);
    
        tgt[R] =  point_0[R];
    for(loopvar[Z]=(int32_t)(start_Z* uconv[Z]); loopvar[Z]<point_0[Z]; loopvar[Z] += 50){
        tgt[Z] =  loopvar[Z];
        go_to_tgt(R);
        go_to_tgt(Z);
   //     delay(1000);
    }
    
}