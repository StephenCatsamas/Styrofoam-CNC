//idea, have mode where you can manually position the arm to determine coordiates and measure things.
// add nice serial user interface, ei menu to select a cutting opperation and other thigns like that.
// the second motor controler can be used for the heater aswell
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

#define ENTh 6
#define IN1Th 52
#define IN2Th 53

#define ENQ 12
#define IN1Q 47
#define IN2Q 45




//custom cut program matrix
const uint8_t dpthlst[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 71, 71, 71, 71, 70, 69, 69, 69, 69, 69, 68, 67, 67, 67, 67, 67, 67, 67, 66, 65, 65, 65, 65, 65, 65, 65, 64, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 62, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 60, 57, 55, 47, 46, 37, 36, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 34, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 34, 35, 35, 35, 35, 35, 36, 37, 37, 37, 37, 37, 37, 37, 38, 39, 39, 39, 40, 41, 41, 41, 41, 41, 42, 43, 43, 43, 44, 45, 45, 45, 46, 47, 47, 48, 48, 49, 50, 50, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// indexes for encoder R and Z
const uint8_t R = 0;
const uint8_t Z = 1;
const uint8_t Th = 2;
const uint8_t Q = 3;

const uint8_t encPhaseA[] = {2, 3, 18};        // R-ARIS  encoder 1 on pins 2 and 4
const uint8_t encPhaseB[] = {4, 5, 17};


const uint32_t maxPos[] = {1251, 35001, 40000};            // Right most encoder boundary  // MEASURE THIS
const uint8_t minPos[] = {0, 0, 0};                   // Right most encoder boundary
const float uconv[] = {5.95,117};              // converstion between pos and millimeters units: pos / mm //MEASURE THIS  
float coP[] = {0.1,2,-0.0001};                     //PID coefficients for each axis adjust these to taste.
float coI[] = {0,0.01,-0.0001};
float coD[] = {-2,0.00,-0.001};
const float limI[] = {15,75,1000};
const int posTolerance[] = {2,5,100};

volatile int8_t encPhaseALast[] = {LOW, LOW, LOW}; 
bool reached[] ={false,false,false};                 // true if at tgt
bool go_dir = false;                           // if true all_to_tgt will not zero R first or wait for turn table // returns to false after all_to_tgt call
bool dir[] = {0,0,0};                            //motor direction, any semblance to actual movement in purely coincidental
int32_t pos[] = {0,0,0};                    //axis position
int32_t dist[] = {0,0,0};                    //axis total distance traveled, gets rest from time to time
int32_t tgt[] = {0, 0,0};                    // Taget position for carriage
int32_t tgt_man[] = {0, 0,0};               //a secondary tgt marker for manual positioning.
int32_t PID[] = {0,0,0};                     //resultant PID value after calling in one go_to_tgt
float Perr[] = {0,0,0};                      //PID calulation internal use only
float Ierr[] = {0,0,0};
float Derr[] = {0,0,0};                      
uint8_t count[] = {0,0,0};                   //counts how many to_tgts for occational zeroing

const int8_t len = 30;                             //sets the average period for which the derivative occurs
int32_t time[len];
int32_t errFnc[3][len];
uint32_t timetgt[len];
float speedErrFnc[len] = {0.0};
float speedCo[] = {1000,1000,100};
int16_t speedPID;
float desiredSpeed = 0.5;
int32_t timeElapsed;

int32_t point_0[] = {0,0,0};             //for use in random functions
int32_t point_1[] = {0,0,0};
int32_t loopvar[] = {0,0,0};
int32_t v;
int8_t i;

L298N motorR(ENR, IN1R, IN2R);  //create motors on R and Z axis
L298N motorZ(ENZ, IN1Z, IN2Z);
L298N motorTh(ENTh, IN1Th, IN2Th);
L298N motorQ(ENQ, IN1Q, IN2Q);

//L298N motorD(ENQ, IN1Q, IN2Q);


void setup() {
    Serial.begin(2000000);

    motorQ.setSpeed(0);
    motorQ.forward();
    motorQ.stop();
    motorQ.setSpeed(150);
    motorQ.forward();
    delay(1000);
    
    motorTh.setSpeed(0);
    motorTh.forward();
    motorTh.stop();
    
    motorR.setSpeed(0);
    motorR.forward();
    motorR.stop();
    
    motorZ.setSpeed(0);
    motorZ.forward();
    motorZ.stop();
    
    zeroAxis(R);
    zeroAxis(Z);

    // Serial.println("set speed 200 and run");
    // motorTh.setSpeed(255);
    // motorTh.forward();
    
    // Serial.println("delay 10s");
    // delay(3000);
    
    // Serial.println("Motorc.stop");
    // motorTh.stop();
    // delay(100);

    
    pos[Th] = 0;
    tgt[Th] = 25000;
    

    pinMode(encPhaseA[R], INPUT_PULLUP);
    pinMode(encPhaseA[Z], INPUT_PULLUP);
    pinMode(encPhaseA[Th], INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(encPhaseA[R]), encoderTriggerR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Z]), encoderTriggerZ, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Th]), encoderTriggerTh, CHANGE);
}

void loop() {
    

  //  go_to_tgt(Th);




    cut_cylinder(30, 100, 50);
    //pos_reader();
   // manual_ctrl();
}


void manual_ctrl(){ //for manual control of motors
    int8_t inread =0;
    
    if (Serial.available() > 0){
        inread = Serial.read();
        
    if(inread == 113){ //W for Z up
            speedCo[0] = 2*speedCo[0];
            Serial.println(speedCo[0]);
    }
    if(inread == 97){ //W for Z up
            coP[Th] = 0.5*coP[Th];
            Serial.println(coP[Th]);
    }
    if(inread == 119){ //W for Z up
            coI[Th] = 2*coI[Th];
            Serial.println(coI[Th]);
    }
    if(inread == 115){ //W for Z up
            coI[Th] = 0.5*coI[Th];
            Serial.println(coI[Th]);
    }
    if(inread == 101){ //W for Z up
            coD[Th] = 2*coD[Th];
            Serial.println(coD[Th]);
    }
    if(inread == 100){ //W for Z up
            coD[Th] = 0.5*coD[Th];
            Serial.println(coD[Th]);
    }
    /*    if(inread == 87){ //W for Z up
            tgt_man[Z] = tgt_man[Z] - 100;
           // go_to_tgt(Z);
        }else if (inread == 83){ //S for Z down
            tgt_man[Z] = tgt_man[Z] + 100;
           // go_to_tgt(Z);
        }else if (inread == 65){ //A for R back
            tgt_man[R] = tgt_man[R] - 50;
           // go_to_tgt(R);
        }else if (inread == 68){ // D for R forward
            tgt_man[R] = tgt_man[R] + 50;
           // go_to_tgt(R);
        }
        go_dir = true;
        all_to_tgt(tgt_man[R],tgt_man[Z]);
        */
    }
}


//motor[axis]->setSpeed(0 to 255);
//motor[axis]->backward();
//motor[axis]->forward();
//motor[axis]->stop();





