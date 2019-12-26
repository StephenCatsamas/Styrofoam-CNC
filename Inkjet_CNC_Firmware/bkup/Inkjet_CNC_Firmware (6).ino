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


const uint32_t maxPos[] = {1251, 35001};            // Right most encoder boundary  // MEASURE THIS
const uint8_t minPos[] = {0, 0};                   // Right most encoder boundary
const float uconv[] = {5.95,117};              // converstion between pos and millimeters units: pos / mm //MEASURE THIS  
const float coP[] = {0.1,2};                     //PID coefficients for each axis adjust these to taste.
const float coI[] = {0,0.01};
const float coD[] = {-2,0.00};

volatile int8_t encPhaseALast[] = {LOW, LOW, LOW}; 
volatile bool reached[] ={false,false};                 // true if at tgt
volatile bool go_dir = false;                           // if true all_to_tgt will not zero R first or wait for turn table // returns to false after all_to_tgt call
volatile bool dir[] = {0,0};                            //motor direction, any semblance to actual movement in purely coincidental
volatile int32_t pos[] = {0,0};                    //axis position
volatile int32_t dist[] = {0,0};                    //axis total distance traveled, gets rest from time to time
volatile int32_t tgt[] = {0, 0};                    // Taget position for carriage
volatile int32_t tgt_man[] = {0, 0};               //a secondary tgt marker for manual positioning.
volatile int32_t PID[] = {0,0};                     //resultant PID value after calling in one go_to_tgt
volatile float Perr[] = {0,0};                      //PID calulation internal use only
volatile float Ierr[] = {0,0};
volatile float Derr[] = {0,0};                      
volatile uint8_t count[] = {0,0};                   //counts how many to_tgts for occational zeroing

const int8_t len = 30;                             //sets the average period for which the derivative occurs
volatile int32_t time[len];
volatile int32_t errFncR[len];
volatile int32_t errFncZ[len];
volatile int32_t errFncTh[len];
volatile uint32_t timetgt[len];

volatile int32_t point_0[] = {0,0};             //for use in random functions
volatile int32_t point_1[] = {0,0};
volatile int32_t loopvar[] = {0,0};
volatile int32_t v;
volatile int8_t i;

L298N motorR(ENR, IN1R, IN2R);  //create motors on R and Z axis
L298N motorZ(ENZ, IN1Z, IN2Z);
L298N motorTh(ENTh, IN1Th, IN2Th);

//L298N motorD(ENQ, IN1Q, IN2Q);


void setup() {
    Serial.begin(2000000);

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

    Serial.println("set speed 200 and run");
    motorTh.setSpeed(255);
    motorTh.forward();
    
    Serial.println("delay 10s");
    delay(3000);
    
    Serial.println("Motorc.stop");
    motorTh.stop();
    delay(100);

    
    pinMode(encPhaseA[R], INPUT_PULLUP);
    pinMode(encPhaseA[Z], INPUT_PULLUP);
    pinMode(encPhaseA[Th], INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(encPhaseA[R]), encoderTriggerR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Z]), encoderTriggerZ, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Th]), encoderTriggerTh, CHANGE);
}

void loop() {
    cut_cylinder(30, 100, 50);
    //pos_reader();
    //manual_ctrl();
    
}

//motor[axis]->setSpeed(0 to 255);
//motor[axis]->backward();
//motor[axis]->forward();
//motor[axis]->stop();




void zeroAxis(uint8_t axis){
    if (axis == R){ //as it says on the tin
        Serial.println("Zero R");
        motorR.setSpeed(255);
        motorR.forward();
        delay(50);
        motorR.setSpeed(230);
        motorR.forward();
        delay(1500);
        motorR.stop();
        delay(500);
        pos[R] = 0;
        dist[R] = 0;
        count[R] = 0;
        reached[R] = false;
    }
    if (axis == Z){ //as it says on the tin
        Serial.println("Zero Z");
        motorZ.setSpeed(250); 
        motorZ.backward();
        delay(4000);
        motorZ.stop(); 
        delay(500);
        pos[Z] = 0;
        dist[Z] = 0;
        count[Z] = 0;
        reached[Z] = false;
    }

}
