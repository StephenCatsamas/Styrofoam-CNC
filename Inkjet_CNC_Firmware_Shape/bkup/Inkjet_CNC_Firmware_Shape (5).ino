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
const uint8_t dpthlst[] = {96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 95, 93, 92, 91, 89, 88, 87, 86, 84, 83, 82, 80, 79, 78, 77, 75, 74, 73, 71, 70, 69, 68, 66, 65, 64, 63, 61, 60, 59, 57, 56, 55, 54, 52, 51, 50, 48, 47, 46, 45, 44, 44, 43, 42, 42, 41, 41, 40, 40, 40, 40, 41, 42, 43, 44, 44, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 52, 53, 54, 54, 55, 55, 56, 56, 57, 57, 58, 58, 59, 59, 59, 60, 60, 60, 61, 61, 61, 62, 62, 62, 62, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 63, 63, 63, 63, 62, 62, 62, 62, 61, 61, 61, 60, 60, 60, 59, 59, 59, 58, 58, 57, 57, 56, 56, 55, 55, 54, 54, 53, 52, 52, 51, 51, 50, 49, 48, 48, 47, 46, 45, 44, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
float coP[] = {0.14,2,-0.0001};                     //PID coefficients for each axis adjust these to taste.
float coI[] = {0,0.01,-0.0001};
float coD[] = {-2,0.00,-0.001};
float covP[] = {0,0,-0.00055};                     //PID coefficients for each axis adjust these to taste.
float covI[] = {0,0,-0.00000015};
float covD[] = {0,0,0.003};
const float limI[] = {15,75,255};
const float limvI[] = {0,0,255};
const int posTol[] = {2,5,20};

volatile int8_t encPhaseALast[] = {LOW, LOW, LOW}; 
bool reached[] ={false,false,false};                 // true if at tgt
bool go_dir = false;                           // if true all_to_tgt will not zero R first or wait for turn table // returns to false after all_to_tgt call
bool dir[] = {0,0,0};                            //motor direction, any semblance to actual movement in purely coincidental
int32_t pos[] = {0,0,0};                    //axis position
int32_t dist[] = {0,0,0};                    //axis total distance traveled, gets rest from time to time
int32_t vel[] = {0,0,0};                    //axis velocity
int32_t tgt[] = {0, 0,0};                    // Taget position for carriage
int32_t tgtVel[] = {0,0,0};
int32_t tgt_man[] = {0, 0,0};               //a secondary tgt marker for manual positioning.
int32_t PID[] = {0,0,0};                     //resultant PID value after calling in one go_to_tgt
int32_t velPID[] = {0,0,0};
float Perr[] = {0,0,0};                      //PID calulation internal use only
float Ierr[] = {0,0,0};
float Derr[] = {0,0,0};  
float vPerr[] = {0,0,0};                      //PID calulation internal use only
float vIerr[] = {0,0,0};
float vDerr[] = {0,0,0};                     
uint8_t count[] = {0,0,0};                   //counts how many to_tgts for occational zeroing

const int8_t len = 30;                             //sets the average period for which the derivative occurs
int32_t time[len];
int32_t errFnc[3][len];
int32_t velErrFnc[3][len];
uint32_t timetgt[len];

int32_t timeElapsed;

int32_t point_0[] = {0,0,0};             //for use in random functions
int32_t point_1[] = {0,0,0};
int32_t loopvar[] = {0,0,0};
int32_t v;
int8_t i;
bool stall = false;

L298N motorR(ENR, IN1R, IN2R);  //create motors on R and Z axis
L298N motorZ(ENZ, IN1Z, IN2Z);
L298N motorTh(ENTh, IN1Th, IN2Th);
L298N motorQ(ENQ, IN1Q, IN2Q);


void setup() {
    Serial.begin(2000000);
//////////////////////////////don't touch this. Dodgy motor control requires this code to be run
    motorQ.setSpeed(0);
    motorQ.forward();
    motorQ.stop();
    motorQ.setSpeed(150); //try higher, also needs to change in other places
    //motorQ.setSpeed(0);
    motorQ.forward();
    
    motorTh.setSpeed(0);
    motorTh.forward();
    motorTh.stop();
    
    motorR.setSpeed(0);
    motorR.forward();
    motorR.stop();
    
    motorZ.setSpeed(0);
    motorZ.forward();
    motorZ.stop();
////////////////////////////////////////

    zeroAxis(R);
    zeroAxis(Z);
   
    Serial.println("Heating");
//    delay(10000);
   
    pinMode(encPhaseA[R], INPUT_PULLUP);
    pinMode(encPhaseA[Z], INPUT_PULLUP);
    pinMode(encPhaseA[Th], INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(encPhaseA[R]), encoderTriggerR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Z]), encoderTriggerZ, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Th]), encoderTriggerTh, CHANGE);
}

void loop() {
   
  //cut_pgrm(35);
  cut_cylinder(130, 80, 15, 120);
  //pos_reader();
   // manual_ctrl();
}


void manual_ctrl(){ //for manual control of motors
    int8_t inread =0;
    
    if(inread == 87){ //W for Z up
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
        delay(100);
        motorR.setSpeed(240);
        motorR.forward();
        delay(1000);
        motorR.stop();
        delay(500);
        pos[R] = 0;
        dist[R] = 0;
        count[R] = 0;
        reached[R] = false;
    }
    if (axis == Z){ //as it says on the tin
        Serial.println("Zero Z");
        motorZ.setSpeed(255); 
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


void pos_reader(){ // displays the current position // able to give position for when axis are moved physically or electronically
    if (dist[Z] > 120000){ //Z axis occational zero
        zeroAxis(R);
        zeroAxis(Z);
    }
    if (dist[R] > 20000){ // R axis occational zero
        zeroAxis(R);
    }
    
    Serial.print("R: ");
    Serial.print(pos[R]);
    Serial.print(" (");
    Serial.print(pos[R]/uconv[R]);
    Serial.print(" mm)");
    Serial.print("      Z: ");
    Serial.print(pos[Z]);
    Serial.print(" (");
    Serial.print(pos[Z]/uconv[Z]);
    Serial.println(" mm)");
    
    }


    
void cut_cylinder(float radius, float height, float start_Z, float start_R){      //in millimeters //start_Z is an offest downwards
    point_0[R] = (int32_t)(radius * uconv[R]);
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);

    for(loopvar[Z]=(int32_t)(start_Z* uconv[Z]); loopvar[Z]<point_0[Z]; loopvar[Z] += 900){
        for(loopvar[R]=(int32_t)(start_R * uconv[R]); loopvar[R]<point_0[R]; loopvar[R] += 50){
            all_to_tgt(loopvar[R],loopvar[Z]);
            
            Serial.print("R Progress %: ");
            Serial.println((100 * (loopvar[R]-100))/(point_0[R]-100));
        }
        
        Serial.print("Z Progress %: ");
        Serial.println((100 * (loopvar[Z]-(int32_t)(start_Z* uconv[Z])))/(point_0[Z]-(int32_t)(start_Z* uconv[Z])));
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

    for(loopvar[Z]=point_0[Z]; loopvar[Z]<point_1[Z]; loopvar[Z] += 100){
        point_0[R] = point_1[R] + surface*(int32_t)(sqrt((float)(radius*radius) + (float)(loopvar[Z]-center_Z*uconv[Z])*(loopvar[Z]-center_Z*uconv[Z])/(float)(uconv[Z]*uconv[Z]))* uconv[R]);
        for(loopvar[R]=100; loopvar[R]<point_0[R]; loopvar[R] += 20){
            all_to_tgt(loopvar[R],loopvar[Z]);
        }
    }
}

void cut_cone( float radius,  float gradient,  float height,  float start_Z){      //in millimeters //radius is the radius at the top and should be the smallest (largest pos value) // gradient is R(mm)/Z(mm)
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);

    for(loopvar[Z]=start_Z; loopvar[Z]<point_0[Z]; loopvar[Z] += 100){
        point_0[R] = (int32_t)((radius - (float)(gradient * loopvar[Z])/(float)uconv[Z])*uconv[R]);
        for(loopvar[R]=0; loopvar[R]<point_0[R]; loopvar[R] += 20){
            if(loopvar[Z] > point_0[Z]){
                loopvar[Z] = point_0[Z];
            }
            if(loopvar[R] > point_0[R]){
                loopvar[R] = point_0[R];
            }
            all_to_tgt(loopvar[R],loopvar[Z]);
        }
    }
}

void cut_layer(float radius, float height, float start_Z){      //in millimeters // for trimming layers off a blank. // only do a small cut each time
    point_0[R] = (int32_t)(radius * uconv[R]);
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);
    
    tgt[R] =  point_0[R];
    
    for(loopvar[Z]=(int32_t)(start_Z* uconv[Z]); loopvar[Z]<point_0[Z]; loopvar[Z] += 100){
        tgt[Z] =  loopvar[Z];
        go_to_tgt(R);
        go_to_tgt(Z);
    }
}


void cut_pgrm(float start_R){      //in millimeters // for trimming layers off a blank. // only do a small cut each time
     
    for(loopvar[Z]=0; loopvar[Z]<299; loopvar[Z] += 4){
        if (dpthlst[loopvar[Z]] != 0){
            point_0[Z] = loopvar[Z]*uconv[Z];
            point_0[R] = (int32_t)(dpthlst[loopvar[Z]]*uconv[R]);
            for(loopvar[R]=(int32_t)(start_R * uconv[R]); loopvar[R]<point_0[R]; loopvar[R] += 20){
                all_to_tgt(loopvar[R],point_0[Z]);
            }
        }
    }
}


void exeErr(String errMsg){
    //zeros axis but without resetting variables.
    motorR.setSpeed(240);
    motorR.forward();
    delay(1000);
    motorR.stop();
    delay(500);
    motorZ.setSpeed(255); 
    motorZ.backward();
    delay(4000);
    motorZ.stop(); 
    delay(500);
    
    Serial.println("");
    Serial.println("------Program Stop------");
    Serial.println("");
    Serial.println(errMsg);
    
    var_dump();
     
    while(true == true){
        delay(10000);
    }
}

//these next three are interupt functions for the pos
void encoderTriggerR() {
    doEncoder(R);
    dist[R] += 1; //cnc's need odometers too
}

void encoderTriggerZ() {
    doEncoder(Z);
    dist[Z] += 1;
}

void encoderTriggerTh() {
    doEncoder(Th);
    dist[Th] += 1;
}

void doEncoder(uint8_t axis) { //reads the encoders and updates the positions
    uint8_t curVal = digitalRead(encPhaseA[axis]);
    
    if ((encPhaseALast[axis] == LOW) && (curVal == HIGH)) {
        if (digitalRead(encPhaseB[axis]) == LOW) {
            pos[axis] = pos[axis] + 1; // - 2 * axis; // dodgy hack cuz
        } else {
            pos[axis] = pos[axis] - 1;// - 0.36 * axis; // due to rope slipping or something on the way up
        }
    }
    
    encPhaseALast[axis] = curVal;

    cli();
}

void listShift(uint32_t list[len]){ //nifty little function to shift the items in a list one down, deleting last and leaving first as was
    for (i=0; i<len; ++i){
        list[(len-1)-i] = list[(len-2)-i];
    }
}


void var_dump () {
    Serial.println("");
    Serial.println("------Variable Dump------");
    Serial.println("");
    
    Serial.print("reached [R,Z]: [");
    Serial.print(reached[R]);
    Serial.print(", ");
    Serial.print(reached[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("dir [R,Z]: [");
    Serial.print(dir[R]);
    Serial.print(", ");
    Serial.print(dir[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("pos [R,Z,Th]: [");
    Serial.print(pos[R]);
    Serial.print(", ");
    Serial.print(pos[Z]);
    Serial.print(", ");
    Serial.print(pos[Th]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("dist [R,Z,Th]: [");
    Serial.print(dist[R]);
    Serial.print(", ");
    Serial.print(dist[Z]);
    Serial.print(", ");
    Serial.print(dist[Th]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("tgt [R,Z,Th]: [");
    Serial.print(tgt[R]);
    Serial.print(", ");
    Serial.print(tgt[Z]);
    Serial.print(", ");
    Serial.print(tgt[Th]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("PID [R,Z]: [");
    Serial.print(PID[R]);
    Serial.print(", ");
    Serial.print(PID[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("Perr [R,Z]: [");
    Serial.print(Perr[R]);
    Serial.print(", ");
    Serial.print(Perr[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("Ierr [R,Z]: [");
    Serial.print(Ierr[R]);
    Serial.print(", ");
    Serial.print(Ierr[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("Derr [R,Z]: [");
    Serial.print(Derr[R]);
    Serial.print(", ");
    Serial.print(Derr[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("count [R,Z]: [");
    Serial.print(count[R]);
    Serial.print(", ");
    Serial.print(count[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("time []: [");
    Serial.print(time[0]);
     for (i=1; i<len; ++i){
        Serial.print(", ");
        Serial.print(time[i]);
    }
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("errFnc[R] []: [");
    Serial.print(errFnc[R][0]);
     for (i=1; i<len; ++i){
        Serial.print(", ");
        Serial.print(errFnc[R][i]);
    }
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("errFnc[Z] []: [");
    Serial.print(errFnc[Z][0]);
     for (i=1; i<len; ++i){
        Serial.print(", ");
        Serial.print(errFnc[Z][i]);
    }
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("timetgt []: [");
    Serial.print(timetgt[0]);
     for (i=1; i<len; ++i){
        Serial.print(", ");
        Serial.print(timetgt[i]);
    }
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("point_0 [R,Z]: [");
    Serial.print(point_0[R]);
    Serial.print(", ");
    Serial.print(point_0[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("loopvar [R,Z]: [");
    Serial.print(loopvar[R]);
    Serial.print(", ");
    Serial.print(loopvar[Z]);
    Serial.print("]");
    Serial.println("");
    Serial.println("");
}

void go_to_tgt(uint8_t axis) { //R axis positioning is only prescise on extension //only use this method for manual i-know-what-i-am-doing positioning.
    if((tgt[axis] > maxPos[axis]) || (tgt[axis] < minPos[axis])){
      if(axis != Th)  {
        exeErr("Unhandled Error: tgt out of range");
        }
    }
    
    if (count[Z] == 8 || dist[Z] > 120000 || (pos[Z] > 20000 && tgt[Z] - pos[Z] < 0)){ //Z axis occational zero
        zeroAxis(R);
        zeroAxis(Z);
    }
    if (count[R] == 8 || dist[R] > 20000){ // R axis occational zero
        zeroAxis(R);
    }
    
    if(axis == R || axis == Z){
        count[axis]++;
    }
    //reset pid values
    clearPID();
    
    
    reached[axis] = false;
    
        
    if (errFnc[axis][0] <= posTol[axis] && errFnc[axis][0] >= -posTol[axis]){
        reached[axis] = true;
    } // tells you if axis is in position


    if (reached[axis] == false){
        if(tgt[axis] - pos[axis] < 40 && tgt[axis] - pos[axis] > -40 && axis == R){       //if too close it moves slightly backward
            motorR.stop();
            motorR.setSpeed(250);
            motorR.forward();
            delay(70);
            delay(70);
            motorR.stop();
            Serial.println("Small Jump on R");
            delay(50);
        }

        timetgt[axis] = millis();
    }
    
   
    while (reached[axis] == false){
        if(axis == Th){
            
            if(pos[R] - tgt[R] <= 5 && pos[R] - tgt[R] >= -5){//ensures that arm is always at correct R when cutting
            
            }else{
            motorTh.stop();
            delay(100);
            go_to_tgt(R);
            
            delay(10);
            clearPID();
            }
            
            velPID[Th] = (int32_t)doVelPID(Th);
            
            if(velPID[Th] >= 255){
                motorTh.setSpeed(255);
                motorTh.backward();
            } else if (velPID[Th] < 255 && velPID[Th] > 0) {
                motorTh.setSpeed(velPID[Th]);
                motorTh.backward();
            } else if (velPID[Th] == 0) {
                motorTh.setSpeed(0);
                motorTh.backward();
            } else if (velPID[Th] > -255 && velPID[Th] < 0 ) {
                motorTh.setSpeed(-(velPID[Th]));
                motorTh.forward();
            } else if (velPID[Th] <= -255) {
                motorTh.setSpeed(255);
                motorTh.forward();
            } else {
                motorTh.setSpeed(0);
                
            }

            if(pos[Th] - tgt[Th] <= 20 && pos[Th] - tgt[Th] >= -20){ //checks if at tgt
                reached[Th] = true;
                motorTh.stop();
                delay(200);    
            }
           /*
           //decreases power load if platform is slow
            if (vel[Th] < 100000 && vel[Th] > -100000){
            motorQ.setSpeed(120);
            motorQ.forward();
            }else{
            motorQ.setSpeed(150);
            motorQ.forward();
            }
            */
            //excecutes if stuck
            // if Theta keeps on being stuck hit the bearing from underneath a couple times
            if (vel[Th] == 0 && stall == false){
            timetgt[Th] = millis();
            stall = true;            
            }
            if (vel[Th] != 0){
            stall = false;
            }
            if(stall == true && millis()-timetgt[Th] > 800){
                Serial.println("Jolt on Th");
                motorTh.setSpeed(0);
                motorTh.forward();
                delay(200);
                motorTh.setSpeed(255);
                motorTh.backward();
                delay(200);
                motorTh.setSpeed(255);
                motorTh.forward();
                delay(250);
                stall = false;
            }
        }
        
        
        if(axis == R){
            PID[R] = (int32_t)doPID(R);

            if(PID[R] < 0){                             //normal PID control 
                if(220 - PID[R] > 245){
                    motorR.setSpeed(245);
                }else{
                    motorR.setSpeed(220 - PID[R]);
                }
                
                motorR.forward();
                dir[R] = 0;
            }
            if(PID[R] > 0){
                if(220 + PID[R] > 245){
                    motorR.setSpeed(245);
                }else{
                    motorR.setSpeed(220 + PID[R]);
            }
            
            motorR.backward();
            dir[R] = 1;
            }

            if(tgt[R]-pos[R] < 20 && dir[R] == 1 && millis() - timetgt[R] > 100 && tgt[R] < 1000){ //slows down on approach
                motorR.setSpeed(180);
                motorR.backward();
                delay(1);
            }
            if(tgt[R]-pos[R] > -30 && dir[R] == 0 && millis() - timetgt[R] > 100 && tgt[R] < 1000){
                motorR.setSpeed(150);
                motorR.forward();
                delay(1);
            }


            if(pos[R] - tgt[R] <= 1 && pos[R] - tgt[R] >= -1){ //checks if at tgt
                reached[R] = true;
                motorR.stop();
                delay(200);
                if(pos[R] - tgt[R] <= 1 && pos[R] - tgt[R] >= -1){ //checks if at tgt
                reached[R] = true;
                motorR.stop();
                delay(10);
            }
            }
            
            
            if(millis() - timetgt[R] > 800 && Derr[R] == 0){ //if its taking too long it gets a jolt
                motorR.stop();
                motorR.setSpeed(255);
                motorR.forward();
                delay(100);
                motorR.stop();
                Serial.println("Jolt on R");
                delay(100);
                timetgt[R] = millis();
                count[R]++;
            }
            
            if (count[R] == 20 || dist[R] > 20000){ // R axis occational zero
                zeroAxis(R);
            }
        }

        if(axis == Z){
            PID[Z] = (int32_t)doPID(Z);


            if(PID[Z] > 0){                             //normal PID control
                if(120 + PID[Z] > 255){
                    motorZ.setSpeed(255);
                }else{
                    motorZ.setSpeed(120 + PID[Z]);
                }
                
                motorZ.forward();
                dir[Z] = 1;
            }
            if(PID[Z] < 0){
                if(120 - PID[Z] > 255){
                    motorZ.setSpeed(255);
                }else{
                    motorZ.setSpeed(120 - PID[Z]);
                }
                
                motorZ.backward();
                dir[Z] = 0;
            }

            
            if(pos[Z] - tgt[Z] <= 2 && pos[Z] - tgt[Z] >= -2){ //checks if at tgt
                reached[Z] = true;
                motorZ.stop();
                delay(200);
            }
        }
    }
}

float doPID(uint8_t axis){
    float resultPID = 0;

    listShift(time);
    time[0] = millis();

    for (int ax = 0; ax<3; ax++) {
        listShift(errFnc[ax]); //shifts list to make way for new values
        errFnc[ax][0] = tgt[ax] - pos[ax];
    }

    
    Perr[axis] = errFnc[axis][0]; //P
    
    if (errFnc[axis][0] < 100 && errFnc[axis][0] > -100){ //I
        Ierr[axis] = Ierr[axis] + errFnc[axis][0]*(time[0]-time[1]); // milli pos second
        if (Ierr[axis]*coI[axis] > limI[axis]){
            Ierr[axis] = limI[axis]/coI[axis];
        }
        if (Ierr[axis]*coI[axis] < -limI[axis]){
            Ierr[axis] = -limI[axis]/coI[axis];
        }
    }
    
    Derr[axis] = (float)(errFnc[axis][0] - errFnc[axis][len-1])/(time[0]-time[len-1]); // milli pos per second //D
   
    
    return resultPID = coP[axis]*Perr[axis] + coI[axis]*Ierr[axis] + coD[axis]*Derr[axis]; //putting it all together we get PID
}

float doVelPID(uint8_t axis) {
    float resultVelPID = 0;
    
    listShift(time);
    time[0] = millis();
    
    for (int ax = 0; ax<3; ax++) {
        listShift(errFnc[ax]); //shifts list to make way for new values
        errFnc[ax][0] = tgt[ax] - pos[ax];
    }

    
    for (int ax = 0; ax<3; ax++) {
        vel[ax] = (1000000*(errFnc[ax][0] - errFnc[ax][len-1])) / (time[0] - time[len-1]);
        listShift(velErrFnc[ax]);
        velErrFnc[ax][0] = tgtVel[ax] - vel[ax];
    }
    
     vPerr[axis] = velErrFnc[axis][0];
     
     vIerr[axis] = vIerr[axis] + (velErrFnc[axis][0])*(time[0] - time[1]);
        if (vIerr[axis]*covI[axis] > limvI[axis]){
            vIerr[axis] = limvI[axis]/covI[axis];
        }
        if (vIerr[axis]*covI[axis] < -limvI[axis]){
            vIerr[axis] = -limvI[axis]/covI[axis];
        }
        
     vDerr[axis] = (velErrFnc[axis][0] - velErrFnc[axis][len-1]) / (time[0] - time[len-1]);
      
    return resultVelPID = vPerr[axis] * covP[axis] + vIerr[axis] * covI[axis] + vDerr[axis] * covD[axis];
}

void all_to_tgt(int32_t Rtgt, uint32_t Ztgt){ //use this method for automatic positioning
    
    Serial.println("all new tgt: (R,Z)");
    Serial.print (Rtgt);
    Serial.print (" ");
    Serial.println(Ztgt);

    tgt[R] =  Rtgt;
    tgt[Z] =  Ztgt;

    reached [Z] = false;
    reached [R] = false;

    if(pos[R] - tgt[R] <= 2 && pos[R] - tgt[R] >= -2){
        reached[R] = true;
    }
    if(pos[Z] - tgt[Z] <= 30 && pos[Z] - tgt[Z] >= -30){
        reached[Z] = true;
    }

    
    while (reached[R] == false || reached[Z] == false){
        
        if (reached[Z] == true){
            go_to_tgt(R);
        }else{
            if (go_dir == false){
                zeroAxis(R);
                go_to_tgt(Z);
                go_to_tgt(R);
            }else{
                go_to_tgt(Z);  
            }
        }
        
        if(pos[R] - tgt[R] <= 2 && pos[R] - tgt[R] >= -2){
            reached[R] = true;
        }
        if(pos[Z] - tgt[Z] <= 5 && pos[Z] - tgt[Z] >= -5){
            reached[Z] = true;
        }
    }
    
    pos[Th] = 0;
    tgt[Th] = -21000;
    tgtVel[Th] = 500000;
    if (go_dir == false){
        go_to_tgt(Th);
    }
    
    go_dir == false;
    reached [Z] = false;
    reached [R] = false;
    reached [Th] = false;
}

void clearPID(){
    
    for(int ax = 0; ax < 3; ax++) {
    Ierr[ax] = 0;
    }
    for(int ax = 0; ax < 3; ax++) {
    vIerr[ax] = 0;
    }
    
    for(i=0; i<len; ++i){
        time[i] = millis();
    }
    
    for (int ax = 0; ax < 3; ax++) {
        for(i=0; i<len; ++i){ //reset errFnc for new tgt
            errFnc[ax][i] = tgt[ax] - pos[ax]; 
        }
    }
        
    for (int ax = 0; ax<3; ax++) {
        for(i=0; i<len; ++i){
        vel[ax] = 0;
        velErrFnc[ax][i] = tgtVel[ax] - vel[ax];
        }
    }
}



