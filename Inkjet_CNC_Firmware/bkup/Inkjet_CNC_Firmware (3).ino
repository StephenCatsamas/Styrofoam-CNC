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

L298N motorA(ENR, IN1R, IN2R);  //create motors on R and Z axis
L298N motorB(ENZ, IN1Z, IN2Z);
L298N* motor[] = { &motorA, &motorB };


// indexes for encoder R and Z
const uint8_t R = 0;
const uint8_t Z = 1;
const uint8_t Th = 2;

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
volatile int32_t pos[] = {0, 0, 0};                    //axis position
volatile int32_t dist[] = {0,0, 0};                    //axis total distance traveled, gets rest from time to time
volatile int32_t tgt[] = {0, 0, 0};                    // Taget position for carriage
volatile int32_t tgt_man[] = {0, 0, 0};               //a secondary tgt marker for manual positioning.
volatile int32_t PID[] = {0,0};                     //resultant PID value after calling in one go_to_tgt
volatile float Perr[] = {0,0};                      //PID calulation internal use only
volatile float Ierr[] = {0,0};
volatile float Derr[] = {0,0};                      
volatile uint8_t count[] = {0,0};                   //counts how many to_tgts for occational zeroing

const int8_t len = 30;                             //sets the average period for which the derivative occurs
volatile int32_t time[len];
volatile int32_t errFncR[len];
volatile int32_t errFncZ[len];
volatile uint32_t timetgt[len];

volatile int32_t point_0[] = {0,0};             //for use in random functions
volatile int32_t point_1[] = {0,0};
volatile int32_t loopvar[] = {0,0};
volatile int32_t v;
volatile int8_t i;

void setup() {
    Serial.begin(2000000);

    zeroAxis(R);
    zeroAxis(Z);

    pinMode(encPhaseA[R], INPUT_PULLUP);
    pinMode(encPhaseA[Z], INPUT_PULLUP);
    pinMode(encPhaseA[Th], INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(encPhaseA[R]), encoderTriggerR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Z]), encoderTriggerZ, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPhaseA[Th]), encoderTriggerTh, CHANGE);
}

void loop() {
    //cut_cylinder(30, 100, 50);
    pos_reader();
    manual_ctrl();
}

//motor[axis]->setSpeed(0 to 255);
//motor[axis]->backward();
//motor[axis]->forward();
//motor[axis]->stop();

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
    tgt[Th] = 25000;
    
    if (go_dir == false){
        while (pos[Th] < tgt[Th] && pos[Th] > -tgt[Th]){
            delay(10);
        }
    }
    
    go_dir == false;
    reached [Z] = false;
    reached [R] = false;
}

void go_to_tgt(uint8_t axis) { //R axis positioning is only prescise on extension //only use this method for manual i-know-what-i-am-doing positioning.
    if(tgt[axis] > maxPos[axis] || tgt[axis] < minPos[axis]){
        exeErr("Unhandled Error: tgt out of range");
    }
    
    if (count[Z] == 20 || dist[Z] > 120000 || (pos[Z] > 20000 && tgt[Z] - pos[Z] < 0)){ //Z axis occational zero
        zeroAxis(R);
        zeroAxis(Z);
    }
    if (count[R] == 20 || dist[R] > 20000){ // R axis occational zero
        zeroAxis(R);
    }
    
    if(axis == R){
        count[R]++;
    }
    if(axis == Z){
        count[Z]++;
    }

    
    Ierr[R] = 0;
    Ierr[Z] = 0;
    for(i=0; i<len; ++i){ //reset errFnc for new tgt
        errFncR[i] = tgt[axis] - pos[axis]; 
    }
    for(i=0; i<len; ++i){
        errFncZ[i] = tgt[axis] - pos[axis];
    }
    for(i=0; i<len; ++i){
        time[i] = millis();
    }
    
    
    reached[axis] = false;
    if (errFncR[0] <= 2 && errFncR[0] >= -2 && axis == R){
        reached[R] = true;
    }
    if (errFncZ[0] <= 5 && errFncZ[0] >= -5 && axis == Z){
        reached[Z] = true;
    }


    if (reached[axis] == false){
        if(tgt[axis] - pos[axis] < 40 && tgt[axis] - pos[axis] > -40 && axis == R){       //if too close it moves slightly backward
            motor[axis]->stop();
            motor[axis]->setSpeed(250);
            motor[axis]->forward();
            delay(70);
            motor[axis]->stop();
            Serial.println("Small Jump on R");
            delay(50);
        }

        timetgt[axis] = millis();
    }
    
    
    while (reached[axis] == false){
        if(axis == R){
            PID[R] = (int32_t)doPID(R);

            if(PID[R] < 0){                             //normal PID control 
                if(220 - PID[R] > 235){
                    motor[R]->setSpeed(235);
                }else{
                    motor[R]->setSpeed(220 - PID[R]);
                }
                
                motor[R]->forward();
                dir[R] = 0;
            }
            if(PID[R] > 0){
                if(220 + PID[R] > 235){
                    motor[R]->setSpeed(235);
                }else{
                    motor[R]->setSpeed(220 + PID[R]);
            }
            
            motor[R]->backward();
            dir[R] = 1;
            }

            if(tgt[R]-pos[R] < 20 && dir[R] == 1 && millis() - timetgt[R] > 100 && tgt[R] < 1000){ //slows down on approach
                motor[R]->setSpeed(180);
                motor[R]->backward();
                delay(1);
            }
            if(tgt[R]-pos[R] > -20 && dir[R] == 0 && millis() - timetgt[R] > 100 && tgt[R] < 1000){
                motor[R]->setSpeed(150);
                motor[R]->forward();
                delay(1);
            }


            if(pos[R] - tgt[R] <= 1 && pos[R] - tgt[R] >= -1){ //checks if at tgt
                reached[R] = true;
                motor[R]->stop();
                delay(200);
            }
            
            
            if(millis() - timetgt[R] > 800 && Derr[R] == 0){ //if its taking too long it gets a jolt
                motor[R]->stop();
                motor[R]->setSpeed(255);
                motor[R]->forward();
                delay(100);
                motor[R]->stop();
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
                    motor[Z]->setSpeed(255);
                }else{
                    motor[Z]->setSpeed(120 + PID[Z]);
                }
                
                motor[Z]->forward();
                dir[Z] = 1;
            }
            if(PID[Z] < 0){
                if(120 - PID[Z] > 255){
                    motor[Z]->setSpeed(255);
                }else{
                    motor[Z]->setSpeed(120 - PID[Z]);
                }
                
                motor[Z]->backward();
                dir[Z] = 0;
            }

            
            if(pos[Z] - tgt[Z] <= 2 && pos[Z] - tgt[Z] >= -2){ //checks if at tgt
                reached[Z] = true;
                motor[Z]->stop();
                delay(200);
            }
            
            
            /* if(millis() - timetgt[Z] > 1500){ //if taking too long it get a jolt // not really a good idea as it screws up the tension in the rope
                motor[Z]->setSpeed(0);
                motor[Z]->forward();
                Serial.print("Jolt Z");
                delay(200);
                timetgt[Z] = millis();
            } */
        }
    }
}

float doPID(uint8_t axis){
    float resultPID = 0;

    listShift(errFncR); //shifts list to make way for new values
    listShift(errFncZ);
    listShift(time);

    errFncR[0] = tgt[R] - pos[R]; // calulates new list values
    errFncZ[0] = tgt[Z] - pos[Z];
    time[0] = millis();

    
    if (axis == R){ //calulates PID components
        Perr[R] = errFncR[0]; //P
        
        if (errFncR[0] < 100 && errFncR[0] > -100){ //I
            Ierr[R] = Ierr[R] + errFncR[0]*(time[0]-time[1]); // milli pos second
            if (Ierr[R]*coI[R] > 15){
                Ierr[R] = 15/coI[R];
            }
        }
        
        Derr[R] = (float)(errFncR[0] - errFncR[len-1])/(time[0]-time[len-1]); // milli pos per second //D
    }
    if (axis == Z){
        Perr[Z] = errFncZ[0]; //P
        
        if (errFncZ[0] < 100 && errFncZ[0] > -100){ //I
            Ierr[Z] = Ierr[Z] + errFncZ[0]*(time[0]-time[1]); // milli pos second
            if (Ierr[Z]*coI[Z] > 75){
                Ierr[Z] = 75/coI[Z];
            }
        }
        
        Derr[Z] = (float)(errFncZ[0] - errFncZ[len-1])/(time[0]-time[len-1]); // milli pos per second //D
    }

    
    return resultPID = coP[axis]*Perr[axis] + coI[axis]*Ierr[axis] + coD[axis]*Derr[axis]; //putting it all together we get PID
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

void manual_ctrl(){ //for manual control of motors
    int8_t inread =0;
    
    if (Serial.available() > 0){
        inread = Serial.read();
        
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
}
    
void cut_cylinder(float radius, float height, float start_Z){      //in millimeters //start_Z is an offest downwards
    point_0[R] = (int32_t)(radius * uconv[R]);
    point_0[Z] = (int32_t)((height+start_Z) * uconv[Z]);

    for(loopvar[Z]=(int32_t)(start_Z* uconv[Z]); loopvar[Z]<point_0[Z]; loopvar[Z] += 100){
        for(loopvar[R]=100; loopvar[R]<point_0[R]; loopvar[R] += 20){
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

void zeroAxis(uint8_t axis){
    if (axis == R){ //as it says on the tin
        Serial.println("Zero R");
        motor[R]->setSpeed(240);
        motor[R]->forward();
        delay(1000);
        motor[R]->stop();
        delay(500);
        pos[R] = 0;
        dist[R] = 0;
        count[R] = 0;
        reached[R] = false;
    }
    if (axis == Z){ //as it says on the tin
        Serial.println("Zero Z");
        motor[Z]->setSpeed(255); 
        motor[Z]->backward();
        delay(4000);
        motor[Z]->stop(); 
        delay(500);
        pos[Z] = 0;
        dist[Z] = 0;
        count[Z] = 0;
        reached[Z] = false;
    }

}

void exeErr(String errMsg){
    //zeros axis but without resetting variables.
    motor[R]->setSpeed(240);
    motor[R]->forward();
    delay(1000);
    motor[R]->stop();
    delay(500);
    motor[Z]->setSpeed(255); 
    motor[Z]->backward();
    delay(4000);
    motor[Z]->stop(); 
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
    
    Serial.print("errFncR []: [");
    Serial.print(errFncR[0]);
     for (i=1; i<len; ++i){
        Serial.print(", ");
        Serial.print(errFncR[i]);
    }
    Serial.print("]");
    Serial.println("");
    Serial.println("");
    
    Serial.print("errFncZ []: [");
    Serial.print(errFncZ[0]);
     for (i=1; i<len; ++i){
        Serial.print(", ");
        Serial.print(errFncZ[i]);
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