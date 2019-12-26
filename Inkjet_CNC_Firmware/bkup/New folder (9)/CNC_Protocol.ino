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

    for(loopvar[Z]=(int32_t)(start_Z* uconv[Z]); loopvar[Z]<point_0[Z]; loopvar[Z] += 100){
        for(loopvar[R]=(int32_t)(start_R* uconv[R]); loopvar[R]<point_0[R]; loopvar[R] += 20){
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


void cut_pgrm(){      //in millimeters // for trimming layers off a blank. // only do a small cut each time
     
    for(loopvar[Z]=0; loopvar[Z]<299; loopvar[Z] += 1){
        if (dpthlst[loopvar[Z]] != 0){
            point_0[Z] = loopvar[Z]*uconv[Z];
            point_0[R] = (int32_t)(dpthlst[loopvar[Z]]*uconv[R]);
            for(loopvar[R]=0; loopvar[R]<point_0[R]; loopvar[R] += 20){
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
