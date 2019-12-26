void go_to_tgt(uint8_t axis) { //R axis positioning is only prescise on extension //only use this method for manual i-know-what-i-am-doing positioning.
    if((tgt[axis] > maxPos[axis]) || (tgt[axis] < minPos[axis])){
      if(axis != Th)  {
        exeErr("Unhandled Error: tgt out of range");
        }
    }
    
    if (count[Z] == 20 || dist[Z] > 120000 || (pos[Z] > 20000 && tgt[Z] - pos[Z] < 0)){ //Z axis occational zero
        zeroAxis(R);
        zeroAxis(Z);
    }
    if (count[R] == 20 || dist[R] > 20000){ // R axis occational zero
        zeroAxis(R);
    }
    
    if(axis == R || axis == Z){
        count[axis]++;
    }
    //reset pid values
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
            if(stall == true && millis()-timetgt[Th] > 1500){
                Serial.println("Jolt on Th");
                motorTh.setSpeed(0);
                motorTh.forward();
                delay(200);
                motorTh.setSpeed(255);
                motorTh.backward();
                delay(200);
                stall = false;
            }
        }
        
        
        if(axis == R){
            PID[R] = (int32_t)doPID(R);

            if(PID[R] < 0){                             //normal PID control 
                if(220 - PID[R] > 235){
                    motorR.setSpeed(235);
                }else{
                    motorR.setSpeed(220 - PID[R]);
                }
                
                motorR.forward();
                dir[R] = 0;
            }
            if(PID[R] > 0){
                if(220 + PID[R] > 235){
                    motorR.setSpeed(235);
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
            if(tgt[R]-pos[R] > -20 && dir[R] == 0 && millis() - timetgt[R] > 100 && tgt[R] < 1000){
                motorR.setSpeed(150);
                motorR.forward();
                delay(1);
            }


            if(pos[R] - tgt[R] <= 1 && pos[R] - tgt[R] >= -1){ //checks if at tgt
                reached[R] = true;
                motorR.stop();
                delay(200);
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
