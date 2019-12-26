void go_to_tgt(uint8_t axis) { //R axis positioning is only prescise on extension //only use this method for manual i-know-what-i-am-doing positioning.
    //if((tgt[axis] > maxPos[axis]) || (tgt[axis] < minPos[axis])){
    //  if(axis != Th)  {
    //    exeErr("Unhandled Error: tgt out of range");
    //}
    //}
    
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

    
    Ierr[R] = 0;
    Ierr[Z] = 0;
    
    for (int ax = 0; ax < 3; ax++) {
        for(i=0; i<len; ++i){ //reset errFnc for new tgt
            errFnc[ax][i] = tgt[ax] - pos[ax]; 
        }
    }
    
    for(i=0; i<len; ++i){
        time[i] = millis();
    }
    
    
    reached[axis] = false;
    
        
    if (errFnc[axis][0] <= posTolerance[axis] && errFnc[axis][0] >= -posTolerance[axis]){
        reached[axis] = true;
    } // tells u if axis is in position


    if (reached[axis] == false){
        if(tgt[axis] - pos[axis] < 40 && tgt[axis] - pos[axis] > -40 && axis == R){       //if too close it moves slightly backward
            motorR.stop();
            motorR.setSpeed(250);
            motorR.forward();
            delay(70);
            motorR.stop();
            Serial.println("Small Jump on R");
            delay(50);
        }

        timetgt[axis] = millis();
    }
    
    timeElapsed = millis(); //for speed control
    
    while (reached[axis] == false){
        if(axis == Th){

            speedPID = (int16_t)doSpeedPID(timeElapsed);

            if(150 + speedPID > 255){
                motorTh.setSpeed(255);
            } else if (150 + speedPID < 1) {
                motorTh.setSpeed(0);
            } else {
                motorTh.setSpeed(150 + speedPID);
            }
            motorTh.backward();

            if(pos[Th] - tgt[Th] <= 20 && pos[Th] - tgt[Th] >= -20){ //checks if at tgt
                reached[Th] = true;
                motorTh.stop();
                delay(200);    
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

    for (int ax = 0; ax<3; ax++) {
        listShift(errFnc[ax]); //shifts list to make way for new values
        errFnc[ax][0] = tgt[ax] - pos[ax];
    }

    listShift(time);
    time[0] = millis();
    
    Perr[axis] = errFnc[axis][0]; //P
    
    if (errFnc[axis][0] < 100 && errFnc[axis][0] > -100){ //I
        Ierr[axis] = Ierr[axis] + errFnc[axis][0]*(time[0]-time[1]); // milli pos second
        if (Ierr[axis]*coI[axis] > limI[axis]){
            Ierr[axis] = limI[axis]/coI[axis];
        }
    }
    
    Derr[axis] = (float)(errFnc[axis][0] - errFnc[axis][len-1])/(time[0]-time[len-1]); // milli pos per second //D
    
    Serial.println(errFnc[Th][0]);    
    
    return resultPID = coP[axis]*Perr[axis] + coI[axis]*Ierr[axis] + coD[axis]*Derr[axis]; //putting it all together we get PID
}

float doSpeedPID(int32_t timeElapsed) {
    float resultPID = 0;
    
    listShift(time);
    time[0] = millis();
    
    listShift(errFnc[Th]); //shifts list to make way for new values
    errFnc[Th][0] = pos[Th];
    
    int32_t dt = time[0] - time[len-1];

    float vel = (float)((errFnc[Th][0]) - (errFnc[Th][len-1])) / (float)(dt);
    
    floatListShift(speedErrFnc);
    speedErrFnc[0] = desiredSpeed - vel; // 2 is desired speed in enc/milis
    
    float DsErr = (speedErrFnc[0] - speedErrFnc[len-1]) / dt;
    float IsErr = desiredSpeed * (time[0] - timeElapsed) - pos[Th]; //2 is desired speed
    float PsErr = speedErrFnc[0];
    
 //   Serial.print(1);
  //  Serial.print(" ");
  //  Serial.print(-1); 
  // Serial.print(" ");  
    //Serial.print(" ");
    //Serial.print(PsErr * speedCo[0]);
    //Serial.print(" ");
    //Serial.print(IsErr * speedCo[2]);
    //Serial.print(" ");
    //Serial.print(DsErr * speedCo[1]);
    //Serial.print(" ");
    //Serial.println(PsErr * speedCo[0] + DsErr * speedCo[1] + IsErr * speedCo[2]);
    
    return PsErr * speedCo[0] + DsErr * speedCo[1] + IsErr * speedCo[2];
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
    tgt[Th] = 25000;
    
    if (go_dir == false){
        go_to_tgt(Th);
    }
    
    go_dir == false;
    reached [Z] = false;
    reached [R] = false;
}
