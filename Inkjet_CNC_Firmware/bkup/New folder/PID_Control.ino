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
        errFncTh[i] = tgt[axis] - pos[axis];
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
    
    
    while (reached[axis] == false){
        if(axis == Th){
            PID[Th] = (int32_t)doPID(Th);
            Serial.print(pos[Th]);
            Serial.print(" ");
            Serial.print(Derr[Th]);
            Serial.print(" ");
            Serial.println(PID[Th]);
            if(PID[Th] > 0){                             //normal PID control
                if(200 + PID[Th] > 255){
                    motorTh.setSpeed(255);
                }else{
                    motorTh.setSpeed(200 + PID[Th]);
                }
                
                motorTh.backward();
            }
            if(PID[Th] < 0){
                if(200 - PID[Th] > 255){
                    motorTh.setSpeed(255);
                }else{
                    motorTh.setSpeed(200 - PID[Th]);
                }
                
                motorTh.forward();
            }

            
            if(pos[Th] - tgt[Th] <= 2000 && pos[Th] - tgt[Th] >= -2000){ //checks if at tgt
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

    listShift(errFncR); //shifts list to make way for new values
    listShift(errFncZ);
    listShift(time);

    errFncR[0] = tgt[R] - pos[R]; // calulates new list values
    errFncZ[0] = tgt[Z] - pos[Z];
    errFncTh[0] = tgt[Th] - pos[Th];
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
    
    if (axis == Th){
        Perr[Th] = errFncTh[0]; //P
        
        if (errFncTh[0] < 100 && errFncTh[0] > -100){ //I
            Ierr[Th] = Ierr[Th] + errFncTh[0]*(time[0]-time[1]); // milli pos second
            if (Ierr[Th]*coI[Th] > 75){
                Ierr[Th] = 75/coI[Th];
            }
        }
        
        Derr[Th] = (float)(errFncTh[0] - errFncTh[len-1])/(time[0]-time[len-1]); // milli pos per second //D
    }
    
    return resultPID = coP[axis]*Perr[axis] + coI[axis]*Ierr[axis] + coD[axis]*Derr[axis]; //putting it all together we get PID
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
