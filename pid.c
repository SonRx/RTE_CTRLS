#include "pid.h"

void PID_init(pid* P, float kp, float ki, float kd){//, uint16_t iMin, uint16_t iMax){
    P->Kp = kp;
    P->Ki = ki;
    P->Kd = kd;
   // P->iMin = iMin;
   // P->iMax = iMax;
    P->Ep = 0; 
    P->Ei = 0;
    P->Ed = 0;
    P->error_val_old = 0;
    //P->error_val_n = 0;
    //return;
}

void PID_control(pid* P,uint16_t wheel,uint16_t desiredTime, uint16_t avgTime){
    float error_val_n = (float)avgTime - (float)desiredTime; // using time needed to subtract desired from measured
    
    P->Ep = error_val_n * P->Kp; // Error Proportional
    
    P->Ei += error_val_n * P->Ki; // Error Integration
    if (P->Ei > iMax){         // Max
        P->Ei = iMax;
    }
    else if (P->Ei < iMin){    // Min
        P->Ei = iMin;
    }
    
    P->Ed = (error_val_n - P->error_val_old) * P->Kd; // Error differential
    
    output = (P->Ep + P->Ei + P->Ed); // float
    
    P->error_val_old = error_val_n;  // update old value for next iteration
    
    if (wheel == 0){ // left wheel?
        if (output > 10000){
           OC3R = 10000;
           OC3RS = 10000;
       }
       else if (output < 1000){
           OC3R = 1000;
           OC3RS = 1000;
       }
       else {
           output = floor(output);
           OC3R = (uint16_t)output;
           OC3RS = (uint16_t)output;
       }
   }
    else{ // right wheel?
        if (output > 10000){
           OC2R = 10000;
           OC2RS = 10000;
       }
       else if (output < 1000){
           OC2R = 1000;
           OC2RS = 1000;
       }
       else {
           output = floor(output);
           OC2R = (uint16_t)output;
           OC2RS = (uint16_t)output;
       }
   }      
}


/* *****************************************************************************
 End of File
 */
