/* ************************************************************************** */
/** Descriptive File Name
 * 
  @Summary
    PID algorithm.

  @Description
    Since the initialize function does not have a return statement, line 34 and 35 
 *  is what initializes the objects manually.
 */
/* ************************************************************************** */
//#if !defined(_PID_H)
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <plib.h>
#include <math.h>

/***** Constants*/
#define iMax     10000 // integrator max
#define iMin     -10000 // integrator min

/****** global variables ******/
float output;

float LW_avg_meas;   // calculated in input capture 3
float RW_avg_meas;    // calculated in input capture 2

/******       Constructor           ******/

struct PID{
    float Kp, Ki, Kd;			// PID Constants
    float Ep, Ei, Ed, error_val_old;//, error_val_n;	// Error Values
};

typedef struct PID pid; //wheel 
pid leftWheel;
pid rightWheel;

/*                  initializer                 */

void PID_init(pid* P,float kp, float ki, float kd);                                 // 1500         8000

/*                  Modifiers                   */

void PID_control(pid* P,uint16_t wheel,uint16_t desiredTime, uint16_t avgTime);

#endif /* PID_H */

/* *****************************************************************************
 End of File
 */
