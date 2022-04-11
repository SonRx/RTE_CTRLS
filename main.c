/************************************************************************/
/*																		*/
/*	main.c	--	Main program module for project							*/
/*																		*/
/************************************************************************/
/*	Author: 	Dion Moses												*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This program is a reference design for the Digilent	Basic			*/
/*	Robotic Development Kit (RDK-Basic) with the Cerebot 32MX4 			*/
/*	Microcontroller board.  It uses two timers to drive two motors 		*/
/*	with output compare modules.										*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 12/09/09(DionM): created											*/
/*   12/29/09(LeviB): altered to add movement functions and PmodBtn and */
/*					  PmodSwt functionality								*/
/*	 12/08/10(AaronO): renamed to RDK_Basic								*/	
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include "stdtypes.h"
#include "config.h"
#include "MtrCtrl.h"
#include "spi.h"
#include "util.h"
#include <math.h>
#include "pid.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4
/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
#ifndef OVERRIDE_CONFIG_BITS

#pragma config ICESEL   = ICS_PGx2		// ICE/ICD Comm Channel Select
#pragma config BWP      = OFF			// Boot Flash Write Protect
#pragma config CP       = OFF			// Code Protect
#pragma config FNOSC    = PRIPLL		// Oscillator Selection
#pragma config FSOSCEN  = OFF			// Secondary Oscillator Enable
#pragma config IESO     = OFF			// Internal/External Switch-over
#pragma config POSCMOD  = HS			// Primary Oscillator
#pragma config OSCIOFNC = OFF			// CLKO Enable
#pragma config FPBDIV   = DIV_8			// Peripheral Clock divisor
#pragma config FCKSM    = CSDCMD		// Clock Switching & Fail Safe Clock Monitor
#pragma config WDTPS    = PS1			// Watchdog Timer Postscale
#pragma config FWDTEN   = OFF			// Watchdog Timer 
#pragma config FPLLIDIV = DIV_2			// PLL Input Divider
#pragma config FPLLMUL  = MUL_16		// PLL Multiplier
#pragma config UPLLIDIV = DIV_2			// USB PLL Input Divider
#pragma config UPLLEN   = OFF			// USB PLL Enabled
#pragma config FPLLODIV = DIV_1			// PLL Output Divider
#pragma config PWP      = OFF			// Program Flash Write Protect
#pragma config DEBUG    = OFF			// Debugger Enable/Disable
    
#endif

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#define	stPressed	1
#define	stReleased	0
#define	cstMaxCnt	10 // number of consecutive reads required for
					   // the state of a button to be updated

struct btn {
	BYTE	stBtn;	// status of the button (pressed or released)
	BYTE	stCur;  // current read state of the button
	BYTE	stPrev; // previous read state of the button
	BYTE	cst;	// number of consecutive reads of the same button 
					// state
};

//PmodCLS instructions
static	char szClearScreen[] = { 0x1B, '[', 'j', 0};

static	char szCursorOff[] = { 0x1B, '[', '0', 'c', 0 };
static	char szBacklightOn[]     = { 0x1B, '[', '3', 'e', 0 };

static	char szScrollLeft[] = {0x1B, '[', '1', '@', 0}; 
static	char szScrollRight[] = {0x1B, '[', '1', 'A', 0}; 
static	char szWrapMode[] = {0x1B, '[', '0', 'h', 0}; 

static	char szCursorPos[] = {0x1B, '[', '1', ';', '0', 'H', 0}; 
/* ------------------------------------------------------------ */
/*				Global Variables				                */
/* ------------------------------------------------------------ */

volatile	struct btn	btnBtn1;
volatile	struct btn	btnBtn2;

volatile	struct btn	PmodBtn1;
volatile	struct btn	PmodBtn2;
volatile	struct btn	PmodBtn3;
volatile	struct btn	PmodBtn4;

volatile	struct btn	PmodSwt1;
volatile	struct btn	PmodSwt2;
volatile	struct btn	PmodSwt3;
volatile	struct btn	PmodSwt4;

uint32_t IC2Counter = 0;  
uint32_t IC3Counter = 0; 

#define MAX 156
uint16_t cntr_ms_interval = 19; // 20 ms interval

uint16_t IC2Arr[MAX];
uint16_t IC3Arr[MAX];
uint16_t diffCapArr[MAX];
uint16_t diffCapArr2[MAX];

uint16_t fullRotation = 0;


uint16_t RW_time = 6000;    //desired right wheel pulse time (us)
uint16_t LW_time = 6000;    //desired left wheel pulse time (us)

float RW_speed; //= 0.00480769231/(RW_time * 0.000001);
float LW_speed; //= 0.00480769231/(LW_time * 0.000001);
    
float lwSpeed;
float rwSpeed;

float alpha = 0.5; // IIR filter coefficient

float ADCValue0 = 0; // in ADC handler
float ADCValue1 = 0;
float ADCValue2 = 0;

int offsetLeft = 0;
int offsetRight = 0;
int sit2 = 0;
int sit3 = 0;
int sit4 = 0;

uint8_t mode = 0;
/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void    PID_init(pid* P, float kp, float ki, float kd); // two objects created in pid.h
void    PID_control(pid* P,uint16_t wheel,uint16_t desiredTime, uint16_t avgTime);
void	AppInit(void);
void    AdcInit(void);
void	Wait_ms(WORD ms);
void    InitLeds( void );
void    SetLeds( BYTE stLeds );



void __ISR(_ADC_VECTOR, ipl3) _ADC_HANDLER(void)
{
    prtLed4Set = ( 1 << bnLed4 );
    //mAD1ClearIntFlag(); // eq. to IFS1CLR = 2;
    IFS1CLR = ( 1 << 1 );
    
    ADCValue0 = (float)ADC1BUF0*3.3/1023.0; // Reading AN0(zero), pin 1 of connector JJ -- servo sensor (center)
    ADCValue1 = (float)ADC1BUF1*3.3/1023.0;
    ADCValue2 = (float)ADC1BUF2*3.3/1023.0;
    
    if (mode == 0) {
        
        // rest motors to go forward after transition from mode 1(puppy dog).
        dirMtrLeft	= dirMtrLeftBwd;
		dirMtrRight	= dirMtrRightBwd;
        
        if(ADCValue2 < 0.5 && ADCValue1 < 0.4){ // go north-west bound
           LW_avg_meas = LW_time - 3000;
           RW_avg_meas = RW_time - 1500;
        }
        else if (ADCValue1 > 2.60 && ADCValue2 > 1.30){ // avoids hitting the left wall directly, take semi-hard right.
            LW_avg_meas = LW_avg_meas + 2000;
            RW_avg_meas = RW_avg_meas - 750;
        }
        else if(ADCValue1 > 2.15 && ADCValue2 > 2.15){
          RW_avg_meas = RW_avg_meas + 1000;
          sit2 += 1;
          sit4 = 0;
        }
        else if (ADCValue2 > 1.1 && ADCValue2 <= 2.15) { // sweet spot 
           LW_avg_meas = LW_avg_meas + 250;   // decrease speed of left -> moves closer to left wall
           offsetLeft += 1;
            if (offsetLeft >= 50){
                LW_avg_meas = LW_avg_meas - 250;   // increases speed of left -> moves to the right
                offsetLeft = 0;
                offsetRight += 1;
                if (offsetRight >= 50){
                    LW_avg_meas = RW_avg_meas;
                }
            }
        }
        else if(ADCValue2 < 1.1 && ADCValue1 < 1.1){ // allows the bot to steady creep up to the left wall
            LW_avg_meas = LW_time - 500; // slower
            RW_avg_meas = RW_time - 250; // slowed but faster than left
            sit4 += 1;
            if (sit4 >= 125){
                LW_avg_meas = LW_time;
                RW_avg_meas = RW_time;
                DelayMs(200);
                sit4 = 0;
            }
        }
    }
     // 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
    else if (mode == 1){ 
        if (ADCValue0 >= 1.00){ // stop if it senses an object in front
            MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			DelayMs(50);
			MtrCtrlFwdRight();
			UpdateMotors();
			DelayMs(50);
			MtrCtrlStop();
			UpdateMotors();
        }
        else if (ADCValue0 < 0.65){ // move forward
            MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			DelayMs(50);
			MtrCtrlBwdRight();
			UpdateMotors();
			DelayMs(50);
			MtrCtrlStop();
			UpdateMotors();
        }
        else
            UpdateMotors(); 
    }
     
    prtLed4Clr	= ( 1 << bnLed4 );
}

/* --------------------------------------------0
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for Timer 5 interrupt. Timer 5
**		is used to perform software debouncing of the on-board
**		buttons. It is also used as a time base for updating
**		the on-board LEDs and the Pmod8LD LEDs at a regular interval.
*/

void __ISR(_TIMER_5_VECTOR, ipl7) Timer5Handler(void)
{
	static	WORD tusLeds = 0;
    static  HWORD TMR5_Counter = 0;
    
	mT5ClearIntFlag();
	prtLed1Set = ( 1 << bnLed1 );
   
    // 
    if (TMR5_Counter == cntr_ms_interval){
       // do pid cntrl
       // setpoint -> desired pulse time ; input -> avg pulse time from input cap.
       // void PID_error(uint16_t wheel,uint16_t SetPoint, uint16_t input) // 0 for left wheel; 1 for right wheel;

      PID_control(&leftWheel,0,LW_time,LW_avg_meas);
      PID_control(&rightWheel,1,RW_time,RW_avg_meas);
    }
    
    if (TMR5_Counter++ > cntr_ms_interval){
        TMR5_Counter = 0;
    }

	// Read the raw state of the button pins.
	btnBtn1.stCur = ( prtBtn1 & ( 1 << bnBtn1 ) ) ? stPressed : stReleased;
	btnBtn2.stCur = ( prtBtn2 & ( 1 << bnBtn2 ) ) ? stPressed : stReleased;
	
	//Read the raw state of the PmodBTN pins
	PmodBtn1.stCur = ( prtJE1 & ( 1 << bnJE1 ) ) ? stPressed : stReleased;
	PmodBtn2.stCur = ( prtJE2 & ( 1 << bnJE2 ) ) ? stPressed : stReleased;
	PmodBtn3.stCur = ( prtJE3 & ( 1 << bnJE3 ) ) ? stPressed : stReleased;
	PmodBtn4.stCur = ( prtJE4 & ( 1 << bnJE4 ) ) ? stPressed : stReleased;

	//Read the raw state of the PmodSWT pins
	PmodSwt1.stCur = ( prtJA1 & ( 1 << swtJA1 ) ) ? stPressed : stReleased;
	PmodSwt2.stCur = ( prtJA2 & ( 1 << swtJA2 ) ) ? stPressed : stReleased;
	PmodSwt3.stCur = ( prtJA3 & ( 1 << swtJA3 ) ) ? stPressed : stReleased;
	PmodSwt4.stCur = ( prtJA4 & ( 1 << swtJA4 ) ) ? stPressed : stReleased;

	// Update state counts.
	btnBtn1.cst = ( btnBtn1.stCur == btnBtn1.stPrev ) ? btnBtn1.cst + 1 : 0;
	btnBtn2.cst = ( btnBtn2.stCur == btnBtn2.stPrev ) ? btnBtn2.cst + 1 : 0;

	//Update state counts for PmodBTN
	PmodBtn1.cst = (PmodBtn1.stCur == PmodBtn1.stPrev) ? PmodBtn1.cst +1 : 0;
	PmodBtn2.cst = (PmodBtn2.stCur == PmodBtn2.stPrev) ? PmodBtn2.cst +1 : 0;
	PmodBtn3.cst = (PmodBtn3.stCur == PmodBtn3.stPrev) ? PmodBtn3.cst +1 : 0;
	PmodBtn4.cst = (PmodBtn4.stCur == PmodBtn4.stPrev) ? PmodBtn4.cst +1 : 0;

	//Update state counts for PmodSWT
	PmodSwt1.cst = (PmodSwt1.stCur == PmodSwt1.stPrev) ? PmodSwt1.cst +1 : 0;
	PmodSwt2.cst = (PmodSwt2.stCur == PmodSwt2.stPrev) ? PmodSwt2.cst +1 : 0;
	PmodSwt3.cst = (PmodSwt3.stCur == PmodSwt3.stPrev) ? PmodSwt3.cst +1 : 0;
	PmodSwt4.cst = (PmodSwt4.stCur == PmodSwt4.stPrev) ? PmodSwt4.cst +1 : 0;
	
	// Save the current state.
	btnBtn1.stPrev = btnBtn1.stCur;
	btnBtn2.stPrev = btnBtn2.stCur;

	// Save the current state for PmodBTN
	PmodBtn1.stPrev = PmodBtn1.stCur;
	PmodBtn2.stPrev = PmodBtn2.stCur;
	PmodBtn3.stPrev = PmodBtn3.stCur;
	PmodBtn4.stPrev = PmodBtn4.stCur;

	// Save the current state for PmodSWT
	PmodSwt1.stPrev = PmodSwt1.stCur;
	PmodSwt2.stPrev = PmodSwt2.stCur;
	PmodSwt3.stPrev = PmodSwt3.stCur;
	PmodSwt4.stPrev = PmodSwt4.stCur;
	
	// Update the state of button 1 if necessary.
	if ( cstMaxCnt == btnBtn1.cst ) {
		btnBtn1.stBtn = btnBtn1.stCur;
		btnBtn1.cst = 0;
	}
	
	// Update the state of button 2 if necessary.
	if ( cstMaxCnt == btnBtn2.cst ) {
		btnBtn2.stBtn = btnBtn2.stCur;
		btnBtn2.cst = 0;
	}

	//if statements for buttons

	// Update the state of PmodBTN1 if necessary.
	if ( cstMaxCnt == PmodBtn1.cst ) {
		PmodBtn1.stBtn = PmodBtn1.stCur;
		PmodBtn1.cst = 0;
	}
	
	// Update the state of PmodBTN2 if necessary.
	if ( cstMaxCnt == PmodBtn2.cst ) {
		PmodBtn2.stBtn = PmodBtn2.stCur;
		PmodBtn2.cst = 0;
	}

	// Update the state of PmodBTN3 if necessary.
	if ( cstMaxCnt == PmodBtn3.cst ) {
		PmodBtn3.stBtn = PmodBtn3.stCur;
		PmodBtn3.cst = 0;
	}

	// Update the state of PmodBTN4 if necessary.
	if ( cstMaxCnt == PmodBtn4.cst ) {
		PmodBtn4.stBtn = PmodBtn4.stCur;
		PmodBtn4.cst = 0;
	}

	//if statements for switches

	// Update the state of PmodSWT1 if necessary.
	if ( cstMaxCnt == PmodSwt1.cst ) {
		PmodSwt1.stBtn = PmodSwt1.stCur;
		PmodSwt1.cst = 0;
	}
	
	// Update the state of PmodSWT2 if necessary.
	if ( cstMaxCnt == PmodSwt2.cst ) {
		PmodSwt2.stBtn = PmodSwt2.stCur;
		PmodSwt2.cst = 0;
	}

	// Update the state of PmodSWT3 if necessary.
	if ( cstMaxCnt == PmodSwt3.cst ) {
		PmodSwt3.stBtn = PmodSwt3.stCur;
		PmodSwt3.cst = 0;
	}

	// Update the state of PmodSWT4 if necessary.
	if ( cstMaxCnt == PmodSwt4.cst ) {
		PmodSwt4.stBtn = PmodSwt4.stCur;
		PmodSwt4.cst = 0;
	}
    prtLed1Clr	= ( 1 << bnLed1 );
}

void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl5) _IC2_IntHandler(void) // change to 5
{
    // clear interrupt flag for Input Capture 2
    mIC2ClearIntFlag();
    uint16_t bufferData;
    uint16_t diffCapTime;
    prtLed2Set = ( 1 << bnLed2 );
    
    while (IC2CON & (1 << 3))
        bufferData = (uint16_t)IC2BUF;
    
    if (IC2Counter < MAX-1)
        IC2Counter++;
    else{
        IC2Counter = 0;
        //fullRotation++;
    }
    
    // store bufferData into an array
    IC2Arr[IC2Counter] = bufferData;
    
    // upon start of algo, take first element and subtract with last element
    if (IC2Counter == 0)
        diffCapTime = IC2Arr[0] - IC2Arr[MAX - 1];
    else
        diffCapTime = IC2Arr[IC2Counter] - IC2Arr[IC2Counter - 1];
    
    // if time is negative, add max to offset
    if (diffCapTime < 0)
        diffCapTime += 65000;  
    
    // store diffCapTime into an array
    diffCapArr[IC2Counter] = diffCapTime;
    
    // IIR Filter 
    if (diffCapTime > 2000 && diffCapTime < 20000)
        RW_avg_meas = RW_avg_meas * alpha + (float)diffCapTime*(1-alpha);
    else{
        RW_avg_meas = RW_avg_meas;  //essentially, dont update filter 
    }
    
     float secs = RW_avg_meas * 0.000001;
    // X pulse * 1 rev / 156 pulse * 0.75 ft / 1 rev = 0.00480769231*X ft
    // speed = dist/time => ft/s
    rwSpeed = 0.00480769231/secs;
    
/*    if (fullRotation >= 10){  // 157 rev
		OC2R = 0;
        OC2RS = 0;
    } */
    
    prtLed2Clr	= ( 1 << bnLed2 );
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl5) _IC3_IntHandler(void) 
{
// clear interrupt flag for Input Capture 3
    mIC3ClearIntFlag();
    uint16_t bufferData;
    uint16_t diffCapTime;
    prtLed3Set = ( 1 << bnLed3 );
    
    while (IC3CON & (1 << 3))
        bufferData = (uint16_t) IC3BUF; 
    
    if (IC3Counter < MAX-1) // 155
        IC3Counter++;
    else{
        IC3Counter = 0;
        //fullRotation++;
    }

    // store bufferData into an array
    IC3Arr[IC3Counter] = bufferData;
    
    // upon start of algo, take first element and subtract with last element
    if (IC3Counter == 0)
        diffCapTime = IC3Arr[0] - IC3Arr[MAX - 1];
    else
        diffCapTime = IC3Arr[IC3Counter] - IC3Arr[IC3Counter - 1];
    
    // if time is negative, add max to offset
    if (diffCapTime < 0)
        diffCapTime += 65000;        

    // store diffCapTime into an array
    diffCapArr2[IC3Counter] = diffCapTime;   
    
    // IIR Filter 
    if (diffCapTime > 2000 && diffCapTime < 20000){
        LW_avg_meas = LW_avg_meas * alpha + (float)diffCapTime*(1-alpha);
    }
    else{
        LW_avg_meas = LW_avg_meas;  //essentially, don't update filter 
    }
    float secs = LW_avg_meas * 0.000001;
    // X pulse * 1 rev / 156 pulse * 0.75 ft / 1 rev = 0.00480769231*X ft
    // speed = dist/time => ft/s
    lwSpeed = 0.00480769231/secs;
/*    if (fullRotation == 10){ // 157 rev
        OC3R = 0;
        OC3RS = 0;
    } */
    
    prtLed3Clr	= ( 1 << bnLed3 );
}

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/

int main(void) {

	BYTE	stBtn1;
	BYTE	stBtn2;

	BYTE	stPmodBtn1;
	BYTE	stPmodBtn2;
	BYTE	stPmodBtn3;
	BYTE	stPmodBtn4;

	BYTE	stPmodSwt1;
	BYTE	stPmodSwt2;
	BYTE	stPmodSwt3;
	BYTE	stPmodSwt4;

    
	DeviceInit();
	AppInit();
    AdcInit();
    InitLeds(); // checkpoint
        
    //       wheel        kp, ki, kd   , iMin,iMax
    PID_init(&leftWheel,  1, 0.75,0.075);//,0,10000); // 0.75 0.075
    
    PID_init(&rightWheel, 1, 0.75,0.075);//,0,10000);
    
    
	//INTDisableInterrupts();
	DelayMs(500);
	

	//write to PmodCLS
//	SpiEnable();
//	SpiPutBuff(szClearScreen, 3);
//	DelayMs(4);
//	SpiPutBuff(szBacklightOn, 4);
//	DelayMs(4);
//	SpiPutBuff(szCursorOff, 4);
//	DelayMs(4);
//	SpiPutBuff("Hello from", 10);
//	DelayMs(4);
//	SpiPutBuff(szCursorPos, 6);
//	DelayMs(4);
//	SpiPutBuff("Digilent!", 9);
//	DelayMs(2000);
//	SpiDisable();
    
	INTEnableInterrupts();
    char strout[100];
    char strout2[100];
    char ADC0out[100];
    char ADC1out [100];
    char ADC2out[100];
   
    OC2R = 5000; // right wheel
    OC2RS = 5000;
    OC3R = 5000; // left wheel
    OC3RS = 5000;
    
	while (fTrue)
	{	
        RW_speed = 0.00480769231/(RW_time * 0.000001);
        LW_speed = 0.00480769231/(LW_time * 0.000001);
		//INTDisableInterrupts();
       
        SpiEnable();
        SpiPutBuff(szClearScreen, 3);
        DelayMs(4);
        SpiPutBuff(szBacklightOn, 4);
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);
        DelayMs(4);
//        sprintf(strout, "spd=%.1f ft/s", speed);
//        SpiPutBuff(strout, strlen(strout));
        
//        sprintf(strout2, "D=%.3f C=%.3f", RW_speed, rwSpeed);//, RW_avg_meas); Cur=%.1f"
//        SpiPutBuff(strout2, strlen(strout2));
        
        sprintf(ADC0out, "0=%.2f 1=%.2f", ADCValue0, ADCValue1);
        //sprintf(ADC0out, "A0=%.2f", ADCValue0);
        SpiPutBuff(ADC0out, strlen(ADC0out));
        DelayMs(4);
        SpiPutBuff(szCursorPos, 6);
        DelayMs(4);
        sprintf(ADC2out,"2=%i :%.1f %.1f",mode, lwSpeed, rwSpeed);
        //sprintf(ADC2out, "A2=%.2f",ADCValue2);
        SpiPutBuff(ADC2out, strlen(ADC2out));
        //sprintf(strout, "d=%.3f c=%.3f", LW_speed, lwSpeed);//, RW_avg_meas); Cur=%.1f"
        //SpiPutBuff(strout, strlen(strout));
        DelayMs(250);
        SpiDisable();
	
		//get data here
		stBtn1 = btnBtn1.stBtn;
		stBtn2 = btnBtn2.stBtn;

		stPmodBtn1 = PmodBtn1.stBtn;
		stPmodBtn2 = PmodBtn2.stBtn;
		stPmodBtn3 = PmodBtn3.stBtn;
		stPmodBtn4 = PmodBtn4.stBtn;

		stPmodSwt1 = PmodSwt1.stBtn;
		stPmodSwt2 = PmodSwt2.stBtn;
		stPmodSwt3 = PmodSwt3.stBtn;
		stPmodSwt4 = PmodSwt4.stBtn;

		INTEnableInterrupts();
		//configure OCR to go forward
        
        if (stBtn1 == stPressed){
            if (mode == 0)
                mode = 1;
                // run wall following mode
            else if (mode == 1)
                mode = 0;
                // run puppy dog mode
        }
/*      
		if(stPressed == stPmodBtn1){
			//start motor if button 2 pressed

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
		
		}else if(stPressed == stPmodBtn2){
			//start left turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlBwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn3){
			//start right turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodBtn4){
			//start move backward

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlLeft();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt1){
			//make square to right

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();		// first turn
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();     // second turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();		// third turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt2){
			//make triangle to left

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00); //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();  	//first turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//second turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//third turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
		
		}else if(stPressed == stPmodSwt3){
			// Three point turn around

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdRight();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlBwdLeft();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();

		}else if(stPressed == stPmodSwt4){
			// dance
			
			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdLeft(); // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();		
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdLeft();  // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlLeft();     // spin
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
		}  //end if  
*/
	}  //end while
}  //end main

/* ------------------------------------------------------------ */
/***	DeviceInit
**
**	Synopsis:
**		DeviceInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes on chip peripheral devices to the default
**		state.
*/

void DeviceInit() {

	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward
	
	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward

	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= (1 << 3) | ( 1 << 2 ) | ( 1 << 1 );	// pwm set up, Using timer 3
	OC2R	= dtcMtrStopped;
	OC2RS	= dtcMtrStopped;

	// Configure Output Compare 3.
	OC3CON = ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm, Using timer 3
	OC3R	= dtcMtrStopped;
	OC3RS	= dtcMtrStopped;
    //////////////////////////////////////////////////////
    // Interrupt Priority Control Register 2 // table 7-1 pic32mx
    IPC2SET = (1 << 12) | (1 << 10); // priority 5 - sub 0
    IFS0CLR = (1 << 9);
    IEC0SET	= (1 << 9);
    
    // 15 > enable cap, 9 > capture rise first, 
    // 7 > Timer2 counter src for cap, 1 & 0 > rising edge
    IC2CON = (1 << 15) | (1 << 9) | (1 << 7) | (1 << 1) | (1 << 0);  
    
    // Interrupt Priority Control Register 3 // table 7-1 pic32mx
    IPC3SET = (1 << 12) | (1 << 10); // priority 5 - sub 0
    IFS0CLR = (1 << 13);
    IEC0SET	= (1 << 13);
    
    // 15 > enable cap, 9 > capture rise first,
    // 7 > Timer2 counter src for cap, 1 & 0 > rising edge
    IC3CON = (1 << 15) | (1 << 9) | (1 << 7)  | (1 << 1) | (1 << 0);
    
    ////////////////////////////////////////////////////////
	// Configure Timer 2.
	TMR2	= 0;									// clear timer 2 count
	PR2		= 64999; // used for input cap

	// Configure Timer 3.
	TMR3	= 0;
	PR3		= 9999; // used for output compare

	// Start timers and output compare units.
	T2CON		= ( 1 << 15 ) | (1 << TCKPS21) | ( 1 << TCKPS20 );	// timer 2 prescale = 8 , 4 5 6
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
	T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30); 	// timer 3 prescale = 8

	// Configure Timer 5.
	TMR5	= 0;
	PR5		= 999; // period match every 1000 us , was 99 -> 100 us // 9999-> 10ms
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 7, sub 3
	IFS0CLR = ( 1 << 20 );
	IEC0SET	= ( 1 << 20 );
	
	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8
       
	//enable SPI
	SpiInit();

	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
    //
}

/* ------------------------------------------------------------ */
/***	AppInit
**
**	Synopsis:
**		AppInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs application specific initialization. Sets devices
**		and global variables for application.
*/

void AppInit() {  
    int c = 0;
    for (; c<MAX; c++)
    {
        IC2Arr[c] = 0;
        IC3Arr[c] = 0;
        diffCapArr[c] = 0;
        diffCapArr2[c] = 0;
    }
}

void    AdcInit(void){
//	CONFIGURE ADC
	AD1PCFG	= 0XFFF8;
	// CONFIGURE AN0, AN1, and AN2 AS ANALOG INPUTS
	AD1CON1	= 0X00E4;   
	// BIT 7-5 SSRC 111 = INTERNAL COUNTER ENDS SAMPLING AND STARTS CONVERSION
	// BIT 4 CLRASM 0 = Normal Operation, buffer contents will be overwritten by the next conversion sequence
	// BIT 2 ASAM 1 = SAMPLING BEGINS immediately after conversion completes; SAMP bit is automatically set
	// BIT 1 SAMP 0 = ADC IS NOT SAMPLING
	// BUT 0 DONE 0 = STATUS BIT
	AD1CON2	= 0X0408; // 0000 0100 0000 1000     				
	// BIT 10 CSCNA 1 = SCAN INPUTS
	// BIT 2-3 SMPL 1-1 = ONE INTERRUPT AFTER EVERY THIRD CONVERSION
	AD1CON3	= 0X1FFF;
	// BIT 15 ADRC 0 = CLOCK DERIVED FROM PERIPHERAL BUS CLOCK
	// SAMC AND ADCS - I NEED TO READ MORE ABOUT TIMING TO UNDERSTAND THE FUNCTION OF THESE TWO VARIABLES
	AD1CHS	= 0X00000000;   // 32 bit SFR  
	AD1CSSL	= 0X0007;  
	// CSSL = SCAN CHANNELS 2,1 and 0
	IPC6SET	= ( 1 << 27 ) | ( 1 << 26 ); // ADC interrupt priority level 3, sub 0
	IFS1CLR	= 2;    // CLEAR ADC INTERRUPT FLAG
	IEC1SET = 2;	// ENABLE ADC INTERRUPT
	AD1CON1SET = 0X8000;	// 	TURN ADC ON
}

/* ------------------------------------------------------------ */
/***	Wait_ms
**
**	Synopsis:
**		Wait_ms(WORD)
**
**	Parameters:
**		WORD (range from 0 to 65535)
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Will wait for specified number of milliseconds.  Using a 
**		word variable allows for delays up to 65.535 seconds.  The value
**		in the for statement may need to be altered depending on how your
**		compiler translates this code into AVR assembly.  In assembly, it is
**		possible to generate exact delay values by counting the number of clock
**		cycles your loop takes to execute.  When writing code in C, the compiler
**		interprets the code, and translates it into assembly.  This process is 
**		notoriously inefficient and may vary between different versions of AVR Studio
**		and WinAVR GCC.  A handy method of calibrating the delay loop is to write a 
**		short program that toggles an LED on and off once per second using this 
**		function and using a watch to time how long it is actually taking to
**		complete. 
**
*/

void Wait_ms(WORD delay) {

	WORD i;

	while(delay > 0){

		for( i = 0; i < 375; i ++){ // 375
			;;
		}
		delay -= 1;
	}
}

void InitLeds( void )
{
	// Configure LEDs as digital outputs.
	trisLed1Clr = ( 1 << bnLed1 );
	trisLed2Clr = ( 1 << bnLed2 );
	trisLed3Clr = ( 1 << bnLed3 );
	trisLed4Clr = ( 1 << bnLed4 );
	
	// Turn off the LEDs.
	prtLed1Clr	= ( 1 << bnLed1 );
	prtLed2Clr	= ( 1 << bnLed2 );
	prtLed3Clr	= ( 1 << bnLed3 );
	prtLed4Clr	= ( 1 << bnLed4 );
}

void SetLeds( BYTE stLeds ) {
	// Set the state of Led1.
	if ( stLeds & ( 1 << 0 ) ) {
		prtLed1Set = ( 1 << bnLed1 );
	}
	else {
		prtLed1Clr = ( 1 << bnLed1 );
	}
	
	// Set the state of Led2.
	if ( stLeds & ( 1 << 1 ) ) {
		prtLed2Set = ( 1 << bnLed2 );
	}
	else {
		prtLed2Clr = ( 1 << bnLed2 );
	}
	
	// Set the state of Led3.
	if ( stLeds & ( 1 << 2 ) ) {
		prtLed3Set = ( 1 << bnLed3 );
	}
	else {
		prtLed3Clr = ( 1 << bnLed3 );
	}
	
	// Set the state of Led4.
	if ( stLeds & ( 1 << 3 ) ) {
		prtLed4Set = ( 1 << bnLed4 );
	}
	else {
		prtLed4Clr = ( 1 << bnLed4 );
	}

}

//static int GetLSB(int intValue)
//{
//    return (intValue & 0x0000FFFF);
//}
//static int GetMSB(int intValue)
//{
//    return (intValue & 0xFFFF0000);
//}

/************************************************************************/