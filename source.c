/*
 * File:   source.c
 * Author: Tevis Dryden
 *
 * Created on October 28, 2017, 2:09 PM
 */


#include "xc.h"
#pragma config FNOSC = FRC
#define FCY 4000000
#include <libpic30.h>

//

void driveStop() {
    OC1R = 0;
}

void driveForwards() {
    OC1R = 4000;
    _LATB1 = 1;
    _LATA1 = 0;
}

void driveBackwards() {
    OC1R = 4000;
    _LATB1 = 0;
    _LATA1 = 1;
}

int count=0;
double rev=0;
static int countToRevConvsersion = 20 * 8; //1/8 step

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    
 
    // Remember to clear the Timer1 interrupt flag when
    // this ISR is entered.
    // PLACE CODE TO CLEAR THE TIMER1 INTERRUPT FLAG HERE
    _OC2IF = 0;
    
    
    count++;
    if (count == countToRevConvsersion) {
        count = 0;
        rev+=.1;
    }
    
    if (rev <= 3) {
        driveForwards();
    } else if (rev <= 3.2) {
        driveStop();
    } else if (rev <= 6.2) {
        driveBackwards();
    } else if (rev <= 6.4) {
        driveStop();
    }  else {
        rev = 0;
    }
    
    // Place in this ISR whatever code should be executed
    // when the timer reaches the period (PR1) that you
    // specify
    // PLACE CUSTOM CODE HERE
   
    
    
}

int main(void) {
    
    /*** Configure Desired Port Pins as Analog Inputs ***/
    // TRIS 1 = Input
    // TRIS 0 = Output
    // ANS 1 = Analog
    // ANS 0 = Digital
	_TRISB2 = 0;		// TRISA/B, pg. 45 datasheet
	_ANSB2 = 0;			// ANSA/B, pg. 136-137
    _TRISA0 = 1;		// TRISA/B, pg. 45 datasheet
	_ANSA0 = 0;			// ANSA/B, pg. 136-137
    
    
    /* Wheels */
    // ON OFF
    _TRISA2 = 0; // P7
    _TRISA2 = 0;
    
    // Wheel PWM
    _TRISB0 = 0; // P4
    _ANSB0 = 0;
    
    // Left Wheel
    _TRISB1 = 0; // P5		
	_ANSB1 = 0;
    
    // Right Wheel
    _TRISA1 = 0; // P3
    _ANSA1 = 0;
    
    
	/*** Select Voltage Reference Source ***/
	// use AVdd for positive reference
	_PVCFG = 00;		// AD1CON2<15:14>, pg. 212-213 datasheet
	// use AVss for negative reference
	_NVCFG = 0;			// AD1CON2<13>


	/*** Select Analog Conversion Clock Rate ***/
	// make sure Tad is at least 600ns, see Table 29-41 datasheet
	_ADCS = 3;	// AD1CON3<7:0>, pg. 213 datasheet


	/*** Select Sample/Conversion Sequence ***/
	// use auto-convert
	_SSRC = 0b0111;		// AD1CON1<7:4>, pg. 211 datasheet
	// use auto-sample
	_ASAM = 1;			// AD1CON1<2>
	// choose a sample time >= 1 Tad, see Table 29-41 datasheet
	_SAMC = 1;		// AD1CON3<12:8>


	/*** Choose Analog Channels to be Used ***/
	// scan inputs
	_CSCNA = 1;			// AD1CON2<10>
	// choose which channels to scan, e.g. for ch AN12, set _CSS12 = 1;
	_CSS4 = 1;			// AD1CSSH/L, pg. 217
    _CSS0 = 1;

	/*** Select How Results are Presented in Buffer ***/
	// set 12-bit resolution
	_MODE12 = 1;		// AD1CON1<10>
	// use absolute decimal format
	_FORM = 0;			// AD1CON1<9:8>
	// load results into buffer determined by converted channel, e.g. ch AN12 
    // results appear in ADC1BUF12
	_BUFREGEN = 1;		// AD1CON2<11>


	/*** Select Interrupt Rate ***/
	// interrupt rate should reflect number of analog channels used, e.g. if 
    // 5 channels, interrupt every 5th sample
	_SMPI = 1;		// AD1CON2<6:2>


	/*** Turn on A/D Module ***/
	_ADON = 1;			// AD1CON1<15>
    
    OC2CON1 = 0;
    OC2CON2 = 0;
    
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC1R = 4000;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    
                                // Tpwm = (OC1RS + 1) * Tcy * PRESCALE
    
    OC1RS = 7999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F;
    
    OC2R = 500;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = 4000/1;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC2CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC2CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    OC1CON1bits.OCM = 0b110;
    OC1CON2bits.OCTRIG = 0;
    _OC2IP=4; // Set Interrupt Priority
    _OC2IE=1; // Enable OC2 Interrupt
   _OC2IF=0; // turns flag off
    
    //-----------------------------------------------------------
    // RUN

    // Wait and let the PWM do its job behind the scenes
    while(1)
    {
        _RA2 = _RA0;
    }
    
   
    return 0;
}