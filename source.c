/*
 * File:   source.c
 * Author: Tevis Dryden
 *
 * Created on October 28, 2017, 2:09 PM
 */


#include "xc.h"
#pragma config FNOSC = FRCDIV
#define FCY 1000000
#include <libpic30.h>
#pragma config OSCIOFNC = OFF
#pragma config ICS = PGx3

void driveStop() {
    OC2R = 0;
}

void driveForwards() {
    OC2R = 100;
    _LATB2 = 1;
    _LATA1 = 0;
}

void driveBackwards() {
    OC2R = 100;
    _LATB2 = 0;
    _LATA1 = 1;
}

void turnRight() {
    OC2R = 100;
    _LATB2 = 1;
    _LATA1 = 1;
}



int count=0;
double rev=0;
static int countToRevConvsersion = 2 * 8; //1/8 step

enum setUpStates {DELAY, DRIVEOUT, BRIDGING, DRIVEBACK, CHANGESTATE} setUpPosition;
enum states {FINDINGCORNER, SETTINGUP, SHOOTING} state;

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    
 
    // Remember to clear the Timer1 interrupt flag when
    // this ISR is entered.
    // PLACE CODE TO CLEAR THE TIMER1 INTERRUPT FLAG HERE
    _OC2IF = 0;
    
    count++;
    // Initial Delay
    if (setUpPosition == DELAY) {
        if(count == 100) {
            count = 0;
            _RA2 = 1;
            setUpPosition++;
        }
    } else if ((setUpPosition == DRIVEOUT)||(setUpPosition == DRIVEBACK)) {
        // Conversion
        if (count == countToRevConvsersion) {
            count = 0;
            rev+=.01;
        }
        
        if(setUpPosition == DRIVEOUT) {
            if(rev <= 2.3) {
                driveForwards();
            } else {
                rev = 0;
                _RA2 = 0;
                _OC2IE = 0;
                setUpPosition++;
            }
        } else if(setUpPosition == DRIVEBACK) {
            if(rev <= .9) {
                driveBackwards();
            } else {
                rev = 0;
                _RA2 = 0;
                _OC2IE = 0;
                setUpPosition++;
            }
        }
    }
    
    
    /* This is the code for the turning video!
    if (count == countToRevConvsersion) {
        count = 0;
        rev+=.01;
    }
    
    if (rev <= 1) {
        driveForwards();
    } else if (rev <= 1.73) {
        turnRight();
    } else if (rev <= 3) {
        driveForwards();
    } else if (rev <= 3.73) {
        turnRight();
    } else {
        rev = 0;
    }
     */
    
//    if (rev <= 3.43) {
//        driveForwards();
//    } else if (rev <= 4) {
//        turnRight();
//    } else if (rev <= 6) {
//        driveForwards();
//    } else if (rev <= 7) {
//        driveStop();
//    }  else {
//        rev = 0;
//    }
    
    // Place in this ISR whatever code should be executed
    // when the timer reaches the period (PR1) that you
    // specify
    // PLACE CUSTOM CODE HERE
   
    
    
}

int side = 0;
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    // Clear Timer1 interrupt flag so that the program doesn't
    // just jump back to this function when it returns to the
    // while(1) loop.
    _T1IF = 0;
    if (side) {
        OC1R = 10500/4;
        side = 0;
    } else {
        OC1R = 9500/4;
        side = 1;
    }
    // Change state of pin 14 (RA6)
    
}
int shooterDebouncerMax = 10000;
int shooterDebouncer = 0;

int blackBall = 0;
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    _T2IF = 0;
    blackBall = 0;
    _T2IE = 0;
    shooterDebouncer = shooterDebouncerMax;
    
}

void _ISR _ADC1Interrupt(void) {
    if(ADC1BUF9 < 1365) {  
        blackBall = 1;
        TMR2 = 0;
        _T2IE = 1;
        shooterDebouncer = shooterDebouncerMax;
    }
}

int main(void) {
    _RCDIV = 0b010;
    shooterDebouncer = shooterDebouncerMax;
    
    /*** Configure Desired Port Pins as Analog Inputs ***/
    // TRIS 1 = Input
    // TRIS 0 = Output
    // ANS 1 = Analog
    // ANS 0 = Digital
    _TRISB2 = 0;        // TRISA/B, pg. 45 datasheet
    _ANSB2 = 0;         // ANSA/B, pg. 136-137
    _TRISA0 = 1;        // TRISA/B, pg. 45 datasheet
    _ANSA0 = 0;         // ANSA/B, pg. 136-137
    
    
    /* Wheels */
    // ON OFF
    _TRISA2 = 0; // P7
    _TRISA2 = 0;
    
    // Wheel PWM
    _TRISB0 = 0; // P4
    _ANSB0 = 0;
    
    // Left Wheel
    _TRISB2 = 0; // P5      
    _ANSB2 = 0;
    
    // Right Wheel
    _TRISA1 = 0; // P3
    _ANSA1 = 0;
    
    // Goal IR Sensor
    _TRISA3 = 1; // P8
    _ANSA3 = 1;
    
    // Bumper Sensors
    _TRISB9 = 1; // P13
    
    // Ball Shooter
    _TRISB8 = 0; // P11
    _LATB8 = 0; // Start Off
    
    // Left IR Sensor
    _TRISB12 = 1; // P15
    _ANSB12 = 1;
    
    // Center IR Sensor
    _TRISB13 = 1; // P16
    _ANSB13 = 1;
    
    // Right IR Sensor
    _TRISB14 = 1; // P17
    _ANSB14 = 1;
    
    // Black Ball Sensor
    _TRISB15 = 1; // P18
    _ANSB15 = 1;
    
    /*** Select Voltage Reference Source ***/
    // use AVdd for positive reference
    _PVCFG = 00;        // AD1CON2<15:14>, pg. 212-213 datasheet
    // use AVss for negative reference
    _NVCFG = 0;         // AD1CON2<13>


    /*** Select Analog Conversion Clock Rate ***/
    // make sure Tad is at least 600ns, see Table 29-41 datasheet
    _ADCS = 3;  // AD1CON3<7:0>, pg. 213 datasheet


    /*** Select Sample/Conversion Sequence ***/
    // use auto-convert
    _SSRC = 0b0111;     // AD1CON1<7:4>, pg. 211 datasheet
    // use auto-sample
    _ASAM = 1;          // AD1CON1<2>
    // choose a sample time >= 1 Tad, see Table 29-41 datasheet
    _SAMC = 1;      // AD1CON3<12:8>


    /*** Choose Analog Channels to be Used ***/
    // scan inputs
    _CSCNA = 1;         // AD1CON2<10>
    // choose which channels to scan, e.g. for ch AN12, set _CSS12 = 1;
    _CSS14 = 1;          // AD1CSSH/L, pg. 217
    

    /*** Select How Results are Presented in Buffer ***/
    // set 12-bit resolution
    _MODE12 = 1;        // AD1CON1<10>
    // use absolute decimal format
    _FORM = 0;          // AD1CON1<9:8>
    // load results into buffer determined by converted channel, e.g. ch AN12 
    // results appear in ADC1BUF12
    _BUFREGEN = 1;      // AD1CON2<11>


    /*** Select Interrupt Rate ***/
    // interrupt rate should reflect number of analog channels used, e.g. if 
    // 5 channels, interrupt every 5th sample
    _SMPI = 0;      // AD1CON2<6:2>


    /*** Turn on A/D Module ***/
    _ADON = 1;          // AD1CON1<15>
    
    OC2CON1 = 0;
    OC2CON2 = 0;
    
    OC1CON1 = 0;
    OC1CON2 = 0;
    
    OC3CON1 = 0;
    OC3CON2 = 0;
    
    
    // Occelators 
    int hz50 = 20000; 
    
    OC3R = hz50 * .123;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    
                                // Tpwm = (OC1RS + 1) * Tcy * PRESCALE
    
    OC3RS = hz50 - 1;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC3CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC3CON2bits.SYNCSEL = 0x1F;
    
    OC3CON1bits.OCM = 0b110;
    OC3CON2bits.OCTRIG = 0;
    
    OC1R = 3000/4;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    
                                // Tpwm = (OC1RS + 1) * Tcy * PRESCALE
    
    OC1RS = hz50 - 1;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F;
    
    OC2R = 1000/4;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = 999;               // Period of OC1 to achieve desired PWM 
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
    _OC2IE=0; // Enable OC2 Interrupt
    _OC2IF=0; // turns flag off
   
   
   // Timer for interrupting laser!!!!!
   _TON = 1;           // Turn on Timer1
   _TCS = 0;           // internal clock
   _TCKPS = 0b11;      // 256 prescale
   
   _T1IP = 4;          // Select Timer1 interrupt priority
   _T1IE = 0;          // Enable Timer1 interrupt
   _T1IF = 0;          // Clear Timer1 interrupt flag
   PR1 = 15625/2;
   TMR1 = 0;
   
   // Timer for black ball!!!!!
   T2CONbits.TON = 1;           // Turn off Timer2
   T2CONbits.TCS = 0;           // internal clock
   T2CONbits.T32 = 1;           // Make clock 32 bit
   T2CONbits.TCKPS = 0b11;      // 256 prescale
   
   _T2IP = 7;          // Select Timer2 interrupt priority
   _T2IE = 0;          // Enable Timer2 interrupt
   _T2IF = 0;          // Clear Timer2 interrupt flag
   PR2 = 15625;
   TMR2 = 0;
   
    
    //-----------------------------------------------------------
    // RUN

    // Wait and let the PWM do its job behind the scenes
   int x = 0;
   int faults = 0;
   int going = 0;
   
   int bumperDebouncer = 0;
   
   state = FINDINGCORNER;
   
   int leftGoalPWM = hz50 * .035;
   int centerGoalPWM = hz50 * .07;
   int rightGoalPWM = hz50 * .105;
   
   
    while(1)
    {
        if (state == FINDINGCORNER) {
            _RA2 = _RA0;
            if((ADC1BUF14 > 1365)&& x == 100) {
                driveBackwards();
                //driveForwards();
                going = 1;
            } else if(ADC1BUF14 > 1217) {
                x++;
            } else if((faults < 1000)&&(going == 1)) {
                faults++;
             }else {
                turnRight();
                x = 0;
                faults = 0;
                going = 0;
            }
            
            //TODO Bumpers to send it into next state.
            
            if (_RB9) {
                if(bumperDebouncer < 1000) {
                    bumperDebouncer++;
                } else {
                    state++;
                    setUpPosition = DELAY;

                    //driveForwards();

                    _OC2IE = 1; // Start Counting
                    _RA2 = 0;
                    bumperDebouncer = 0;
                }
            }
        } else if (state == SETTINGUP) {
            if (setUpPosition == BRIDGING) {
                OC1R = 10500/4;
                //TODO Turn bridge
                
                
                setUpPosition++;
                _RA2 = 1;
                _OC2IE = 1;
            } else if (setUpPosition == CHANGESTATE) {
                state++;
            
                _T1IE = 1;
                TMR1 = 0;
                
                // Turn off Goal IR Sensor
                _CSS14 = 0;
                
                // Start checking the IR Sensors
                _SMPI = 3;
                _CSS12 = 1; // Left Sensor P15
                _CSS11 = 1; // Center Sensor P16
                _CSS10 = 1; // Right Sensor P17
                _CSS9 = 1; // Black Ball Sensor P18
                
//                // Turn on interrupt
//                _AD1IE = 1;
//                _AD1IP = 7;
//                _AD1IF = 0;
//                
//                // Turn on shooter
//                _LATB8 = 1;
            }
        } else if (state == SHOOTING) {
            if (shooterDebouncer >= shooterDebouncerMax) {
                shooterDebouncer = 0;
                if(blackBall) {
                    //PWM
                    if(OC3R <= centerGoalPWM) {
                        OC3R = (centerGoalPWM + leftGoalPWM)/2;
                    } else {
                        OC3R = (centerGoalPWM + rightGoalPWM)/2;
                    }
                } else if(ADC1BUF11 > 1365) {
                    //PWM
                    OC3R = centerGoalPWM;
                } else if(ADC1BUF12 > 1365) {
                    //PWM
                    OC3R = leftGoalPWM;
                } else if(ADC1BUF10 > 1365) {
                    //PWM
                    OC3R = rightGoalPWM;
                 }else {
//                    OC3R = hz50 * .05;
                }
            } else {
                shooterDebouncer++;
            } 
        }

    }
    
   
    return 0;
}