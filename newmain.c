/*
 * File:   newmain.c
 * Author: dkg
 * PIC12f683 - Parking Alarm
 * Sensing range - 4cm - 4m (400cm)
 * Created on March 11, 2017, 6:02 PM
 */

// PIC Configuration.
#pragma config FOSC = INTOSCIO // Oscillator Selection bits (INTOSC oscillator: //CLKOUT function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF     // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Detect (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#define _XTAL_FREQ 4000000

// Pin declarations.
#define US_TRIG 0x20 //GPIO 5
#define T1GP 0x10 // GPIO 4
#define LED 0x1 // GP 0

// Functions.
#define PWM_ON CCP1CON=CCP1CON |  0b11111100
#define PWM_OFF CCP1CON=CCP1CON & 0b11110000
#define ON(x) GPIO |= x
#define OFF(x) GPIO &= ~x

// Other Definitions.
#define MAX_RANGE 80 // Max Range to detect objects.

#include <xc.h>

// Function Declarations.
void init(void);
unsigned int GetElapsedTime(void);
void ResetTimer(void);
void ultrasonicTask(void);
void blinkLEDTask(void);
void ADCTask(void);

// Some global variables.
unsigned int Dist, BlinkFreq = 0;
unsigned long Adc = 0;
bit Blink = 0;

void main(void) {
    unsigned long alertDist = 0;

    init();
    PWM_OFF;
    ei(); //enable global interrupts

    while (1) {
        ultrasonicTask();
        blinkLEDTask();
        ADCTask();

        // adc range 30 - 1000 (alertDist: 11 cm - 390 cm)
        alertDist = 400 * Adc / 1024;
        int delta = Dist - alertDist;

        if (delta <= 0) {
            Blink = 1;
            BlinkFreq = 10;
            PWM_ON;
            continue;
        } else if (delta > 0 && delta <= 30) {
            Blink = 1;
            BlinkFreq = 100;
            PWM_OFF;
            continue;
        } else if (delta > 30 && delta <= 60) {
            Blink = 1;
            BlinkFreq = 400;
            PWM_OFF;
            continue;
        } else if (delta > 60 && delta < 90) {
            Blink = 1;
            BlinkFreq = 1200;
            PWM_OFF;
            continue;
        }
        PWM_OFF;
        Blink = 0;
    }
    return;
}

void init(void) {
    // Basic Pic initialization.
    OSCCON = 0x70;
    GPIO = 0x00;
    ANSEL = 0x00;
    TRISIO = 0x00; // Setup GPIO as output.
    ADCON0 = 0x00; // Disable AD.
    CMCON0 = 0x7;
    WPU = 0x00;

    // Initialize Timer 1 in Gating Mode to calculate pulse width.
    TRISIO |= T1GP;
    T1CON = 0b11010001;
    T1GSS = 1; //Set Gate Source as T1g
    TMR1L = 0;
    TMR1H = 0;

    // Setup PWM to generate 2441 Hz for Buzzer on GP5.   
    PR2 = 0b11001100;
    T2CON = 0b00000101;
    CCPR1L = 0b10100011;
    CCP1CON = 0b00111100;

    // Setup ADC to read distance setting from GP1/AN1.
    TRISIO = TRISIO | 0x2;
    ANSEL = 0b00010010; // 001 - Fosc/8, 0010 - AN1 active. 
    ADCON0 = 0b10000101; // 1 Right Justified, Vdd ref, AN0 channel, A/D On.

    // Configure Timer 0 to interrupt every 1 ms
    TMR0 = 0x06;
    OPTION_REG = 0b11000010; // Prescalar 1:8
    T0IE = 1;

}

void ADCTask(void) {
    static int start = 0;

    static enum {
        START = 0,
        WAIT,
        START_ADC,
        WAIT_CONVERSION,
        GET_RESULT,
    } state = START;

    switch (state) {
        case START:
            start = GetElapsedTime();
            state = WAIT;

            // Delay each ADC conversion.
        case WAIT:
            if ((GetElapsedTime() - start) < 50) {
                return;
            }
            start = 0;
            state = START_ADC;

        case START_ADC:
            ADCON0 = 0b10000101;
            ADCON0 |= 0x2;
            state = WAIT_CONVERSION;

        case WAIT_CONVERSION:
            if (ADCON0 & 0x02) {
                return;
            }
            state = GET_RESULT;

        case GET_RESULT:
            Adc = ADRESH;
            Adc = (Adc << 8) | ADRESL;
            state = START;
    }
}

void blinkLEDTask(void) {
    static int last = 0;

    if (!Blink) {
        OFF(LED);
        return;
    }

    int now = GetElapsedTime();
    if (now - last >= BlinkFreq) {
        GPIO ^= LED;
        last = now;
    }
}

void ultrasonicTask(void) {
    unsigned int pulse_width = 0;
    static int start,wait = 0;

    static enum {
        START = 0,
        WAIT,
        SEND_TRIGGER_PULSE,
        WAIT_LINE_HIGH,
        WAIT_LINE_LOW,
        GET_RESULT,
    } state = START;

    switch (state) {

        case START:
            start = GetElapsedTime();
            state = WAIT;

            // Wait for some time between pulses to allow ultrasonic sensor to reset.    
        case WAIT:
            if ((GetElapsedTime() - start) < 400) {
                return;
            }
            start = 0;
            state = SEND_TRIGGER_PULSE;

        case SEND_TRIGGER_PULSE:
            ON(US_TRIG);
            __delay_us(10);
            OFF(US_TRIG);
            wait = GetElapsedTime();
            state = WAIT_LINE_HIGH;

        case WAIT_LINE_HIGH:
            if (!(GPIO & T1GP) && (GetElapsedTime() - wait) < 100) {
                return;
            }
            wait = 0;
            state = WAIT_LINE_LOW;

        case WAIT_LINE_LOW:
            if (GPIO & T1GP) {
                return;
            }
            state = GET_RESULT;

        case GET_RESULT:
            pulse_width = TMR1H;
            pulse_width = (pulse_width << 8) | TMR1L;
            TMR1L = 0;
            TMR1H = 0;
            Dist = pulse_width / (int) 58.82; // Distance in cm.
            state = START;
    }
}

// Interrupt Service Routine.
unsigned int ms_cnt = 0;

void interrupt isr(void) {
    if (T0IE && T0IF) {
        T0IF = 0;
        TMR0 = 0x06;
        ms_cnt++;
    }
}

/************** Timer 0 Functions ************/

unsigned int GetElapsedTime(void) {
    static unsigned int time;
    di();
    time = ms_cnt;
    ei();
    return time;
}

void ResetTimer(void) {
    di();
    ms_cnt = 0;
    ei();
}
