//******************************************************************************
//   MSP430F54x - SIMPLE MOTION AND SOUND SENSOR
//
//   Description: Simple motion and sound sensor using the PIR sensor and external
//   mic to trigger LEDs. Once triggered, the MCU will display a message via UART to 
//   PUTTY console. 
//   
//   
//   
//
//                MSP430F5438
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//     MIC -->|P6.7/A7      P4.0|--> GREEN LED
//     PIR -->|P1.4         P4.1|--> RED LED       
//            |             P4.2|--> BLUE LED

//   
//*****************************************************************************


#include "msp430.h"
#include <stdint.h>
#include "Uart.h"
#include "string.h"
#include "stdio.h"

//define messages
#define MSG "HELLO!"
#define MSG_motion "MOTION DETECTED!"
#define MSG_sound "SOUND DETECTED!"
#define MSG_NEWLINE "\r\n"
#define APP_TX_DATA_SIZE 32

//define LEDs , MIC, and PIR
#define GREEN BIT0
#define RED BIT1
#define BLUE BIT2
#define PIR BIT4
#define MIC BIT7

#define DELAY_05s 62500

unsigned char counter = 0;
unsigned int adcvalue = 0;
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];

//function prototypes
void UART_sendString(uint8_t* string_buf);
void configADC();

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;      // stop watch dog timer
    
    P4DIR = RED + GREEN + BLUE;     //Set LEDs to OUT
    P4DS |= RED + GREEN + BLUE;      // set
    P4OUT |= GREEN;                  //GREEN Always on indicate power

    //Pin 1.4 as input for PIR
    P1REN |= PIR;      //ENABLE PULL UP/DOWN RESISTORS
    P1OUT &= ~PIR;
    P1IE |= PIR;       //ENABLE INTERRUPT
    P1IES &= ~PIR;      //LO/HI EDGE
    P1IFG &= ~PIR;     //IFG CLEARED

    TA1CCR0 = DELAY_05s;
    TA1CTL = TASSEL_2 + MC_1 + ID_3;   // SMCLK, upmode , /8
    TA1CCTL0 = CCIE;

    configADC();    //configuring ADC12
    AppUart_init(); //Initialize UART

    //sending first msg via uart
    UART_sendString((uint8_t *) MSG_NEWLINE);
    UART_sendString((uint8_t *) MSG);
    UART_sendString((uint8_t *) MSG_NEWLINE);

    TBCTL |= MC_1;

    __bis_SR_register(LPM4_bits + GIE);  //Enter low power mode and enable global interrupt


    ADC12CTL0 |= ADC12SC + ADC12ENC; //enable ADC and start conversion

}

// Timer A interrupt service routine
// Interrupt service for simple counter, count up to 5 seconds when motion is detected.
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A_ISR(void)
{
    if(counter == 10) {
        counter = 0;
        P4OUT &= ~RED; //LED OFF
        __bis_SR_register_on_exit(LPM4_bits); //exit LPM4
    } else {
        counter++;
    }
}

// Port 1 interrupt service routine
//Interrupt service for the motion sensor at PIN 1.4
//Once trigger, send message to Putty via UART.
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void)

{
        if(P1IFG & PIR){
            P1IFG &= ~PIR;// IFG cleared
            P4OUT |= RED;      //turn LED on
            //send message
            UART_sendString((uint8_t *) MSG_NEWLINE);
            UART_sendString((uint8_t *) MSG_motion);
            UART_sendString((uint8_t *) MSG_NEWLINE);

            counter = 0;    //reset counter
        }
        __bic_SR_register_on_exit(LPM4_bits - LPM0_bits); // switch from LPM4 to LPM0


}

// ADC12 interrupt service routine
//Once trigger, send message to Putty via UART.
// MIC is connected to PIN 6.7 which is also ADC A7 channel.
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
            adcvalue = ADC12MEM0;      //Move results, iFG is cleared

                     if(adcvalue > 3000){
                         P4OUT |= BLUE;
                         //sending message
                         UART_sendString((uint8_t *) MSG_NEWLINE);
                         UART_sendString((uint8_t *) MSG_sound);
                         UART_sendString((uint8_t *) MSG_NEWLINE);
                     }
                     __bis_SR_register_on_exit(LPM4_bits); //exit LPM4
                     //sprintf((char*) adcvalue, "ADC value: %0.5f\n\n", Analog_in*(3.6/4096));
                     //halLcdPrintLine(adcvalue, 5, 4);
}

//Function to send string via UART, take string as parameter
void UART_sendString(uint8_t* string_buf)
{
    uint32_t i = 0;
    while (*(string_buf + i)){
        AppUart_putChar(*(string_buf +i));
        i++;
    }
}
//config ADC, sampling repeated on single channel using timer B as trigger.
//Max adcvalue = 4096, sampling at 3000, since any lower would be too sensitive.
void configADC(){
    P6OUT &= ~MIC;       
    P6SEL |= MIC;           //Senable A/D chanel A7

    ADC12CTL0 &= ~ADC12ENC; 
    ADC12CTL0 = ADC12SHT02 + ADC12ON; //sampling time, adc12 on
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_2 + ADC12SSEL_2 + ADC12SHS_3; //sampling timer, repeated sampling on single channel,
                                                                      //timer B trigger sampling
    REFCTL0 &= ~REFMSTR;
    ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_7;      // A7, PIN 6.7
    ADC12CTL0 |= ADC12ENC;                       //enable conversion
    ADC12IE = BIT0;                              //Interrupt for MEM0

    /* Initialize Timer_B channel 1 to be used as ADC12 trigger */
    // Use SMCLK as Timer_B source
    TBCTL = TBSSEL_2;

    // Initialize TBCCR0 (period register) 15990784Hz/8000Hz = 1998.85
    // Simple counter with no interrupt. 0...1998 = 1999 counts/sample
    TBCCTL0 = 0x0000;
    TBCCR0 = 1998;

       // Initialize TBCCR1 to generate trigger clock output, reset/set mode
    TBCCTL1 = OUTMOD_7;
    TBCCR1 = 1998 - 100;

       // Disable all other timer channels
    TBCCTL2 = 0x0000;
    TBCCTL3 = 0x0000;
    TBCCTL4 = 0x0000;
    TBCCTL5 = 0x0000;
    TBCCTL6 = 0x0000;


    /* After set-up enable periherals. TB0 not enabled until startup of
       ADC desired. */
    ADC12CTL0 |= ADC12SC + ADC12ENC;             //enable ADC and start conversion


}

