// Lab 2 for ECE 4760, Spring 2012, Cornell University
// Written by William Myers (wgm37) and Guo Jie Chin (gc348), Feb 12, 2012
// Changed: sample positions, 30k instead of 10k (check this in state machine)

//	Port Description
//
//	A0: Probe input, post diode	B0:							C0:	LCD						D0: UART
//	A1:							B1:							C1:	LCD						D1: UART
//	A2:							B2:	Probe input, pre diode	C2:	LCD						D2: Button - Measurement Type (UART Disabled)
//	A3:							B3:	To 10k, 10k V Divider	C3:	LCD						D3: Button - automatic/Manual Toggle (UART Disab.)
//	A4:							B4:							C4:	LCD						D4: Button - Manual Range Toggle
//	A5:	1k Resistor				B5: 							C5:	LCD						D5:
//	A6:	10k Resistor	 			B6:							C6:	LCD						D6:
//	A7:	100k Resistor 			B7:							C7:	LCD						D7:


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "uart.h"
#include "lcd_lib.h"

#define t1 200 				// LCD update
#define t2 20 				// Button Check
#define button_count 3 		// number of buttons to check

//State machine state names
#define voltageState 1
#define resistanceState 2
#define frequencyState 3
#define readyState 4

//State machine state names
#define NoPush 1 
#define MaybePush 2
#define Pushed 3
#define MaybeNoPush 4

//State Machine state names
#define automatic 1
#define manual 2

//Button press macros
#define maxkeys 16
#define PORTDIR DDRD
#define PORTDATA PORTD
#define PORTIN PIND

//flash storage for LCD
const int8_t LCD_voltage[] PROGMEM = "Voltage:\0";
const int8_t LCD_resistance[] PROGMEM = "Resistance:\0";
const int8_t LCD_frequency[] PROGMEM = "Frequency:\0";
const int8_t LCD_ready[] PROGMEM = "Ready!\0";

//Global const

const float highVolt = 5.03;			// Needs to be calibrated for each board!
const float mediumVolt = 2.54;		// Needs to be calibrated for each board!
const float lowVolt = 1.1;			// Needs to be calibrated for each board!
const long unsigned int r100k = 92900;			// Needs to be calibrated for R values!
const unsigned int r10k = 9950;			// Needs to be calibrated for R values!
const unsigned int r1k = 993;				// Needs to be calibrated for R values!
const long unsigned int clk = 16000000;			// processor speed for frequency calc

void initialize(void);
void init_lcd(void);
void check_button(int i); 
unsigned char get_button(void);
void set_ready(void);
void set_voltage(void);
void set_resistance(void);
void set_frequency(void);
void update_LCD(void);
void run_voltage(void);
void run_resistance(void);
void run_frequency(void);


volatile int time1;
volatile int time2;
int8_t lcd_buffer[17];
int8_t lcd_status[1];						// used for automatic / manual display on LCD
unsigned char PushFlag[button_count] = {0,0,0};		// message indicating a button push for 2 buttons
unsigned char PushState[button_count] = {1,1,1};		// state machine for 2 buttons
unsigned char state;							// high level state machine variable
unsigned char init; 							// variable for setting up between transitions
unsigned char voltageStateVar;						// state for voltage range
unsigned char resistanceStateVar;				// state for resistance range
unsigned char frequencyStateVar;				// state for frequency range
unsigned char automaticState;						// state for manual/automatic range detection
unsigned char overflow;						// overflow state (1 = overflow 0 = no)
unsigned short capture;						// period measurement in prescaled cycles
unsigned char ain;							// raw ADC value
unsigned char i;							// used for button checking
unsigned char freqReady;					// used for ensuring proper sampling in frequency measurement
unsigned char butnum ;						// decoded button number
unsigned char button_phys;					// storage for physical button press
unsigned char key ; 						// raw keyscan
float voltage; 								// voltage value
float resistance;							// resistance value
float frequency;								// frequency value


//key pad scan table
unsigned char keytbl[16]={0xee, 0xed, 0xeb, 0xe7, 
						  0xde, 0xdd, 0xdb, 0xd7, 
						  0xbe, 0xbd, 0xbb, 0xb7, 
						  0x7e, 0x7d, 0x7b, 0x77};

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//timer 0 compare ISR - fires every 1ms
ISR (TIMER0_COMPA_vect)
{       	
	if (time1 > 0)  --time1;		// LCD
	if (time2 > 0) 	--time2;		// Buttons
}
ISR(TIMER1_CAPT_vect)
{
	TCNT1 = 0;			// reset counter
	capture = ICR1;		// store the timer value, this is the period
	freqReady = 1;
}
ISR(TIMER1_OVF_vect)
{
	overflow = 1;		// if we get an overflow signal with this variable
}

int main(void)
{
    initialize();		// setup general port and variable stuff once
	init_lcd();			// setup some LCD stuff once

	while(1){
	
		

	    if(time1 == 0){ time1 = t1; update_LCD();}					// dispatcher for LCD updating
	    if(time2 == 0){
	    		time2 = t2;												// dispatcher for button checking
	    		for(i = 0; i < button_count; i++) check_button(i); 	
	    }
		
		switch(state){												// voltage, resistance, or frequency measure
			case readyState:
				if(init == 0) {set_ready(); init = 1;}
				if(PushFlag[0] == 1){								// if we get a measurement type button press...
					init = 0;										// set up for the next initialization
					state = voltageState;							// update state
					PushFlag[0] = 0;									// turn off button
				}
			break;
			case voltageState:
				if(init == 0) {set_voltage(); init = 1;}				// initialize some ports for voltage read
				if(PushFlag[0] == 1){								// if we get a measurement type button press...
					init = 0;										// set up for the next initialization
					state = resistanceState;							// update state
					PushFlag[0] = 0;									// turn off button
				}

				run_voltage();										// calculate voltage!
				
			break;
			case resistanceState:
				if(init == 0) {set_resistance(); init = 1;}			// initialize some ports for resistance read
				if(PushFlag[0] == 1){								// if we get a measurement type button press..
					init = 0;										// set up for the next init
					state = frequencyState;							// update state
					PushFlag[0] = 0;									// turn off button
				}
				
				run_resistance();									// calculate resistance!
				
			break;
			case frequencyState:
				if(init == 0) {set_frequency(); init = 1;}			// initialize some ports for frequency read
				if(PushFlag[0] == 1){								// if we get a measurement type button press..
					init = 0;										// set up for the next init
					state = readyState;							// update state
					PushFlag[0] = 0;									// turn off button
				}
				
				run_frequency();										// calculate frequency!
				
			break;
		}
	}
}

void run_voltage(void){

	switch(automaticState){															// automatic or manual range detection...
		case automatic:
			if(PushFlag[1] == 1) {PushFlag[1] = 0; automaticState = manual;}			// cycle based on button push between automatic and manual
			//ADCSRA |= (1 << ADSC);												// start an ADC conversion
						
			switch(voltageStateVar){
				case 1:															// Vcc
					ADMUX = (1 << ADLAR) | (0 << REFS1) | (1 << REFS0);			// format, [0,5]V range
					ADCSRA |= (1 << ADSC);												// start an ADC conversion
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// store raw value
					voltage = ((float)ain / 255) * highVolt; 							// calculate voltage
					if(voltage <= mediumVolt - .2) voltageStateVar = 2;						// if we are measuring under 2.56 drop a state
							
				break;
				case 2:														// 2.56v
					ADMUX = (1 << ADLAR) | (1 << REFS1) | (1 << REFS0); 			// format, [0,2.56]V range
					ADCSRA |= (1 << ADSC);												// start an ADC conversion
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion and set new conversion
					ain = ADCH;													// store raw value
					voltage = ((float)ain / 255) * mediumVolt;							// calcualte voltage
					if(voltage <= lowVolt - .2) voltageStateVar = 3;						// if we are under 1.1v drop a state
					else if(ain == 0xff) voltageStateVar = 1;							// we are clipping the range, bump up states
				break;		
				case 3:															// 1.1v
					ADMUX = (1 << ADLAR) | (1 << REFS1) | (0 << REFS0); 			// format, [0,1.1]V range
					ADCSRA |= (1 << ADSC);												// start an ADC conversion
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop
					ain = ADCH;													// store raw voltage
					voltage = ((float)ain / 255) * lowVolt;								// calculate voltage
					if(ain == 0xff) voltageStateVar = 2;								// we are clipping, need to bump up states
				break;						
			}
									
		break;
		case manual:
			if(PushFlag[1] == 1) {PushFlag[1] = 0; automaticState = automatic;}
			//ADCSRA |= (1 << ADSC);
					
			switch(voltageStateVar){
				case 1:															// Vcc
					ADMUX = (1 << ADLAR) | (0 << REFS1) | (1 << REFS0);
					ADCSRA |= (1 << ADSC);										
					while((ADCSRA & (1 << ADSC)) == 1){};
					ain = ADCH;
					voltage = ((float)ain / 255) * highVolt;
					if(PushFlag[2] == 1) {PushFlag[2] = 0; voltageStateVar = 2;}		// increment states manually
				break;
				case 2: 															// 2.56v
					ADMUX = (1 << ADLAR) | (1 << REFS1) | (1 << REFS0); 
					ADCSRA |= (1 << ADSC); 
					while((ADCSRA & (1 << ADSC)) == 1){};
					ain = ADCH;
					voltage = ((float)ain / 255) * mediumVolt;
					if(PushFlag[2] == 1) {PushFlag[2] = 0; voltageStateVar = 3;}		// increment states manually
				break;		
				case 3:															// 1.1v
					ADMUX = (1 << ADLAR) | (1 << REFS1) | (0 << REFS0); 
					ADCSRA |= (1 << ADSC); 
					while((ADCSRA & (1 << ADSC)) == 1){};
					ain = ADCH;
					voltage = ((float)ain / 255) * lowVolt;
					if(PushFlag[2] == 1) {PushFlag[2] = 0; voltageStateVar = 1;}		// increment states manually
				break;
			}				
		break;
	}
}
void run_resistance(void){

	switch(automaticState){
		case automatic:
			if(PushFlag[1] == 1) {PushFlag[1] = 0; automaticState = manual;}			// toggle between automatic and manual state
			//ADCSRA |= (1 << ADSC);													// start a conversion
						
			switch(resistanceStateVar){
				case 1:
					DDRA = (1 << PINA7);											// Set A7 as an output, all others to inputs 
					PORTA = (1 << PINA7);										// Set A7 as high, all others turn pull-up off
					ADCSRA |= (1 << ADSC);															// 100k
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// store raw voltage
					voltage = ((float)ain / 255) * highVolt;							// calculate the voltage
					resistance = r100k/(highVolt/voltage-1); 					// calculate the resistance
					if(resistance <= 30000) resistanceStateVar = 2;					// If we measure below 30kOhm, drop a state
				break;
				case 2:	
					DDRA = (1 << PINA6);											// Set A6 as an output, all others to inputs 
					PORTA = (1 << PINA6);										// Set A6 as high, all others turn pull-up off
					ADCSRA |= (1 << ADSC);															// 100k	
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// store the raw voltage
					voltage = ((float)ain / 255) * highVolt;							// calculate voltage
					resistance = r10k/(highVolt/voltage-1); 						// calculate resistance			
					if(resistance <= 3000) resistanceStateVar = 3;					// if below 3kOhm, drop a state
					else if(resistance >= 70000) resistanceStateVar = 1; 			// if above 70kOhm, jump a state
				break;		
				case 3:	
					DDRA = (1 << PINA5);											// Set A5 as an output, all others to inputs 
					PORTA = (1 << PINA5);										// Set A5 as high, all others turn pull-up off
					ADCSRA |= (1 << ADSC);															// 100k
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// get raw voltage
					voltage = ((float)ain / 255) * highVolt;							// calc voltage
					resistance = r1k/(highVolt/voltage-1);						// calc resistance				
					if(resistance >= 7000) resistanceStateVar = 2;  				// if resistance is above 7kOhm, jump a state	
				break;
			}
						
		break;
				
		case manual:
			if(PushFlag[1] == 1) {PushFlag[1] = 0; automaticState = automatic;}
			//ADCSRA |= (1 << ADSC);
						
			switch(resistanceStateVar){
				case 1:															// 100k
					DDRA = (1 << PINA7);											// Set A7 as an output, all others to inputs 
					PORTA = (1 << PINA7);										// Set A7 as high, all others turn pull-up off
					ADCSRA |= (1 << ADSC);															// 100k
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// store raw voltage
					voltage = ((float)ain / 255) * highVolt;							// calculate the voltage
					resistance = r100k/(highVolt/voltage - 1); 					// calculate the resistance
					if(PushFlag[2] == 1) {PushFlag[2] = 0; resistanceStateVar = 2;}	// Manual state change
				break;
				case 2:															// 10k
					DDRA = (1 << PINA6);											// Set A6 as an output, all others to inputs 
					PORTA = (1 << PINA6);										// Set A6 as high, all others turn pull-up off
					ADCSRA |= (1 << ADSC);															// 100k
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// store the raw voltage
					voltage = ((float)ain / 255) * highVolt;							// calculate voltage
					resistance = r10k/(highVolt/voltage - 1); 						// calculate resistance		
					if(PushFlag[2] == 1) {PushFlag[2] = 0; resistanceStateVar = 3;}	// Manual State Change
				break;		
				case 3:															// 1k
					DDRA = (1 << PINA5);											// Set A5 as an output, all others to inputs 
					PORTA = (1 << PINA5);										// Set A5 as high, all others turn pull-up off
					ADCSRA |= (1 << ADSC);															// 100k
					while((ADCSRA & (1 << ADSC)) == 1){};							// wait loop for conversion
					ain = ADCH;													// get raw voltage
					voltage = ((float)ain / 255) * highVolt;							// calc voltage
					resistance = r1k/(highVolt/voltage - 1);						// calc resistance		 
					if(PushFlag[2] == 1) {PushFlag[2] = 0; resistanceStateVar = 1;}	// Manual State Change
				break;
			}		
		break;				
	}
}

void run_frequency(void){

	switch(automaticState){
		case automatic:
			if(PushFlag[1] == 1) {PushFlag[1] = 0; automaticState = manual;}			// Toggle automatic/Manual
						
			switch(frequencyStateVar){
				case 1:															// Prescalar = 1
					TCCR1B = (1 << ICES1) + 1; 									// Prescalar set to 1
					if(freqReady == 1) frequency = clk/((float)capture);									// calculate frequency
					if(overflow == 1) {overflow = 0; frequencyStateVar = 2; freqReady = 0;} 		// if we overflow we're counting too fast, change 
				break;
				case 2: 															// Prescalar = 8
					TCCR1B = (1 << ICES1) + 2; 									// Prescalar set to 8
					if(freqReady == 1) frequency = clk/((float)capture * 8);								// calculate frequency
					if(frequency > 400) {frequencyStateVar = 1; freqReady = 0;}					// If we can safely bump up, do so
				break;		
			}
										
		break;
					
		case manual:
			if(PushFlag[1] == 1) {PushFlag[1] = 0; automaticState = automatic;}
						
			switch(frequencyStateVar){
				case 1:															// Prescalar = 1
					TCCR1B = (1 << ICES1) + 1; 									// Prescalar set to 1
					frequency = clk/((float)capture);
					if(PushFlag[2] == 1) {PushFlag[2] = 0; frequencyStateVar = 2;}
				break;
				case 2:															// Prescalar = 8
					TCCR1B = (1 << ICES1) + 2; 									// Prescalar set to 8
					frequency = clk/((float)capture * 8);
					if(PushFlag[2] == 1) {PushFlag[2] = 0; frequencyStateVar = 1;}
				break;		
			}
		break;			
	}
}

unsigned char get_button(void){

	//get lower nibble
  	PORTDIR = 0x0f;
  	PORTDATA = 0xf0; 
  	_delay_us(5);
  	key = PORTIN;
  	  
  	//get upper nibble
  	PORTDIR = 0xf0;
  	PORTDATA = 0x0f; 
  	_delay_us(5);
  	key = key | PORTIN;
  	  
  	//find matching keycode in keytbl
  	if (key != 0xff){ 
  	  for (butnum=0; butnum<maxkeys; butnum++)
  	  {  
  	  	if (keytbl[butnum]==key)  break;   
  	  }

  	  if (butnum==maxkeys) butnum=0;
  	  else butnum++;	   //adjust by one to make range 1-16
  	} 
  	else butnum=0;

	return butnum;

}

 
void check_button(int i){

	button_phys = get_button();
	

  	switch (PushState[i]){
    		case NoPush: 
    		    if (button_phys == i+1) PushState[i] = MaybePush;
    		    else PushState[i] = NoPush;
    		    break;
    		case MaybePush:
        		if (button_phys == i+1){
           		PushState[i] = Pushed;   
           		PushFlag[i] = 1;
        		}
        		else PushState[i] = NoPush;
        		break;
     	case Pushed:  
        		if (button_phys == i+1) PushState[i] = Pushed; 
        		else PushState[i] = MaybeNoPush;    
        		break;
     	case MaybeNoPush:
        		if (button_phys == i+1) PushState[i] = Pushed; 
        		else {
           		PushState[i] = NoPush;
           		PushFlag[i] = 0;
        		}  
        		break;
  	}		
}

void update_LCD(void){
	
	switch(state){
		case voltageState:
			dtostrf(voltage,1,3,lcd_buffer);
			LCDGotoXY(0,1);
			LCDstring(lcd_buffer,strlen(lcd_buffer));

			sprintf(lcd_status,"%c", voltageStateVar);				// Display an A for automatic Mode
			LCDGotoXY(14,1);
			LCDstring(lcd_status, strlen(lcd_status));
		break;
		case resistanceState:
			dtostrf(resistance,1,3,lcd_buffer);
			LCDGotoXY(0,1);
			LCDstring(strcat(lcd_buffer,"      \0"),strlen(lcd_buffer));
		break;
		case frequencyState:
			dtostrf(frequency,1,3,lcd_buffer);		// decimal output with 4 digits
			LCDGotoXY(0,1);
			LCDstring(lcd_buffer,strlen(lcd_buffer));		
		break;
	}
	
	if(state != readyState){
		switch(automaticState){
			case automatic:
				sprintf(lcd_status,"%c", 'A');				// Display an A for automatic Mode
				LCDGotoXY(15,1);
				LCDstring(lcd_status, strlen(lcd_status));
			break;
			case manual:
				sprintf(lcd_status,"%c", 'M');				// Display an M for Manual Mode
				LCDGotoXY(15,1);
				LCDstring(lcd_status, strlen(lcd_status));
			break;
		}
	}
	
}
void set_ready(void){
	LCDclr();
	CopyStringtoLCD(LCD_ready,0,0);
}

void set_voltage(void){
	DDRA = 0;
	automaticState = manual;					// manual
	
	LCDclr();
	CopyStringtoLCD(LCD_voltage,0,0);
}
void set_resistance(void){	
	DDRA = (1 << PINA7);					// 100k initial output
	PORTA = (1 << PINA7);				// 100k initial high
		
	automaticState = manual;	
	
	ADMUX = (1 << ADLAR) | (0 << REFS1) | (1 << REFS0); // set up ADC for 5v, format register
	
	LCDclr();
	CopyStringtoLCD(LCD_resistance,0,0);
	
}
void set_frequency(void){
	DDRA = 0;
	PORTA = 0;
	DDRB = 0;							// input
	automaticState = manual;	
	capture = 1;							// make sure we don't get div/0 errors
	
	TCNT1 = 0;							// Initialize the counter
	
	
	LCDclr();
	CopyStringtoLCD(LCD_frequency,0,0);
}
void init_lcd(void){
	LCDinit();
	LCDcursorOFF();
	LCDclr();
}
void initialize(void)
{
    DDRD = 0x00;							// set as input for buttons
    
    // set up timer 0 for 1 mSec timebase 
  	OCR0A = 249;  							// set the comparere to 250 time ticks
  	TIMSK0= (1 << OCIE0A);					// turn on timer 0 cmp match ISR 
  	TCCR0B= 3; 								// 0b00000011 - divide by 64;	
  	TCCR0A= (1 << WGM01);			  		// turn on clear-on-match
           	
    // init some values
    time1=t1;  								// counter for LCD
    time2=t2;								// counter for buttons
    voltageStateVar = 1;							// volt state starts at 5v
    resistanceStateVar = 1;						// resistance state starts at 100k
    frequencyStateVar = 1;						// frequency state starts at ...?
    init = 0;								// initially each function is unitialized
    state = readyState;					// start in volt mode
    overflow = 0;							// start with no overflow
    voltage = 0;								// init values
    resistance = 0;							// init values
    frequency = 0;							// init values
    
    // init ADC
   	ADMUX = (1 << ADLAR) | (1 << REFS0);		// Format register and set to 5v  
   	ADCSRA = (1 << ADEN) + 7;				// set prescalar for 125,000, clear interrupt enable
   	
   	// init Timer 1 Capture
   	TCCR1B = (1 << ICES1) + 1; 				// Set capture enable for postive edge and go at full speed
   	TIMSK1 = (1 << ICIE1) | (1 << TOIE1); 	// Set interrupt enables for capture and overflow
   	ACSR = (1 << ACIC); 						// Wire comparator to ICR1 and set pin B.2 as the positive terminal
   											// Negative input is B.3 by default

  
    // init the UART -- uart_init() is in uart.c
    uart_init();
    stdout = stdin = stderr = &uart_str;
    fprintf(stdout,"Starting...\n\r");
      
    //crank up the ISRs
    sei();

}



