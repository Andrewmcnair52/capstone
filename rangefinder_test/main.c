
#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "debugPrint.h"
#include "i2cmaster.h"
#include "millis.h"
#include "VL53L0X.h"
#include "nav.h"
#include "adt7420.h"

void timerSetup();

//global vars for main
bool rangefinder_initialized = false;

//ISR shared variables
volatile bool nav_data_ready = false;
volatile bool is_moving = false;


void init(void) {
	
	//setup for motor drivers
	timerSetup();		//starts PWM on PD6(timer0) and PD3(timer2)
	DDRC |= (1<<PC0);	//setup PC0 for output
	PORTC &= ~(1<<PC0);	//set PC0 low
	DDRD |= (1<<PD2);	//set PD2 for output
	PORTD |= (1<<PD2);	//set PD2 high
	
	DDRB |= (1<<PB2);	//setup PB2 for output
	PORTB &= ~(1<<PB2);	//set PB2 low
	DDRD |= (1<<PD4);	//set PD4 for output
	PORTD &= ~(1<<PD4);	//set PD4 high
	
	debugInit();		//setup bluetooth usart
	
	//I2C setup
	DDRD &= ~(1 << PC5);
	DDRD &= ~(1 << PC4);
	PORTC |= (1<<PC5) | (1<<PC4); // Enable pull ups on I2C lines
	i2c_init();
	
	//timer/counter 2 setup
	initMillis();
	
	//navigation setup
	setupADC();
	
	sei();
}

int main() {
	
	init();
	
	//wait to receive char indicating app is ready to print data
	debug_in();
	
	// Main loop	
	while(1){
		
		
		debug_str("\n\nenter a character to select an option:\n");
		debug_str("r: rangefinder test\n");
		debug_str("t: temperature test\n");
		debug_str("s: print all addresses on I2C bus\n");
		debug_str("m: start moving\n");
		debug_str("c: controller mode\n");
		debug_str("\n> ");
		
		//temporary test switch case, controls robot one iteration at a time
		switch(debug_in()) {
			
			case 'r':							//range finder test
				if(!rangefinder_initialized) {		//initialize rangefinder if not already done
					initVL53L0X(1);
					setMeasurementTimingBudget( 500 * 1000UL );	//500 ms per measurement
					rangefinder_initialized = true;
				}
				debug_str("range data:");
				debug_dec( read_rangefinder() );
				debug_str("\n");
				break;
			
			case 't':		//temperature test
				debug_str("\nstarting temperature test\n");
				i2c_start( ADT7420_ADDRESS | I2C_WRITE );
				i2c_write( ADT7420_REG_ID );
				i2c_rep_start( ADT7420_ADDRESS | I2C_READ );
				uint8_t test = i2c_readNak();
				i2c_stop();
				
				if(test != ADT7420_DEFAULT_ID)
				debug_str("incorrect ID received\n");
				else debug_str("correct ID received");
				break;
				
			case 's':
				searchI2C();
				break;
				
			case 'm':
				start_move();	//starts adc reading on loop
				while(1) {
					if(nav_data_ready) {
						debug_str("\nADCH: ");
						debug_hex(nav_data[0],2);
						debug_str(" ");
						debug_hex(nav_data[1],2);
						debug_str(" ");
						debug_hex(nav_data[2],2);
						debug_str("\n");
						//nav_rules();				//apply rules
						nav_data_ready = false;		//reset flag
						debug_str("\nenter c to read again, or e to end: ");
						if(debug_in()=='c') {
							ADCSRA |= (1 << ADSC);		//start ADC
						} else { 
							break;
						}
					}
				}
				stop_move();
				break;
			
			case 'c':				//controller mode
				debug_str("\n\nController mode\n");
				debug_str("r: increment right by 20\n");
				debug_str("l: increment left by 20\n");
				debug_str("f: increment both by 20\n");
				debug_str("s: decrement both by 20\n");
				debug_str("e: end\n");
				
				//assuming right side is OCR0A and left is OCR2B, swap these if this ends up being wrong
				bool quit = false;
				while(!quit) {
					debug_str("\n> ");
					switch(debug_in()) {
						case 'r':			//increment right by 20, which turns left 
							increment_OCR0A(20);
						case 'l':			//increment left by 20, which turns right
							increment_OCR2B(20);
							break;
						case 'f':
							//increment_OCR0A(20);
							//increment_OCR2B(20);
							OCR0A = 0b10000000;
							OCR2B = 0b10000000;
							break;
						case 's':
							decrement_OCR0A(20);
							decrement_OCR2B(20);
							break;
						case 'e':
							quit = true;
							break;
					}
					debug_str("OCR0A: ");
					debug_hex(OCR0A, 2);
					debug_str(" OCR2B:");
					debug_hex(OCR2B, 2);
					debug_str("\n");
				}
				break;
				
			default:
				break;
				
			
		}
		
		//_delay_ms(600);		//delay 0.6s
		
	}
	return 0;
}



