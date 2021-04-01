#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "debugPrint.h"
#include "i2cmaster.h"
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
	
	
	debugInit();		//setup bluetooth usart
	
	//I2C setup
	DDRD &= ~(1 << PC5);
	DDRD &= ~(1 << PC4);
	PORTC |= (1<<PC5) | (1<<PC4); // Enable pull ups on I2C lines
	i2c_init();
	
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
			debug_str("s: start\n");
			debug_str("l/r: turn left/right\n");
			debug_str("t: stop\n");
			debug_str("d: slower 20\n");
			debug_str("e: end\n");

			//assuming right side is OCR0A and left is OCR2B, swap these if this ends up being wrong
			bool quit = false;
			while(!quit) {
				debug_str("\n> ");
				switch(debug_in()) {
					case 's':			//increment right by 20, which turns left
					forward();
					OCR0A = 0xA0;
					OCR2B = 0xA0;
					break;
					case 't':
					OCR0A = 0x00;
					OCR2B = 0x00;
					break;
					case 'l':			//increment left by 20, which turns right
					left();
					OCR0A = 0xA0;
					OCR2B = 0xA0;
					break;
					case 'r':			//increment left by 20, which turns right
					right();
					OCR0A = 0xA0;
					OCR2B = 0xA0;
					break;
					
					
					case 'e':
					quit = true;
					OCR0A = 0b00000000;
					OCR2B = 0b00000000;
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