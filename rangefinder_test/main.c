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
int counter = 0;
uint16_t distance = 0;

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

	// Main loop
	while(1){
		
		/* bluetooth commands
		s: start move forward
		l: stop and start left turn
		r: stop and start right turn
		b: start move backward
		e: stop whatever it was doing
		p: start obstacle detection
		
		tests:
		tr: rangefinder test, output distance values
		tt: temperature test, read device id
		ts: TWI bus scan, print all addresses on bus
		ta: adc test, output converted readings from IR sensors
		*/
		
		//temporary test switch case, controls robot one iteration at a time
		switch(debug_in()) {
			
			case 's':	//start move forward
			OCR0A = 0x0;
			OCR2B = 0x0;
			forward();
			_delay_ms(200);
			OCR0A = 0x80;
			OCR2B = 0x80;
			break;
			
			case 'l': //start moving left
			OCR0A = 0x0;
			OCR2B = 0x0;
			left();
			_delay_ms(200);
			OCR0A = 0xAA;
			OCR2B = 0xEE;
			break;
			
			case 'r': //start moving right
			OCR0A = 0x0;
			OCR2B = 0x0;
			right();
			_delay_ms(200);
			OCR0A = 0xEE;
			OCR2B = 0xAA;
			break;
			
			case 'b': //start moving backwards
			OCR0A = 0x0;
			OCR2B = 0x0;
			reverse();
			_delay_ms(200);
			OCR0A = 0x76;
			OCR2B = 0x76;
			break;
			
			case 'e':
			OCR0A = 0x0;
			OCR2B = 0x0;
			forward();
			break;
			
			case 'p':	//obstacle detection loop
			if(!rangefinder_initialized) {		//initialize rangefinder if not already done
				initVL53L0X(1);
				setMeasurementTimingBudget( 500 * 1000UL );	//500 ms per measurement
				rangefinder_initialized = true;
			}
			counter = 0;
			forward();
			OCR0A = 0x60;
			OCR2B = 0x60;
			distance = read_rangefinder();
			while( distance > 250 ) {
				distance = read_rangefinder();
				// debug output
				debug_dec(counter);
				debug_str(": distance = ");
				debug_dec(distance);
				debug_str("  ");
				debug_hex(distance,4);
				debug_str("\n");
				counter++;
			}
			OCR0A = 0x00;
			OCR2B = 0x00;
			break;

			case 't':		//tests
			
				switch(debug_in()) {
				
					case 'r':	//rangefinder test
					if(!rangefinder_initialized) {		//initialize rangefinder if not already done
						initVL53L0X(1);
						setMeasurementTimingBudget( 500 * 1000UL );	//500 ms per measurement
						rangefinder_initialized = true;
					}
					do {
						distance = read_rangefinder();
						debug_str("\nrange data:");
						debug_dec(distance);
						debug_str("  ");
						debug_hex(distance,4);
						debug_str("\n");
						debug_str("input any char to read again, or e to end");
						debug_str("\n");
					} while(debug_in()!='e');
					break;
				
					case 't':	//temperature test
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
					
					case 's':	//scan I2C bus=
					searchI2C();
					break;
					
					case 'a':
					start_move();	//starts adc reading on loop
					while(1) {
						if(nav_data_ready) {
							debug_str("\ndata: ");
							debug_hex(nav_data[0],4);
							debug_str(" ");
							debug_hex(nav_data[1],4);
							debug_str(" ");
							debug_hex(nav_data[2],4);
							debug_str("\n");
							debug_str("OCR0A: ");
							debug_hex(OCR0A,2);
							debug_str("\n");
							//nav_rules();				//apply rules
							nav_data_ready = false;		//reset flag
							debug_str("\nenter c to read again, i to increase pwm,  or e to end: ");
							char input = debug_in();
							if(input=='c') {
								ADCSRA |= (1 << ADSC);		//start ADC
							} else if(input=='i') {
								OCR0A = OCR0A + 50;
								ADCSRA |= (1 << ADSC);		//start ADC
							} else {
								break;
							}
						}
					}
					stop_move();
					break;
					
					default:
					break;

				}	// end of test case switch
			break;		//end of test cases
			
			default:
			break;

		}	//end of command handler switch

	}
	return 0;
}