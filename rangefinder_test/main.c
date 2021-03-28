
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

//ISR shared variables
volatile bool nav_data_ready = false;
volatile bool is_moving = false;


void init(void) {
	
	//setup for motor drivers
	timerSetup();		//starts PWM on PD6(timer0) and PD3(timer2)
	DDRC |= (1<<PC0);	//setup PC0 for output
	PORTC |= (1<<PC0);	//set PC0 high
	DDRD |= (1<<PD2);	//set PD2 for output
	PORTD |= (1<<PD2);	//set PD2 high
	
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

	initVL53L0X(1);
	setMeasurementTimingBudget( 500 * 1000UL );	//500 ms per measurement
	
	// Main loop	
	while(1){
		
		debug_in();	//wait for character input before every iteration
		
		//test: search for I2C addresses on bus
		searchI2C();
		
		
		//range finder test
		debug_str("range data:");
		debug_dec( read_rangefinder() );
		debug_str("\n");
		
		/*
		//test3
		debug_str("\nstarting temperature test\n");
		if( i2c_start( ADT7420_ADDRESS | I2C_WRITE ) == 0 ) {
			i2c_write( ADT7420_REG_ID );
			i2c_rep_start( ADT7420_ADDRESS | I2C_READ );
			uint8_t test = i2c_readNak();
			i2c_stop();
			
			if(test != ADT7420_DEFAULT_ID)
			debug_str("incorrect ID received\n");
			else debug_str("correct ID received");
		}
		*/
		
		//process navigation data if available, and if moving
		if(is_moving) {
			
			
			if(nav_data_ready) {
				/*
				debug_str("ADCH: ");
				debug_dec(nav_data[0]);
				debug_str(" ");
				debug_dec(nav_data[1]);
				debug_str(" ");
				debug_dec(nav_data[2]);
				debug_str(" ");
				debug_str("\n");
				*/
				//nav_rules();				//apply rules
				nav_data_ready = false;		//reset flag
				ADCSRA |= (1 << ADSC);		//start ADC
				
			}
		}
		
		_delay_ms(600);		//delay 0.6s
		
	}
	return 0;
}



