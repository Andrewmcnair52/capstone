
#include "nav.h"

//ISR shared variables
extern volatile bool nav_data_ready;
extern volatile bool is_moving;

uint8_t adc[3] = { 0b01100001, 0b01100010, 0b01100011 };	//select which ADC, ADMUX = { adc[0] -> ADC3 || adc[1] -> ADC5 || adc[2] -> ADC7 }
uint8_t nav_data[4];

uint8_t initial_PD6_PWM = 0b00000000;
uint8_t initial_PD3_PWM = 0b00000000;

void start_move() {
	
	ADMUX = adc[0];				//start at ADC3
	is_moving = true;
	ADCSRA |= (1 << ADSC);		//start ADC
	
}

void stop_move() {
	is_moving = false;
}

//=========================================================== PWM control

void increment_OCR0A(uint8_t inc) {
	uint8_t current_val = OCR0A;
	if( (current_val+inc) > 255) OCR0A = 255;
	else OCR0A = current_val + inc;
}

void decrement_OCR0A(uint8_t inc) {
	uint8_t current_val = OCR0A;
	if( (current_val-inc) < 0) OCR0A = 0;
	else OCR0A = current_val - inc;
}

void increment_OCR2B(uint8_t inc) {
	uint8_t current_val = OCR2B;
	if( (current_val+inc) > 255) OCR2B = 255;
	else OCR2B = current_val + inc;
}

void decrement_OCR2B(uint8_t inc) {
	uint8_t current_val = OCR2B;
	if( (current_val-inc) < 0) OCR2B = 0;
	else OCR2B = current_val - inc;
}

//=========================================================== reflectance sensor ISR

ISR(ADC_vect) {

	if(!is_moving) {	//check stop condition
		
		//stop the robot
		
	} else if(ADMUX==adc[0]) {		//if first sensor converted
		
		nav_data[0] = ADCH;			//copy data
		ADMUX = adc[1];				//change adc channel to second sensor
		ADCSRA |= (1 << ADSC);		//start next adc
		
	} else if(ADMUX==adc[1]) {		//if second sensor converted
		
		nav_data[1] = ADCH;			//copy data
		ADMUX = adc[2];				//change adc channel to third sensor
		ADCSRA |= (1 << ADSC);		//start next adc
		
	} else if(ADMUX==adc[2]) {		//if third sensor converted
		
		nav_data[2] = ADCH;			//store data
		ADMUX = adc[0];				//change adc channel to first sensor
		nav_data_ready = true;		//set flag for main
		
	}
	
}

//=========================================================== NAV algorithm

void nav_rules() {
	
	//assuming ADC3=left, ADC5=middle, ADC7=right
	//this will be a basic algorithm, to be modified or replaced entirely once we can test it
	
	uint8_t lightLineThreshold = 1;		//to be determined by sensor output
	// data > whiteLineThreshold is assumed to be over white line
	uint8_t darkLineThreshold = 1;		//to be determined by sensor output
	// data < darkLineThreshold is assumed to be over dark line
	
	//floor values should be above dark threshold and below light threshold
	//ie: darkLineThreshold < data < lightLineThreshold
	
	//		|    | X | X  |X	veered right				dark,light,neither
	//		|    |X  |X  X|		veered right				dark,light,light
	//		|   X|  X|  X |		veered right but can tell
	//		|  X | X | X  |		centered					light,dark,dark
	//		| X  |X  |X   |		veered left but cant tell
	//		|X  X|  X|    |		veered left					light,light,dark
	//	   X|  X | X |    |		veered left					neither,light, dark
	
	if( (nav_data[0]<darkLineThreshold) && (nav_data[1]>lightLineThreshold) && (nav_data[3]<lightLineThreshold) && (nav_data[3]>darkLineThreshold) ) {			//dark,light,neither
		//veered right, correct left
		} else if( (nav_data[0]<darkLineThreshold) && (nav_data[1]>lightLineThreshold) && (nav_data[3]>lightLineThreshold) ) {		//dark,light,light
		//veered right, correct left
		} else if( (nav_data[0]>lightLineThreshold) && (nav_data[1]>lightLineThreshold) && (nav_data[3]<darkLineThreshold) ) {		//light,light,dark
		//veered left, correct right
		} else if( (nav_data[0]<lightLineThreshold) && (nav_data[0]>darkLineThreshold) && (nav_data[1]>lightLineThreshold) && (nav_data[3]<darkLineThreshold) ) {	//neither,light,dark
		//veered left, correct right
		} else if( (nav_data[0]<darkLineThreshold) && (nav_data[1]>lightLineThreshold) && (nav_data[3]>lightLineThreshold) ) {	//light, light, light
		//could use this as a stop condition to denote robot has arrived
	}
	
	
}


//===========================================================  obstacle avoidance

uint8_t read_rangefinder() {
	
	statInfo_t xTraStats;
	
	readRangeSingleMillimeters( &xTraStats );	// blocks until measurement is finished
	/* Returns:
	xTraStats.rangeStatus
	xTraStats.ambientCnt
	xTraStats.signalCnt
	xTraStats.spadCnt
	xTraStats.rawDistance
	*/
	if ( timeoutOccurred() ) {
		debug_str(" !!! Timeout !!! \n");
	}
	
	return xTraStats.rawDistance;
	
}

//=========================================================== setup functions

void setupADC() {
	
	//J6: data ->	PC3 -> ADC3 -> 0011
	//J8: data ->	PC2 -> ADC2 -> 0010
	//J12: data ->	PC1	-> ADC1 -> 0001
	
	
	/*
	ADMUX - ADC Multiplexer Selection Register
	
	bit          7           6          5         4        3         2          1          0
	name       REFS1       REFS0      ADLAR       -       MUX3      MUX2       MUX1       MUX0
	set to       0           1          1         0        0         0          1          1
	
	REFS1 = 0    use AVCC for reference voltage
	REFS0 = 1
	
	ADLAR = 1    left justify ADC result in ADCH/ADCL
	
	bit 4 = 0
	
	MUX3 = 0     //set to ADC3 for now (PC3)
	MUX2 = 0
	MUX1 = 1
	MUX0 = 1
	*/
	ADMUX = 0b01100011;			
	
	/*
	ADCSRA - ADC Control and Status Register A
	
	bit          7           6            5          4          3            2           1           0
	name        ADEN        ADSC        ADATE       ADIF       ADIE        ADPS2       ADPS1       ADPS0
	set to       1           0            0          0          1            0           1           1
	
	ADEN = 1     enable ADC
	ADSC = 0     don't start ADC yet
	ADATE = 0    don't enable ADC auto trigger (i.e. use single conversion mode)
	ADIF = 0     don't set ADC interrupt flag
	ADIE = 1     enable ADC interrupt
	
	ADPS2 = 0
	ADPS1 = 1    1 MHz clock / 8 = 125 kHz ADC clock
	ADPS0 = 1
	*/
	ADCSRA = 0b10001011;
	
	/*
	ADCSRB - ADC Control and Status Register B
	
	bit         7           6           5           4           3         2           1           0
	name        -          ACME         -           -           -       ADTS2       ADTS1       ADTS0
	set to      0           0           0           0           0         0           0           0
	
	bit 7 = 0
	ACME = 0     don't enable analog comparator multiplexer
	bit 5 = 0
	bit 4 = 0
	bit 3 = 0
	ADTS2 = 0
	ADTS1 = 0    register ADCSRA bit ADATE set to 0 so these bits have no effect
	ADTS0 = 0
	*/
	ADCSRB = 0b00000000;
	
}

void timerSetup() {
	
	debug_str("timer setup");
	
	//U1: motor driver L6203
	//-ln1 -> PA3
	//-En  -> PD6 (PWM timer counter 0, 8b)
	//-ln2 -> PA2 

	//U2: motor driver
	//-ln1 -> PA4
	//-En  -> PD3   (PWM timer counter 2, OC2B, 8b)
	//-ln2 -> PA5
	
	DDRD |= (1 << PD6) | (1 << PD3);		//set PD6 and PD3 for output
	
	/*
	TCCR0A - Timer/Counter 0 Control Register A
	
	bit           7         6         5         4        3       2        1        0
	name        COM0A1    COM0A0    COM0B1    COM0B0     -       -      WGM01    WGM00
	set to        1         0         0         0        0       0        1        1
	
	COM0A1 = 1    when Timer/Counter 0 (TCNT0) rolls over, set pin OC0A to high
	COM0A0 = 0    when Timer/Counter 0 (TCNT0) equals OCR0A, set pin OC0A to low
	
	COM0B1 = 0    normal port operation, OC0B disconnected
	COM0B0 = 0
	
	bit 3 = 0
	bit 2 = 0
	
	WGM01 = 1     Fast PWM mode, also see TCCR0B
	WGM00 = 1
	*/
	TCCR0A = 0b10000011;
	
	/*
	TCCR2A - Timer/Counter 2 Control Register A,
	
	bit           7         6         5         4        3       2        1        0
	name        COM0A1    COM0A0    COM0B1    COM0B0     -       -      WGM01    WGM00
	set to        0         0         1         0        0       0        1        1
	
	COM0A1 = 0		normal port operation, OC2A disconnected
	COM0A0 = 0    
	
	COM0B1 = 1		when Timer/Counter 2 (TCNT0) rolls over, set pin OC2B to high
	COM0B0 = 0		when Timer/Counter 2 (TCNT0) equals OCR0A, set pin OC2A to low
	
	bit 3 = 0
	bit 2 = 0
	
	WGM01 = 1     Fast PWM mode, also see TCCR0B
	WGM00 = 1
	*/
	TCCR2A = 0b00100011;
	
	/*
	TCCR0B - Timer/Counter 0 Control Register B
	
	bit           7          6        5       4         3         2         1        0
	name        FOC0A      FOC0B      -       -       WGM02      CS02      CS01     CS00
	set to        0          0        0       0         0         0         0        1
	
	FOC0A = 0     not used in PWM mode
	FOC0B = 0
	
	bit 5 = 0
	bit 4 = 0
	
	WGM02 = 0     Fast PWM mode, also see TCCR0A
	
	CS02 = 0
	CS01 = 0      no prescaling
	CS00 = 1
	*/
	TCCR0B = 0b00000001;
	TCCR2B = 0b00000001;	//same as timer/counter 0
	
	OCR0A = initial_PD6_PWM;			// PWM PD6/OC0A 50% duty cycle
	OCR2B = initial_PD3_PWM;			// PWM PD3/OC2B 50% duty cycle
	
}