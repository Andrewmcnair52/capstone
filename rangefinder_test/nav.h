/*
 * nav.h
 *
 * Created: 2021-03-22 9:41:25 PM
 *  Author: Andrew
 */ 
#ifndef NAV_H_
#define NAV_H_
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "VL53L0X.h"
#include "debugPrint.h"
extern uint8_t adc[3];
extern uint8_t nav_data[4];

void increment_OCR0A(uint8_t);
void decrement_OCR0A(uint8_t);
void increment_OCR2B(uint8_t);
void decrement_OCR2B(uint8_t);
void reverse();
void forward();
void left();
void right();

void timerSetup();			//setup timers 0 and 2 for pwm output
void setupADC();			//setup ADC for reflectance sensors
void start_move();			//start line follower
void stop_move();			//stop line follower
void nav_rules();			//interprets data
uint8_t read_rangefinder();	//read obstacle avoidance data from range finders
#endif /* NAV_H_ */