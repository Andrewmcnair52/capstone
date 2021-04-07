#include <stdint.h>
#include "debugPrint.h"

#if DEBUGPRINT_ENABLED

void debugInit() {
	// Init UART
	PRR &= ~(1<<PRUSART0);
	UCSR0B |= (1<<TXEN0) | (1<<RXEN0);
	UBRR0H = (12 >> 8);
	UBRR0L = 12;
	UCSR0A |= (1<<U2X0);		//fast mode
}

void debug_putc( uint8_t c ) {
	while(!(UCSR0A & (1 << UDRE0)));    /* wait for data register empty */
	UDR0 = c;
}

void debug_str_internal( const char *p ){
	uint8_t c;
	while (( c=pgm_read_byte( p++ ) )){
		debug_putc( c );
		if( c=='\n' ){
			debug_putc('\r');
		}
	}
}

void debug_dec( uint32_t val ){
	char buffer[10];
	char *p = buffer;
	while (val || p == buffer) {
		*(p++) = val % 10;
		val = val / 10;
	}
	while (p != buffer) {
		debug_putc( '0' + *(--p) );
	}
}

void debug_hex( uint32_t val, uint8_t digits ){
	for (int i = (4*digits)-4; i >= 0; i -= 4)
	debug_putc( "0123456789ABCDEF"[(val >> i) % 16] );
}


char debug_in() {	//receive data function

	while (!(UCSR0A & (1 << RXC0)));	//wait until data is received
	return(UDR0);						//return data
}


#endif