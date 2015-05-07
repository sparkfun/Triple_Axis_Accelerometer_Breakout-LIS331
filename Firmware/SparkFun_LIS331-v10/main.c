/*
    3-21-11
    Aaron Weiss, aaron at sparkfun dot com
	
	License: Creative Commons Attribution Share-Alike 3.0
	http://creativecommons.org/licenses/by-sa/3.0
	
	v10 Example
	-outputs x,y,z acceleration values every 1 ms
	-output is driven by a timer overflow interrupt, this allows the user to
	 more easily add additional code without messing with the timing
	-values are in g's
	
	LIS331 Accelerometer Configuration: 
	4-wire SPI, normal mode, 400Hz 
	
	Hardware:
	Atmega328p, 3.3V, ext. 8MHz, 38400 baud
	
	Hardware Conections (LIS to Arduino):
	
	LIS331   |   Arduino
	---------------------
	  VCC    |    VCC (3.3V)
	  CS     |    pin 10 (PB2)
	  SCL    |    pin 13 (PB5)
	  SA0    |    pin 12 (PB4)
	  SDA    |    pin 11 (PB3)
	  GND    |    pin GND
	
	Note: if you need a higher sampling frequency, you need to run a higher clock,
		  INT functionality not used
*/

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define MYUBRR 12  //8MHz, 38400bps 

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define MOSI    3  //PORTB
#define MISO    4  //PORTB
#define SCK     5  //PORTB
#define CS		2  //PORTB

#define SCALE 0.0007324  //sets g-level (48 fullscale range)/(2^16bits) = SCALE

///============Initialize Prototypes=====================///////////////////////
void ioinit(void);      // initializes IO
void UART_Init(unsigned int ubrr);
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
void delay(uint16_t x); // general purpose delay

/////======SPI functions=================================////////////////////////
void SPI_MasterInit(void);
void SPI_MasterTransmit(uint8_t address);

/////======Main Prototypes================================////////////////////////
void config(void);
uint8_t read(uint8_t addr);
void accel(void);

///===========Global Vars=============================//////////////////////////
volatile double xaccel, yaccel, zaccel;

//=========MAIN================/////////////////////////////////////////////////
ISR(TIMER1_OVF_vect)
{
	//every ms the acceleration values are printed
	TCNT1 = 55536;//(8/8MHz)*(65536bits-55536)=0.01s
	accel(); //update LIS registers
	printf("x=%6.2f,", xaccel);
	printf("y=%6.2f,", yaccel);
	printf("z=%6.2f\n\r", zaccel);
}

int main(void)
{	
	ioinit(); //Setup IO pins and defaults
	SPI_MasterInit();
	config();
	
	while(1)
	{	
		//put your code here
	}
	return (0);
}

void config(void)
{
	cbi(PORTB, CS);
	delay(1);
	SPI_MasterTransmit(0x20); //write to 0x20
	SPI_MasterTransmit(0x37); //normal mode, 400Hz, xyz-enabled
	delay(1);
	sbi(PORTB, CS);
	
	delay(100);
	
	cbi(PORTB, CS);
	delay(1);
	SPI_MasterTransmit(0x21); //write to 0x21
	SPI_MasterTransmit(0x00); //HP filter ON (cuttoff@8MHz) = 0x60, HPfilter OFF = 0x00
	delay(1);
	sbi(PORTB, CS);
	
	delay(100);
	
	cbi(PORTB, CS);
	delay(1);
	SPI_MasterTransmit(0x23); //write to 0x23
	SPI_MasterTransmit(0x30); //24g 
	delay(1);
	sbi(PORTB, CS);
}

//usage: read('hex address'), e.g. read(0x20), reads CTRL_REG1
//returns the value in the address, e.g. read(0x20) = 0x07, returns the default value of 0x20 
uint8_t read(uint8_t addr)
{
	uint8_t regbyte;
	uint8_t read = 0;
	read |= (1<<7); 
	uint8_t byte = 0;
	byte = read | addr;
	
	cbi(PORTB, CS);
	delay(1);
	SPI_MasterTransmit(byte);
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	regbyte = SPDR;
	delay(1);
	sbi(PORTB, CS);
	
	return(regbyte);
}

void accel(void)
{	
	uint16_t xl, xh, yl, yh, zl, zh;
	int16_t tempx, tempy, tempz;
	
	cbi(PORTB, CS);
	SPI_MasterTransmit(0xe8); //read bit | consecutive measure bit | 0x28 = 0xe8
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	xl = SPDR;
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	xh = SPDR;
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	yl = SPDR;
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	yh = SPDR;
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	zl = SPDR;
	SPI_MasterTransmit(0x00); //must keep clock moving, so tx another byte
	zh = SPDR;
	sbi(PORTB, CS);
	
	tempx = (xl|(xh << 8)); //concatenate low and high bits together, load into signed 16 bit
	xaccel = SCALE*tempx;    //multiply by scale factor for g-level
	tempy = (yl|(yh << 8));
	yaccel = SCALE*tempy;
	tempz = (zl|(zh << 8));
	zaccel = SCALE*tempz;
}
////////////////////////////////////////////////////////////////////////////////
///==============Initializations=======================================/////////
////////////////////////////////////////////////////////////////////////////////
void ioinit (void)
{
    //1 = output, 0 = input
    DDRB = 0b11101111; //input on PB4 = MISO 
    DDRD = 0b11110010; //PORTD (RX on PD0)
	
	sbi(PORTB, CS);
    UART_Init(MYUBRR);
	
	// Setting Timer 1
	// normal mode
	TCCR1A = 0x00;
	// Set to clk/8 
	TCCR1B |= (1<<CS11);
	//enable overflow interrupt
	TIMSK1 |= (1<<TOIE1);
	//load timer with a value to optimize for 1 second,
	TCNT1 = 55536;	//(8/8MHz)*(65536bits-55536)=0.01s
	
	sei(); //turn on global interrupts and enable timer
}

void UART_Init(unsigned int ubrr)
{
	int ubrr_new;
	// set baud rate
	ubrr_new = ubrr;
	UBRR0H = ubrr_new>>8;
	UBRR0L = ubrr_new;

	// Enable receiver and transmitter
	UCSR0A = (0<<U2X0); //double UART speed
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);

	// Set frame format: 8 bit, no parity, 1 stop bit,
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);

	stdout = &mystdout; //Required for printf init
}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}
//General short delays
void delay(uint16_t x)
{
  uint8_t y;
  for ( ; x > 100 ; x--){
    for ( y = 0 ; y < 200 ; y++){ 
    asm volatile ("nop");
    }
  }
}
//==========================//
//
//SPI functions
//
//==========================//
void SPI_MasterInit(void)
{
	DDRB = (1<<DDB3)|(1<<DDB5)|(1<<DDB2)|(1<<DDB1);
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
}

void SPI_MasterTransmit(uint8_t address)
{
	SPDR = address;
	while(!(SPSR & (1<<SPIF)));
}