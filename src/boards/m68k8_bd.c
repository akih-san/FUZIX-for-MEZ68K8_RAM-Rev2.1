/*
 *  This source is for PIC18F47Q43 UART, I2C, SPI and TIMER0
 *
 * Base source code is maked by @hanyazou
 *  https://twitter.com/hanyazou
 *
 * Redesigned by Akihito Honda(Aki.h @akih_san)
 *  https://twitter.com/akih_san
 *  https://github.com/akih-san
 *
 *  Target: MEZ68K8_RAM Rev2.0
 *  Date. 2025.3.1
*/

#define BOARD_DEPENDENT_SOURCE

#include "../../src/mez68k8.h"
#include <stdio.h>
#include "../../drivers/SDCard.h"
#include "../../drivers/picregister.h"

#define SPI_PREFIX      SPI_SD
#define SPI_HW_INST     SPI1
#include "../../drivers/SPI.h"

#define ADBUS	B
#define ADR_H	D

#define MA16	A0
#define MA17	A1
#define MA18	A2
#define REQ		A3

// R/#W
#define RW		A4
// CLK
#define NCO1_CLK	A5
// UART
#define TXD		A6
#define RXD		A7

#define LTOE	C0
#define AS		C1
#define BG		C2		// BUS GRANT
#define SCL		C3
#define SDA		C4
#define NMI		C5
#define IRQ		C6
// SPI
#define SPI_SS	C7
#define MOSI	B0
#define SPI_CK	B1
#define MISO	B2

#define RST_68K	E0
#define BR		E1		// BUS request
#define LAT_LE	E2

//SD IO
#define SPI_SD_POCI		MISO	//B2
#define SPI_SD_PICO		MOSI	//B0
#define SPI_SD_CLK		SPI_CK	//B1
#define SPI_SD_SS       SPI_SS	//C7

#define CMD_REQ CLC3OUT

#include "m68k8_cmn.c"

static void bus_release_req(void);
static void reset_ioreq(void);
void (*bus_master_operation)(void);

void sys_init()
{
    base_pin_definition();

	// SPI data and clock pins slew at maximum rate

	SLRCON(SPI_SD_PICO) = 0;
	SLRCON(SPI_SD_CLK) = 0;
	SLRCON(SPI_SD_POCI) = 0;

#define CLK_68k8_10M 327680 // 10MHz

	// 68008 clock(RA5) by NCO FDC mode

	NCO1INC = CLK_68k8_10M;
	NCO1CLK = 0x00;		// Clock source Fosc
	NCO1PFM = 0;		// FDC mode
	NCO1OUT = 1;		// NCO output enable
	NCO1EN = 1;			// NCO enable

	RA5PPS = 0x3f;		// RA5 assign NCO1

//
// UART3 initialize
//
	U3BRG = 416;			// 9600bps @ 64MHz
    U3RXEN = 1;				// Receiver enable
    U3TXEN = 1;				// Transmitter enable

    // UART3 Receiver
	TRIS(RXD) = 1;			// RX set as input
    U3RXPPS = 0x07;			// RA7->UART3:RXD;

    // UART3 Transmitter
	LAT(TXD) = 1;		// Default level
	TRIS(TXD) = 0;		// TX set as output
	PPS(TXD)= 0x26;	// UART3:TXD -> RA6;
	
    U3ON = 1;				// Serial port enable

	// ************ timer0 setup ******************
	T0CON0 = 0x89;	// timer enable, 8bit mode , 1:10 Postscaler 10ms
//	T0CON0 = 0x80;	// timer enable, 8bit mode , 1:1 Postscaler  1ms
//	T0CON0 = 0x84;	// timer enable, 8bit mode , 1:5 Postscaler  5ms
//	T0CON0 = 0x81;	// timer enable, 8bit mode , 1:2 Postscaler  2ms
//	T0CON0 = 0x82;	// timer enable, 8bit mode , 1:3 Postscaler  3ms
//	T0CON0 = 0x83;	// timer enable, 8bit mode , 1:4 Postscaler  4ms
	T0CON1 = 0xa1;	// sorce clk:MFINTOSC (500 kHz), 1:2 Prescaler
	MFOEN = 1;
	TMR0H = 0xff;
	TMR0L = 0x00;

//
// Setup CLC
//
	//========== CLC pin assign ===========
    CLCIN0PPS = 0x03;			// assign RA3(REQ)
    CLCIN5PPS = 0x11;			// assign RC1(#AS)

	//========== CLC3 : Make #BR trigger(CLC3OUT) ==========

	CLCSELECT = 2;		// CLC3 select

    CLCnSEL0 = 5;		// CLCIN5PPS : RC1(#AS)
	CLCnSEL1 = 0;		// CLCIN0PPS : RA3(REQ)
	CLCnSEL2 = 0x35;	// CLC3OUT
	CLCnSEL3 = 127;		// NC

    CLCnGLS0 = 0x01;	// invert RC1(#AS) -> lcxg1(DFF clock)
	CLCnGLS1 = 0x08;	// RA3(A19) -> lcxg2(DFF OR input 2)
    CLCnGLS2 = 0x00;	// 0 -> lcxg3(DFF R)
    CLCnGLS3 = 0x20;	// CLC3OUT -> lcxg4(DFF OR input 1)

    CLCnPOL = 0x00;		// POL=0
    CLCnCON = 0x85;		// 2-Input DFF with R , no interrupt occurs

	reset_ioreq();		// reset DFF

    wait_for_programmer();
}

void setup_sd(void) {
    //
    // Initialize SD Card
    //
    static int retry;
    for (retry = 0; 1; retry++) {
        if (20 <= retry) {
            printf("No SD Card?\n\r");
            while(1);
        }
//        if (SDCard_init(SPI_CLOCK_100KHZ, SPI_CLOCK_2MHZ, /* timeout */ 100) == SDCARD_SUCCESS)
        if (SDCard_init(SPI_CLOCK_100KHZ, SPI_CLOCK_4MHZ, /* timeout */ 100) == SDCARD_SUCCESS)
//        if (SDCard_init(SPI_CLOCK_100KHZ, SPI_CLOCK_8MHZ, /* timeout */ 100) == SDCARD_SUCCESS)
            break;
        __delay_ms(200);
    }
}

void setup_I2C(void) {
	//Source clock enable
	MFOEN = 1;		// MFINTOSC is explicitly enabled
	
	// I2C PIN definition
	LATC4 = 0;			// Set as output
	LATC3 = 0;			// Set as output
	TRISC4 = 0;			// Set as output
	TRISC3 = 0;			// Set as output
	WPUC4 = 1;			// week pull up
	WPUC3 = 1;			// week pull up
	
	RC4PPS = 0x38;			// set RC4PPS for I2C1 SDA
	I2C1SDAPPS = 0x14;		// set RC4 for I2C SDA port

	RC3PPS = 0x37;			// set RC3PPS for I2C SCL
	I2C1SCLPPS = 0x13;		// set RC3 for I2C_SCL port

	//Open-Drain Control
	ODCONC = 0x18;		// RC4 and RC3 are Open-Drain output

	//set I2C Pad Control Register TH=01 (I2C-specific input thresholds)
	RC4I2C = 0x01;		// Std GPIO Slew Rate, Std GPIO weak pull-up, I2C-specific input thresholds
	RC3I2C = 0x01;		// Std GPIO Slew Rate, Std GPIO weak pull-up, I2C-specific input thresholds
//	RC4I2C = 0x41;		// Fast mode (400 kHz), Std GPIO weak pull-up, I2C-specific input thresholds
//	RC3I2C = 0x41;		// Fast mode (400 kHz), Std GPIO weak pull-up, I2C-specific input thresholds
//	RC4I2C = 0xc1;		// Fast mode Plus (1 MHz), Std GPIO weak pull-up, I2C-specific input thresholds
//	RC3I2C = 0xc1;		// Fast mode Plus (1 MHz), Std GPIO weak pull-up, I2C-specific input thresholds

	I2C1_Init();

}

void start_M68K(void)
{

    bus_release_req();

	// Unlock IVT
    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x00;

    // Default IVT base address
    IVTBASE = 0x000008;

    // Lock IVT
    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x01;

	reset_ioreq();			// reset CLC3 (CMD_REQ : CLC3OUT = 0)

	// M68K start
    LAT(RST_68K) = 1;		// Release reset
	TRIS(RST_68K) = 1;		// Set as input
}

static void reset_ioreq(void)
{
	// Release wait (D-FF reset)
	G3POL = 1;
	G3POL = 0;
}

static void set_BR_pin(void)
{
	LAT(BR) = 0;			// request BR
	while( R(BG) ) {}		// wait until bus release
}

static void reset_BR_pin(void)
{
	LAT(BR) = 1;
	while( !R(BG) ) {}		// wait until bus release
}

static void bus_hold_req(void) {
	// Set address bus as output
	TRIS(ADBUS) = 0x00;	// output data bus
	TRIS(ADR_H) = 0x00;	// A8-A15
	TRIS(MA16) = 0;			// Set as output
	TRIS(MA17) = 0;			// Set as output
	TRIS(MA18) = 0;			// Set as output
	LAT(LTOE) = 0;		// Set 74LS373 as active(A7-A0)

	TRIS(RW) = 0;			// output
	TRIS(AS) = 0;			// output

	LAT(RW) = 1;			// SRAM READ mode
	LAT(AS) = 1;			// AS disactive
	LAT(LAT_LE) = 0;		// Latch LE disactive
}

static void bus_release_req(void) {
	// Set address bus as input
	TRIS(ADBUS) = 0xff;	// input data bus
	TRIS(ADR_H) = 0xff;	// A8-A15
	TRIS(MA16) = 1;			// Set as input
	TRIS(MA17) = 1;			// Set as input
	TRIS(MA18) = 1;			// Set as input
	LAT(LTOE) = 1;		// Set 74LS373 as Hiz(A7-A0)

	TRIS(AS) = 1;			// input
	TRIS(RW) = 1;			// input
}

//--------------------------------
// event loop ( PIC MAIN LOOP )
//--------------------------------
void board_event_loop(void) {

	while(1) {
		if (CMD_REQ) {					// CLC3OUT =1
			set_BR_pin();				// HOLD = 1, wait until HOLDA = 1
		    bus_hold_req();				// PIC becomes a busmaster
			(*bus_master_operation)();
			bus_release_req();
			reset_ioreq();				// reset CLC3 (CMD_REQ : CLC3OUT = 0)
			reset_BR_pin();				// HOLD = 0, wait until HOLDA = 0
		}
		// check NMI flag
		if (nmi_sig)  {
			LAT(NMI) = 1;			/* NMI interrupt request */
			nmi_sig = 0;
		}
	}
}

#include "../../drivers/pic18f57q43_spi.c"
#include "../../drivers/SDCard.c"

