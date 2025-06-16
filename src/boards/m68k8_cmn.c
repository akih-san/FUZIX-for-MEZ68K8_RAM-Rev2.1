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

#include "../mez68k8.h"
#include "../../drivers/utils.h"

// console input buffers
static unsigned char rx_buf[U3B_SIZE];		//UART Rx ring buffer
static unsigned int rx_wp, rx_rp;
unsigned int rx_cnt;

TPB tim_pb;			// TIME device parameter block

// RTC DS1307 format
// rtc[0] : seconds (BCD) 00-59
// rtc[1] : minuts  (BCD) 00-59
// rtc[2] : hours   (BCD) 00-23 (or 1-12 when bit6=1. bit5: AM(0)/PM(1) )
// rtc[3] : day     (BCD) week day 01-07
// rtc[4] : date    (BCD) 01-31
// rtc[5] : month   (BCD) 01-12
// rtc[6] : year    (BCD) 00-99 : range : (19)80-(19)99, (20)00-(20)79
uint8_t rtc[7];

static uint8_t cnt_sec;		// sec timer (1000ms = 10ms * 100)

//initialize TIMER0 & TIM device parameter block
void timer0_init(void) {
	
	uint16_t year, month, date;

	cnt_sec = 0;	// set initial adjust timer counter
	tim_pb.TIM_DAYS = TIM20250601;		//set 2025.06.01
	tim_pb.TIM_MINS = 0;
	tim_pb.TIM_HRS = 0;
	tim_pb.TIM_SECS = 0;
	tim_pb.TIM_HSEC = 0;

	// convert number of days to year, month and date
	cnv_ymd(tim_pb.TIM_DAYS, &year, &month, &date );
	// convert bin to BCD
	rtc[0] = cnv_bcd(tim_pb.TIM_SECS);
	rtc[1] = cnv_bcd(tim_pb.TIM_MINS);
	rtc[2] = cnv_bcd(tim_pb.TIM_HRS);
	rtc[4] = cnv_bcd((uint8_t)date);
	rtc[5] = cnv_bcd((uint8_t)month);
	rtc[6] = cnv_bcd((uint8_t)year);

//	TMR0IF =0;	// Clear timer0 interrupt flag
//	TMR0IE = 1;	// Enable timer0 interrupt
}

void cvt_bcd_bin(void) {
	uint16_t year, month, date;
	// convert BCD to bin
	rtc[0] &=0x7f;		// mask bit 7(CH: clock disable bit)

	TMR0IE = 0;			// disable timer0 interrupt
	tim_pb.TIM_SECS = cnv_byte(rtc[0]);
	tim_pb.TIM_MINS = cnv_byte(rtc[1]);
	tim_pb.TIM_HRS  = cnv_byte(rtc[2]);
	date  = (uint16_t)cnv_byte(rtc[4]);
	month = (uint16_t)cnv_byte(rtc[5]);
	year  = (uint16_t)cnv_byte(rtc[6]);
	if (year >= 80) year += 1900;
	else year += 2000;

	// convert year, month and date to number of days from 1980
	tim_pb.TIM_DAYS = days_from_1980(year, month, date);
	TMR0IE = 1;			// Enable timer0 interrupt
}

int cnv_rtc_tim(void) {
	if ( read_I2C(DS1307, 0, 7, &rtc[0]) == 0xFF) return 1;
	cvt_bcd_bin();
	return 0;
}

void datcnv_tim_rtc(void) {
	uint16_t year, month, date;
	
	cnv_ymd(tim_pb.TIM_DAYS, &year, &month, &date );
	// convert bin to BCD
	rtc[0] = cnv_bcd(tim_pb.TIM_SECS);
	rtc[1] = cnv_bcd(tim_pb.TIM_MINS);
	rtc[2] = cnv_bcd(tim_pb.TIM_HRS);
	rtc[4] = cnv_bcd((uint8_t)date);
	rtc[5] = cnv_bcd((uint8_t)month);
	rtc[6] = cnv_bcd((uint8_t)year);
}

//
// define interrupt
//
// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

////////////// TIMER0 vector interrupt ////////////////////////////
//TIMER0 interrupt
/////////////////////////////////////////////////////////////////
void __interrupt(irq(TMR0),base(8)) TIMER0_ISR(){

	TMR0IF =0; // Clear timer0 interrupt flag

//	if (!time_dev) {
		if (++cnt_sec == 100) {
			cnt_sec = 0;
			
			if( ++tim_pb.TIM_SECS == 60 ) {
				tim_pb.TIM_SECS = 0;
				if ( ++tim_pb.TIM_MINS == 60 ) {
					tim_pb.TIM_MINS = 0;
					if ( ++tim_pb.TIM_HRS == 24 ) {
						tim_pb.TIM_HRS = 0;
						tim_pb.TIM_DAYS++;
					}
				}
			}
			tim_pb.TIM_HSEC = 0;
		}
//	}

	// IRQ request
	if (irq_flg && !CMD_REQ && R(BG)) LAT(IRQ) = 1;			// A level 5 interrupt request

}

void clr_int(uint8_t s) {
	if (!s) LAT(NMI) = 0;				// release level 5 interrupt signal
	else LAT(IRQ) = 0;				// release level 5 interrupt signal
}

////////////// UART3 Receive interrupt ////////////////////////////
// UART3 Rx interrupt
// PIR9 (bit0:U3RXIF bit1:U3TXIF)
/////////////////////////////////////////////////////////////////
void __interrupt(irq(U3RX),base(8)) URT3Rx_ISR(){

	unsigned char rx_data;

	rx_data = U3RXB;			// get rx data

	if ( irq_flg && rx_data == CTL_Q ) {
		nmi_sig = 1;
		rx_wp = 0;
		rx_rp = 0;
		rx_cnt = 0;
	}
	else if (rx_cnt < U3B_SIZE) {
		rx_buf[rx_wp] = rx_data;
		rx_wp = (rx_wp + 1) & (U3B_SIZE - 1);
		rx_cnt++;
	}
}

// UART3 Transmit
// if TXIF is busy, return status BUSY(not ZERO)
//
uint8_t out_chr(char c) {
	if (!U3TXIF) return 1;		// retrun BUSY
    U3TXB = c;                  // Write data
	return 0;
}

// UART3 Transmit
void putch(char c) {
    while(!U3TXIF);             // Wait or Tx interrupt flag set
    U3TXB = c;                  // Write data
}

// UART3 Recive
int getch(void) {
	char c;

	while(!rx_cnt);             // Wait for Rx interrupt flag set
	GIE = 0;                // Disable interrupt
	c = rx_buf[rx_rp];
	rx_rp = (rx_rp + 1) & ( U3B_SIZE - 1);
	rx_cnt--;
	GIE = 1;                // enable interrupt
    return c;               // Read data
}

// clear rx buffer and enable rx interrupt
void clr_uart_rx_buf(void) {
	rx_wp = 0;
	rx_rp = 0;
	rx_cnt = 0;
    U3RXIE = 1;          // Receiver interrupt enable
}

unsigned int get_str(char *buf, uint8_t cnt) {
	unsigned int c, i;
	
	U3RXIE = 0;					// disable Rx interruot
	i = ( (unsigned int)cnt > rx_cnt ) ? rx_cnt : (unsigned int)cnt;
	c = i;
	while(i--) {
		*buf++ = rx_buf[rx_rp];
		rx_rp = (rx_rp + 1) & ( U3B_SIZE - 1);
		rx_cnt--;
	}
	U3RXIE = 1;					// enable Rx interruot
	return c;
}

static void base_pin_definition()
{
    // System initialize
    OSCFRQ = 0x08;      // 64MHz internal OSC

	// Disable analog function
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    ANSELD = 0x00;
    ANSELE0 = 0;
    ANSELE1 = 0;
    ANSELE2 = 0;

    // RESET output pin
	WPU(RST_68K) = 0;		// disable week pull up
	LAT(RST_68K) = 0;		// Reset
    TRIS(RST_68K) = 0;		// Set as output

	// R/W
	WPU(RW) = 1;		// week pull up
	LAT(RW) = 1;		// Active state READ-High
	TRIS(RW) = 0;		// Set as output

	// #LAT_CE
	WPU(LTOE) = 0;		// disable pull up
	LAT(LTOE) = 0;		// 74LS373 enable output
	TRIS(LTOE) = 0;		// Set as onput

	// LAT_LE
	WPU(LAT_LE) = 0;	// disable week pull up
	LAT(LAT_LE) = 0;	// Disactive state Low
    TRIS(LAT_LE) = 0;	// Set as output

	// Init address LATCH to 0
	// Address/DATA bus AD7-AD0
    WPU(ADBUS) = 0xff;       // Week pull up
    LAT(ADBUS) = 0x00;
    TRIS(ADBUS) = 0x00;      // Set as output

	// Address bus A15-A8 pin
    WPU(ADR_H) = 0xff;       // Week pull up
    LAT(ADR_H) = 0x00;
    TRIS(ADR_H) = 0x00;      // Set as output

	WPU(NMI) = 0;     // Disable pull up
	LAT(NMI) = 0;     // NMI=0
    TRIS(NMI) = 0;    // Set as output

	WPU(IRQ) = 0;     // Disable pull up
	LAT(IRQ) = 0;     // IRQ=0
    TRIS(IRQ) = 0;    // Set as output

	WPU(MA16) = 0;     // Disable pull up
	LAT(MA16) = 0;     // init A16=0
    TRIS(MA16) = 0;    // Set as output

	WPU(MA17) = 0;     // Disable pull up
	LAT(MA17) = 0;     // init A17=0
    TRIS(MA17) = 0;    // Set as output

	WPU(MA18) = 0;     // Disable pull up
	LAT(MA18) = 0;     // init A18=0
    TRIS(MA18) = 0;    // Set as output

	// REQ : PIC BR trigger
	WPU(REQ) = 0;     // Disable pull up
    TRIS(REQ) = 1;    // input

	// address strobe
	WPU(AS) = 1;		// Week pull up
	LAT(AS) = 1;
	TRIS(AS) = 0;		// Set as onput

	// #BR output pin
	// Refar to CLC3
	WPU(BR) = 1;	// week pull up
    LAT(BR) = 1;
    TRIS(BR) = 0;	// Set as output

	// #BG input pin
	WPU(BG) = 1;		// week pull up
    TRIS(BG) = 1;		// Set as input

	// SPI_SS
	WPU(SPI_SS) = 1;		// SPI_SS Week pull up
	LAT(SPI_SS) = 1;		// set SPI disable
	TRIS(SPI_SS) = 0;		// Set as onput

	WPU(NCO1_CLK) = 0;		// disable week pull up
	LAT(NCO1_CLK) = 1;		// init CLK = 1
    TRIS(NCO1_CLK) = 0;		// set as output pin
	
}

//union address_bus_u {
//    uint32_t w;             // 32 bits Address
//    struct {
//        uint8_t ll;        // Address L low
//        uint8_t lh;        // Address L high
//        uint8_t hl;        // Address H low
//        uint8_t hh;        // Address H high
//    };
//};
void write_sram(uint32_t addr, uint8_t *buf, unsigned int len)
{
    union address_bus_u ab;
    unsigned int i;

	ab.w = addr;
	i = 0;

	while( i < len ) {

	    LAT(ADBUS) = ab.ll;
		LAT(ADR_H) = ab.lh;
		LAT(MA16) = ab.hl & 0x01;
		LAT(MA17) = (ab.hl & 0x02) >> 1;
		LAT(MA18) = (ab.hl & 0x04) >> 2;
		
		// 74LS373 Latch address A0 - A7
		LAT(LAT_LE) = 1;
		LAT(LAT_LE) = 0;

		LAT(AS) = 0;			// activate SRAM #CE=0
		LAT(ADBUS) = ((uint8_t*)buf)[i];
		LAT(RW) = 0;			// activate /WE
		LAT(RW) = 1;			// deactivate /WE
		LAT(AS) = 1;			// disactivate SRAM #CE=1
		
		i++;
		ab.w++;
    }
}

void read_sram(uint32_t addr, uint8_t *buf, unsigned int len)
{
    union address_bus_u ab;
    unsigned int i;

	ab.w = addr;

	i = 0;
	while( i < len ) {
	
		LAT(ADBUS) = ab.ll;
		LAT(ADR_H) = ab.lh;
		LAT(MA16) = ab.hl & 0x01;
		LAT(MA17) = (ab.hl & 0x02) >> 1;
		LAT(MA18) = (ab.hl & 0x04) >> 2;

		// Latch address A0 - A7
		LAT(LAT_LE) = 1;
		LAT(LAT_LE) = 0;

		TRIS(ADBUS) = 0xff;					// Set as input
		LAT(AS) = 0;						// activate SRAM #CE=0
		ab.w++;								// Ensure bus data setup time from HiZ to valid data
		((uint8_t*)buf)[i] = PORT(ADBUS);	// read data
		i++;
		LAT(AS) = 1;						// disactivate SRAM #CE=1
		TRIS(ADBUS) = 0x00;					// Set as output
    }
}

static void wait_for_programmer()
{
    //
    // Give a chance to use PRC (RB6) and PRD (RB7) to PIC programer.
    //
    printf("\n\r");
    printf("wait for programmer ...\r");
    __delay_ms(200);
    printf("                       \r");

    printf("\n\r");
}
