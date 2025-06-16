/*
 * Based on main.c by Tetsuya Suzuki 
 * and emuz80_z80ram.c by Satoshi Okue
 * PIC18F47Q43/PIC18F47Q83/PIC18F47Q84 ROM image uploader
 * and UART emulation firmware.
 * This single source file contains all code.
 *
 * Base source code of this firmware is maked by
 * @hanyazou (https://twitter.com/hanyazou) *
 *
 *  Target: MEZ68K8_RAM
 *  Written by Akihito Honda (Aki.h @akih_san)
 *  https://twitter.com/akih_san
 *  https://github.com/akih-san
 *
 *  Date. 2024.5.16
 */

#define INCLUDE_PIC_PRAGMA
#include "../src/mez68k8.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../drivers/utils.h"

static FATFS fs;
static DIR fsdir;
static FILINFO fileinfo;
static FIL rom_fl;

//uint16_t os_flg;	// 0:CP/M 1:FUZIX
uint16_t time_dev;	// 0:Timer0, 1:DS1307
uint8_t	nmi_sig;	// NMI request flag
uint8_t	irq_flg;	// irq flag
uint8_t tmp_buf[2][TMP_BUF_SIZE];
#define BUF_SIZE TMP_BUF_SIZE * 2

debug_t debug = {
    0,  // disk
    0,  // disk_read
    0,  // disk_write
    0,  // disk_verbose
    0,  // disk_mask
};

#define BS	0x08
static char *cpmdir	= "CPMDISKS";
static char *unimon = "UMON_68K.BIN";
static char *basic = "BASIC68K.BIN";
static char *cbios = "CBIOS.BIN";
static char *cpm68k = "CPM68K.BIN";

static char *osdir	= "OSDISKS";
static char *mon = "MON.BIN";
static char *fuzix = "FUZIX.BIN";

static int disk_init(void);
static void chk_dsk(char *);
static int load_program(uint8_t *buf, uint32_t load_adr);
static void setup_cpm(void);
static void setup_fuzix(void);
static void set_tod(void);
static int get_line(char *, int);

static char *board_name = "MEZ68K8_RAM Firmware Rev2.1";

// main routine
void main(void)
{
	int c;
	int	selection;

	irq_flg = 0;
	nmi_sig = 0;	// clear NMI request flag
	sys_init();
	setup_sd();

	printf("Board: %s\n\r", board_name);

	clr_uart_rx_buf();	// clear rx buffer and enable rx interrupt
	timer0_init();		// clear timer value and enable timer0 interrupt
	setup_I2C();
	time_dev = chk_i2cdev();	//0:timer 1:I2C(DS1307)
	mem_init();
	if (disk_init() < 0) while (1);
	
	GIE = 1;             // Global interrupt enable
	TMR0IF =0;			// Clear timer0 interrupt flag
	TMR0IE = 1;			// Enable timer0 interrupt

sel_list:
	selection = 1;		// default : unimon68k
	printf("\n\rSelect:\n\r");
	printf("0:TOD(Time Of Day)\n\r");
	printf("1:unimon68k\n\r");
	printf("2:basic68k\n\r");
	printf("3:CP/M-68K\n\r");
	printf("4:FUZIX\n\r");
	printf("? ");
	while (1) {
		c = (uint8_t)getch();  // Wait for input char
		if ( c == '0' || c == '1' || c == '2' || c == '3' || c == '4' ) {
			putch((char)c);
			putch((char)BS);
			selection = c - (int)'0';
		}
		if ( c == 0x0d || c == 0x0a ) break;
	}
	printf("\n\r");

	// set default bus master operation function
	bus_master_operation = cpm_bus_master_operation;
	c = 0;
	switch (selection) {
		case 0:		// set Time of day
			set_tod();
			goto sel_list;
		case 1:		// unimon68k
			fileinfo.fname[0] = 0;		// set No directory
			c = load_program((uint8_t *)unimon, UNIMON_OFF);
			irq_flg = 1;				// using irq interruption
			break;
		case 2:		// basic68k
			fileinfo.fname[0] = 0;		// set No directory
			c = load_program((uint8_t *)basic, BASIC_OFF);
			irq_flg = 1;				// using irq interruption
			break;
		case 3:	// CP/M-68K
			chk_dsk(cpmdir);
			if ( open_cpmdsk(&fileinfo) < 0 ) {
		        printf("No boot disk.\n\r");
				while(1);
			}
			setup_cpm();
			irq_flg = 0;
			break;
		default:	// FUZIX
			chk_dsk(osdir);
			if ( open_fxdsk(&fileinfo) < 0 ) {
		        printf("No boot disk.\n\r");
				while(1);
			}
			setup_fuzix();
			irq_flg = 1;				// using irq interruption
	}
	if ( c ) {
		printf("Program File Load Error.\r\n");
		while(1);
	}
	
	//
    // Start MC68008
    //
	printf("Use NCO1 %2.3fMHz\r\n",NCO1INC * 30.5175781 / 1000000);
    printf("\n\r");
	
    start_M68K();
	board_event_loop();
}

static uint16_t get_dn( uint8_t *str, uint16_t *cnt ) {
	uint16_t n, er;
	uint8_t s;

	er = 0xffff;	// error flag
	n = *cnt = 0;
	while( *str ) {
		s = *str++;
		*cnt += 1;
		if ( s < (uint8_t)'0' || s > (uint8_t)'9' ) break;
		n = n*10+(uint16_t)(s-(uint8_t)'0');
		er = 0;
	}
	if ( er == 0xffff ) {
		n = er;
		return n;
	}

	// skip non value char or detect delimiter(0)
	while ( *str < (uint8_t)'0' || *str > (uint8_t)'9' ) {
		if ( *str == 0 ) break;
		str++;
		*cnt += 1;
	}
	return n;
}

static int get_todval( uint8_t *str, uint16_t *yh, uint16_t *mm, uint16_t *ds ) {
	uint16_t n, cnt;
	
	n = get_dn( str, &cnt );
	if ( n == 0xffff ) return -1;
	*yh = n;
	str += cnt;
	
	n = get_dn( str, &cnt );
	if ( n == 0xffff ) return -1;
	*mm = n;
	str += cnt;
	
	n = get_dn( str, &cnt );
	if ( n == 0xffff ) return -1;
	*ds = n;
	
	return 0;
}

//
// set time of day
// time_dev 0:timer 1:I2C(DS1307)
//
// DS1307 format
// rtc[0] : seconds (BCD) 00-59
// rtc[1] : minuts  (BCD) 00-59
// rtc[2] : hours   (BCD) 00-23 (or 1-12 +AM/PM)
// rtc[3] : day     (BCD) week day 01-07
// rtc[4] : date    (BCD) 01-31
// rtc[5] : month   (BCD) 01-12
// rtc[6] : year    (BCD) 00-99 : range : (19)80-(19)99, (20)00-(20)79

static void set_tod(void) {

	uint8_t *s, commit;
	uint16_t year, month, date;
	uint16_t yh, mm, ds;

	commit = 0;
	// Set date
	// convert number of days to year, month and date
	cnv_ymd(tim_pb.TIM_DAYS, &year, &month, &date );
	if (year >= 80) year += 1900;
	else year += 2000;

re_inp1:
	printf("Date %04d/%02d/%02d = ", year, month, date);
	s = &tmp_buf[0][0];
	if ( get_line((char *)s, 80) != 0 ) {
		if ( get_todval( s, &yh, &mm, &ds ) ) {
			printf("\r\n");
			goto re_inp1;
		}
		if (yh < 1980 || yh > 2079 ) {
			printf("\r\n");
			goto re_inp1;
		}
		if ( mm > 12 || mm == 0 ) {
			printf("\r\n");
			goto re_inp1;
		}
		if ( mm == 2 && ds == 29 ) {
			if ( chk_leap(yh) ) goto skip_ds;
			printf("\r\n");
			goto re_inp1;
		}
		if ( ds > mtod[mm-1] || ds == 0) {
			printf("\r\n");
			goto re_inp1;
		}
skip_ds:
		year = yh;
		month = mm;
		date = ds;
		commit = 1;
	}
	// set time
	yh = tim_pb.TIM_HRS;
	mm = tim_pb.TIM_MINS;
	ds = tim_pb.TIM_SECS;
	
re_inp2:
	printf("Time %02d:%02d:%02d = ", tim_pb.TIM_HRS, tim_pb.TIM_MINS, tim_pb.TIM_SECS);
	s = &tmp_buf[0][0];
	if ( get_line((char *)s, 80) != 0 ) {
		if ( get_todval( s, &yh, &mm, &ds ) ) {
			printf("\r\n");
			goto re_inp2;
		}
		if ( yh  > 23 ) {
			printf("\r\n");
			goto re_inp2;
		}
		if ( mm > 59 ) {
			printf("\r\n");
			goto re_inp2;
		}
		if ( ds > 59 ) {
			printf("\r\n");
			goto re_inp2;
		}
		TMR0IE = 0;			// disable timer0 interrupt
		tim_pb.TIM_HRS = (uint8_t)yh;
		tim_pb.TIM_MINS = (uint8_t)mm;
		tim_pb.TIM_SECS = (uint8_t)ds;
		datcnv_tim_rtc();
		TMR0IE = 1;			// Enable timer0 interrupt
		commit = 1;
	}
	if ( commit ) {
		// convert year, month and date to number of days from 1980
		tim_pb.TIM_DAYS = days_from_1980(year, month, date);
		if ( time_dev ) {		// DS1307 ready!
			datcnv_tim_rtc();	// convert time data to DS1307 format
			// write to RTC
			if (write_I2C(DS1307, 0, 7, &rtc[0] ) == 0xFF) {
				printf("DS1307 Wite I2C error!\r\n");
				return;
			}
		}
		printf("\r\nSet Date(%04d/%02d/%02d)\r\n", year, month, date);
		printf("Set Time(%02d:%02d:%02d)\r\n", yh, mm, ds);
	}
}

static int get_line(char *s, int length) {
	char n;
	int c;
	
	for (c=0;;) {
		n = (char)getch();
		if ( n == BS ) {
			if ( c > 0) {
				putch(BS);
				putch(' ');
				putch(BS);
				c--;
				s--;
			}
			continue;
		}
		if ( n == 0x0d || n == 0x0a ) {
			*s = 0x00;
			printf("\r\n");
			return c;
		}
		if ( c <= length-1 ) {
			putch(n);
			if ( n >='a' && n <='z' ) n -= 0x20;		// lower to upper
			*s++ = n;
			c++;
		}
	}
	return c;
}

//
// load program from SD card
//
static int load_program(uint8_t *fname, uint32_t load_adr) {
	
	FRESULT		fr;
	void		*rdbuf;
	UINT		btr, br;
	uint16_t	cnt, size;
	uint32_t	adr;

	TCHAR	buf[30];

	rdbuf = (void *)&tmp_buf[0][0];		// program load work area(512byte)
	
	sprintf((char *)buf, "%s", fname);

	fr = f_open(&rom_fl, buf, FA_READ);
	if ( fr != FR_OK ) return((int)fr);

	adr = load_adr;
	cnt = size = (uint16_t)f_size(&rom_fl);				// get file size
	btr = BUF_SIZE;										// default 512byte
	while( cnt ) {
		fr = f_read(&rom_fl, rdbuf, btr, &br);
		if (fr == FR_OK) {
			write_sram(adr, (uint8_t *)rdbuf, (unsigned int)br);
			adr += (uint32_t)br;
			cnt -= (uint16_t)br;
			if (btr > (UINT)cnt) btr = (UINT)cnt;
		}
		else break;
	}
	if (fr == FR_OK) {
		printf("Load %s : Adr = %06lx, Size = %04x\r\n", fname, load_adr, size);
	}
	f_close(&rom_fl);
	return((int)fr);
}

//
// mount SD card
//
static int disk_init(void)
{
    if (f_mount(&fs, "0://", 1) != FR_OK) {
        printf("Failed to mount SD Card.\n\r");
        return -2;
    }

    return 0;
}

//
// check dsk
//
static void chk_dsk(char *dir)
{
    int sig;
    uint8_t c;

    //
    // Select disk image folder
    //
    if (f_opendir(&fsdir, "/")  != FR_OK) {
        printf("Failed to open SD Card.\n\r");
		while(1);
    }

	sig = 0;
	f_rewinddir(&fsdir);
	while (f_readdir(&fsdir, &fileinfo) == FR_OK && fileinfo.fname[0] != 0) {
		if (strcmp(fileinfo.fname, dir) == 0) {
			sig = 1;
			printf("Detect %s\n\r", fileinfo.fname);
			break;
		}
	}
	f_closedir(&fsdir);
	
	if ( !sig ) {
		printf("No %s directory found.\r\n", dir);
		while(1);
	}
	return;
}

static void setup_cpm(void) {
	
	const TCHAR	buf[30];
	int flg;

	cpmio_init();
	printf("\n\r");

	sprintf((char *)buf, "%s/%s", fileinfo.fname, cpm68k);
	flg = load_program((uint8_t *)buf, CPM68K_OFF);
	if (!flg) {
		sprintf((char *)buf, "%s/%s", fileinfo.fname, cbios);
		flg = load_program((uint8_t *)buf, CBIOS_OFF);
	}
	if ( flg ) {
		printf("Program File Load Error.\r\n");
		while(1);
	}
}
static void setup_fuzix(void) {
	
	const TCHAR	buf[30];
	int flg;

	// set FUZIX operation 
	bus_master_operation = fx_bus_master_operation;

	picif_init();
	printf("\n\r");

	sprintf((char *)buf, "%s/%s", fileinfo.fname, fuzix);
	flg = load_program((uint8_t *)buf, FUZIX_OFF);
	if (!flg) {
		sprintf((char *)buf, "%s/%s", fileinfo.fname, mon);
		flg = load_program((uint8_t *)buf, MON_OFF);
	}
	if ( flg ) {
		printf("Program File Load Error.\r\n");
		while(1);
	}
}
