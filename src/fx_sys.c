/*
 * Base source code is maked by @hanyazou
 *  https://twitter.com/hanyazou
 *
 * Redesigned by Akihito Honda(Aki.h @akih_san)
 *  https://twitter.com/akih_san
 *  https://github.com/akih-san
 *
 *  Target: PIC18F57QXX/PIC18F47QXX
 *  Date. 2024.4.20
 */

#include "../src/mez68k8.h"
#include <stdio.h>
#include <assert.h>

#include "../fatfs/ff.h"
#include "../drivers/utils.h"

#define SECTOR_SIZE      512

#define RETURN_TBL 2		/* bytes of return parameter */

static FIL fxfile;

// request table
static crq_hdr_fx req_tbl;

void picif_init(void) {
	int	i;
	
	req_tbl.UREQ_COM = 0;
	req_tbl.CREQ_COM = 0;
}

//
// Open disk images
//
int open_fxdsk(FILINFO *fileinfo) {
	
	char * const buf = (char *)tmp_buf[0];

	sprintf(buf, "%s/DISK.IMG", fileinfo->fname);
	if (f_open(&fxfile, buf, FA_READ|FA_WRITE) == FR_OK) {
		printf("Detect %s/DISK.IMG ... OK\n\r",fileinfo->fname);
	    return 0;
	}
	else return -4;
}

static int seek_disk(void) {
	FRESULT fres;

	// check drive (only 0)
	if ( req_tbl.disk_drive ) {
		printf("Drive(%d) ERROR %d\n\r", req_tbl.disk_drive, fres);
		return(-1);
	}

	if ((fres = f_lseek(&fxfile, req_tbl.lba * SECTOR_SIZE)) != FR_OK) {
		printf("f_lseek(): ERROR %d\n\r", fres);
		return(-1);
	}
	return 0;
}

static int write_sd(void) {
	unsigned int n;
	FRESULT fres;
	uint8_t cnt;
	
	cnt = req_tbl.blocks;
	while( cnt-- ) {
		if (seek_disk()) return -1;

		// transfer write data from SRAM to the buffer
		read_sram(req_tbl.data_dma, &tmp_buf[0][0], SECTOR_SIZE);

		if (DEBUG_DISK_WRITE && DEBUG_DISK_VERBOSE && !(debug.disk_mask & (1 << req_tbl.disk_drive))) {
			util_hexdump_sum("buf: ", &tmp_buf[0][0], SECTOR_SIZE);
		}

		// write buffer to the DISK
		if ((fres = f_write(&fxfile, &tmp_buf[0][0], SECTOR_SIZE, &n)) != FR_OK || n != SECTOR_SIZE) {
			printf("f_write(): ERROR res=%d, n=%d\n\r", fres, n);
			return -1;
		}
		else if ((fres = f_sync(&fxfile)) != FR_OK) {
			printf("f_sync(): ERROR %d\n\r", fres);
			return -1;
		}
			// update next wrhite parameter
		req_tbl.data_dma += SECTOR_SIZE;
		req_tbl.lba++;
	}
	return 0;
}

static int read_sd(void) {
	unsigned int n;
	FRESULT fres;
	uint8_t cnt;
	
	cnt = req_tbl.blocks;
	while( cnt-- ) {
		if (seek_disk()) return -1;

		// read from the DISK
		if ((fres = f_read(&fxfile, &tmp_buf[0][0], SECTOR_SIZE, &n)) != FR_OK || n != SECTOR_SIZE) {
			printf("f_read(): ERROR res=%d, n=%d\n\r", fres, n);
			return -1;
		}
		else {
			if (DEBUG_DISK_READ && DEBUG_DISK_VERBOSE && !(debug.disk_mask & (1 << req_tbl.disk_drive))) {
				util_hexdump_sum("buf: ", &tmp_buf[0][0], SECTOR_SIZE);
			}
			// transfer read data to SRAM
			write_sram((uint32_t)req_tbl.data_dma, &tmp_buf[0][0], SECTOR_SIZE);

			#ifdef MEM_DEBUG
			printf("f_read(): SRAM address(%08lx)\n\r", req_tbl.data_dma);
			read_sram(req_tbl.data_dma, &tmp_buf[0][0], SECTOR_SIZE);
			util_hexdump_sum("RAM: ", &tmp_buf[0][0], SECTOR_SIZE);
			#endif  // MEM_DEBUG
			
			// update next wrhite parameter
			req_tbl.data_dma += SECTOR_SIZE;
			req_tbl.lba++;
		}
	}
	return 0;
}

static void dsk_err(void) {
	req_tbl.UNI_CHR = 1;
}

/* ------ RTC I/F ----------------
* DS1307 format
* rtc[0] : seconds (BCD) 00-59
* rtc[1] : minuts  (BCD) 00-59
* rtc[2] : hours   (BCD) 00-23 (or 1-12 +AM/PM)
* rtc[3] : day     (BCD) week day 01-07
* rtc[4] : date    (BCD) 01-31
* rtc[5] : month   (BCD) 01-12
* rtc[6] : year    (BCD) 00-99 : range : (19)80-(19)99, (20)00-(20)79

fuzix Format Type BCD
fx_rtc[0] : year : high 19, 20
fx_rtc[1] : year : low 19(70 - 99), 20(00 - 69)
fx_rtc[2] : month ( 0 based )
fx_rtc[3] : date (1 - 31)
fx_rtc[4] : hour (0 - 23)
fx_rtc[5] : minute(0 - 59)
fx_rtc[6] : second(0 - 59)

(unsigned int)req_tbl.CBI_CHR : request data length from 68K

*/
static uint8_t rd_time() {

	uint32_t trans_adr;
	uint16_t year;
	uint8_t month;
	uint8_t fx_rtc[8], *p;

	p = fx_rtc;
	trans_adr = req_tbl.data_dma;

	//read TIME
	if (time_dev) { 				//DS1307
		if( cnv_rtc_tim() ) return 1;
	}
	else datcnv_tim_rtc();			//TIMER0

	year = (uint16_t)rtc[6];
	if (year >= 0x70) year = 0x1900 | year;
	else year = 0x2000 | year;
	
	*p++ = (uint8_t)(year >> 8);	
	*p++ = (uint8_t)year;

	month = rtc[5];
	month--;						/* 0 based */
	if ((month & 0x0F) > 9) month -= 0x06;
	*p++ = month;

	*p++ = rtc[4];	/* Day of month */
	*p++ = rtc[2];	/* Hour */
	*p++ = rtc[1];	/* Minute */
	*p = rtc[0];	/* Second */

	// write time date to TPB
	write_sram( trans_adr, fx_rtc, (unsigned int)req_tbl.CBI_CHR );
	return 0;
}

static uint8_t wr_time() {

	uint32_t trans_adr;
	uint8_t fx_rtc[8];
	
	trans_adr = req_tbl.data_dma;
	read_sram( trans_adr, fx_rtc, (unsigned int)req_tbl.CBI_CHR );

	rtc[0] = fx_rtc[6];		// second
	rtc[1] = fx_rtc[5];		// minute
	rtc[2] = fx_rtc[4];		// hour
	rtc[4] = fx_rtc[3];		// date
	rtc[5] = fx_rtc[2];		// month
	rtc[6] = fx_rtc[1];		// year (19)80 - (20)79
	cvt_bcd_bin();

	// set time
	if (time_dev) { //DS1307
		// write to RTC
		if (write_I2C(DS1307, 0, 7, &rtc[0] ) == 0xFF) return 1;
	}
	return 0;
}	

static void unimon_console(void) {

	uint8_t *buf, c;
	uint16_t cnt;

	switch (req_tbl.UREQ_COM) {
		// CONIN
		case CONIN_REQ:
			req_tbl.UNI_CHR = (uint8_t)getch();
			break;
		// CONOUT
		case CONOUT_REQ:
			putch((char)req_tbl.UNI_CHR);		// Write data
			break;
		// CONST
		case CONST_REQ:
			req_tbl.UNI_CHR = (uint8_t)(rx_cnt !=0);
			break;
		case STROUT_REQ:
			buf = tmp_buf[0];
			cnt = (uint16_t)req_tbl.UNI_CHR;
			// get string
			read_sram(req_tbl.STR_adr, buf, cnt);
			while( cnt ) {
				putch( *buf++);
				cnt--;
			}
			break;
		case CONIN_REQ1:
			if ( rx_cnt ) req_tbl.UNI_CHR = (uint8_t)getch();
			else req_tbl.UNI_CHR = 0;
			break;
		case CONOUT_REQ1:
			c = req_tbl.UNI_CHR;
			req_tbl.UNI_CHR = out_chr(c);
			break;
		case STRIN_REQ:

			buf = tmp_buf[0];
			cnt = (uint16_t)get_str((char *)buf, req_tbl.UNI_CHR);
			req_tbl.UNI_CHR = (uint8_t)cnt;
			if (cnt) write_sram(req_tbl.STR_adr, buf, (unsigned int)cnt);
			break;
		case CLR_IRQ:
			clr_int(req_tbl.UNI_CHR);
			break;
		default:
			printf("UNKNOWN unimon CMD(%02x)\r\n", req_tbl.UREQ_COM);
	}
	req_tbl.UREQ_COM = 0;	// clear unimon request
}

//
// bus master handling
// this fanction is invoked at main() after HOLDA = 1
//
// bioreq_ubuffadr = top address of unimon
//
//  ---- request command to PIC
// UREQ_COM = 1 ; CONIN  : return char in UNI_CHR
//          = 2 ; CONOUT : UNI_CHR = output char
//          = 3 ; CONST  : return status in UNI_CHR
//                       : ( 0: no key, 1 : key exist )
//          = 4 ; STROUT : string address = (PTRSAV, PTRSAV_SEG)
//          = 5 ; DISK READ
//          = 6 ; DISK WRITE
//          = 0 ; request is done( return this flag from PIC )
//                return status is in CBI_CHR (unimon : UNI_CHR);
//debug
//printf("\r\nCOM(%02x), drive(%02x), track(%04x), sec(%04x), dma(%08lx)\r\n",
//   req_tbl.CREQ_COM, req_tbl.disk_drive, req_tbl.disk_track, req_tbl.disk_sector, req_tbl.data_dma);
//debug

void fx_bus_master_operation(void) {
	uint8_t *buf, c;
	uint16_t cnt;

	// read request from MC68008
	read_sram(bioreq_ubuffadr, (uint8_t *)&req_tbl, (unsigned int)sizeof(crq_hdr_fx));

	if (req_tbl.UREQ_COM) {
		unimon_console();
		// write end request to SRAM for MC68008
		write_sram(bioreq_ubuffadr, (uint8_t *)&req_tbl, RETURN_TBL);	// 2bytes
	}
	else {
		switch (req_tbl.CREQ_COM) {
			// CONIN
			case CONIN_REQ:
				req_tbl.CBI_CHR = (uint8_t)getch();
				break;
			// CONOUT
			case CONOUT_REQ:
				putch((char)req_tbl.CBI_CHR);		// Write data
				break;
			// CONST
			case CONST_REQ:
				req_tbl.CBI_CHR = (rx_cnt !=0) ? 255 : 0;
				break;
			case STROUT_REQ:
				buf = tmp_buf[0];
				cnt = (uint16_t)req_tbl.CBI_CHR;
				// get string
				read_sram(req_tbl.data_dma, buf, cnt);
				while( cnt ) {
					putch( *buf++);
					cnt--;
				}
				break;
			case REQ_DREAD:
				if ( read_sd() ) dsk_err();
				break;
			case REQ_DWRITE:
				if ( write_sd() ) dsk_err();
				break;
			case READ_TIME:
				req_tbl.CBI_CHR = rd_time();
				break;
			case SET_TIME:
				req_tbl.CBI_CHR = wr_time();
				break;
			case CONIN_REQ1:
				if ( rx_cnt ) req_tbl.CBI_CHR = (uint8_t)getch();
				else req_tbl.CBI_CHR = 0;
				break;
			case CONOUT_REQ1:
				c = req_tbl.CBI_CHR;
				req_tbl.CBI_CHR = out_chr(c);
				break;
			case STRIN_REQ:
				buf = tmp_buf[0];
				cnt = (uint16_t)get_str((char *)buf, req_tbl.CBI_CHR);
				req_tbl.CBI_CHR = (uint8_t)cnt;
				if (cnt) write_sram(req_tbl.data_dma, buf, (unsigned int)cnt);
				break;
			case CLR_IRQ:
				clr_int(req_tbl.CBI_CHR);
		}
		req_tbl.CREQ_COM = 0;	// clear cbios request
		// write end request to SRAM for MC68008
		write_sram(bioreq_cbuffadr, (uint8_t *)&req_tbl.CREQ_COM, RETURN_TBL);	// 2bytes
	}

}

