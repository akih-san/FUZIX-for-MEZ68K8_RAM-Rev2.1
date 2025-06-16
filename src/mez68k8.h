/*
 * CP/M-86 and MS-DOS for MEZ88_47Q
 * This firmware is only for PICF47QXX.
 * This firmware can run CPM-86 or MS-DOS on CPU i8088/V20.
 *
 * Based on main.c by Tetsuya Suzuki 
 * and emuz80_z80ram.c by Satoshi Okue
 * PIC18F47Q43/PIC18F47Q83/PIC18F47Q84 ROM image uploader
 * and UART emulation firmware.
 * This single source file contains all code.
 *
 * Base source code of this firmware is maked by
 * @hanyazou (https://twitter.com/hanyazou) *
 *
 *  Target: MEZ88_47Q_RAM 512KB Rev1.0
 *  Written by Akihito Honda (Aki.h @akih_san)
 *  https://twitter.com/akih_san
 *  https://github.com/akih-san
 *
 *  Date. 2024.4.20
 */

#ifndef __SUPERMEZ80_H__
#define __SUPERMEZ80_H__

#include "../src/picconfig.h"
#include <xc.h>
#include <stdint.h>
#include "../fatfs/ff.h"

//
// Configlations
//

#define CPM		1
#define MSDOS	2
#define FUZIX	3

#define P64 15.625

#define ENABLE_DISK_DEBUG

#define NUM_FILES        4
#define NUM_DRIVES		NUM_FILES

#define TMP_BUF_SIZE     256
#define U3B_SIZE 128

#define MEM_CHECK_UNIT	TMP_BUF_SIZE * 16	// 4 KB
#define MAX_MEM_SIZE	0x00080000			// 512KB
#define bioreq_ubuffadr	0x100				// monitor request IO header address
#define bioreq_cbuffadr	0x106				// function request IO header address
//
// Constant value definitions
//

#define CTL_Q 0x11
#define CTL_P 0x10

#define UNIMON_OFF		0x0000			// MONITOR
#define BASIC_OFF		0x0000
#define CPM68K_OFF		0x0000
#define CBIOS_OFF		0x6200
#define MON_OFF			0x0000
#define FUZIX_OFF		0x2000

// from unimon
#define CONIN_REQ	0x01
#define CONOUT_REQ	0x02
#define CONST_REQ	0x03
#define STROUT_REQ	0x04
#define REQ_DREAD	0x05
#define REQ_DWRITE	0x06
#define READ_TIME	0x07
#define SET_TIME	0x08
#define CONIN_REQ1	0x09
#define CONOUT_REQ1	0x0A
#define STRIN_REQ	0x0B
#define CLR_IRQ		0x0C

typedef struct {
	uint8_t  UREQ_COM;		// unimon CONIN/CONOUT request command
	uint8_t  UNI_CHR;		// charcter (CONIN/CONOUT) or number of strings
	uint32_t STR_adr;		// string address
	uint8_t  CREQ_COM;		// unimon CONIN/CONOUT request command
	uint8_t  CBI_CHR;		// charcter (CONIN/CONOUT) or number of strings
	uint8_t	 disk_drive;
	uint8_t	 dummy;			/* alignment word boundary */
	uint16_t disk_track;
	uint16_t disk_sector;
	uint32_t data_dma;
} crq_hdr;

typedef struct {
	uint8_t  UREQ_COM;		// unimon CONIN/CONOUT request command
	uint8_t  UNI_CHR;		// charcter (CONIN/CONOUT) or number of strings
	uint32_t STR_adr;		// string address
	uint8_t  CREQ_COM;		// unimon CONIN/CONOUT request command
	uint8_t  CBI_CHR;		// charcter (CONIN/CONOUT) or number of strings
	uint8_t	 disk_drive;
	uint8_t	 blocks;		// read/write blocks 1-255, 0=256
	uint32_t lba;
	uint32_t data_dma;
} crq_hdr_fx;

//
// Type definitions
//

// Address Bus
union address_bus_u {
    uint32_t w;             // 32 bits Address
    struct {
        uint8_t ll;        // Address L low
        uint8_t lh;        // Address L high
        uint8_t hl;        // Address H low
        uint8_t hh;        // Address H high
    };
};

union io_address {
	uint16_t adr;
	struct {
		uint8_t l8;
		uint8_t h8;
	};
};

typedef struct {
    uint8_t disk;
    uint8_t disk_read;
    uint8_t disk_write;
    uint8_t disk_verbose;
    uint16_t disk_mask;
} debug_t;

typedef struct {
    uint8_t *addr;
    uint16_t offs;
    unsigned int len;
} mem_region_t;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  media;			// MEDIA DESCRIPTOR
	uint16_t trans_off;		// TRANSFER OFFSET
	uint16_t trans_seg;		// TRANSFER SEG
	uint16_t count;			// COUNT OF BLOCKS OR CHARACTERS
	uint16_t start;			// FIRST BLOCK TO TRANSFER
} iodat;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  bpb1;			// number of support drives.
	uint16_t bpb2_off;		// DWORD transfer address.
	uint16_t bpb2_seg;
	uint16_t bpb3_off;		// DWORD pointer to BPB
	uint16_t bpb3_seg;
	uint8_t  bdev_no;		// block device No.
} CMDP;

typedef struct {
	uint8_t  UREQ_COM;		// unimon CONIN/CONOUT request command
	uint8_t  UNI_CHR;		// charcter (CONIN/CONOUT) or number of strings
	uint16_t STR_off;		// unimon string offset
	uint16_t STR_SEG;		// unimon string segment
	uint8_t  DREQ_COM;		// device request command
	uint8_t  DEV_RES;		// reserve
	uint16_t PTRSAV_off;	// request header offset
	uint16_t PTRSAV_SEG;	// request header segment
} PTRSAV;


typedef struct {
	uint8_t  jmp_ner[3];	// Jmp Near xxxx  for boot.
	uint8_t  mane_var[8];	// Name / Version of OS.
} DPB_HEAD;

typedef struct {
	DPB_HEAD reserve;
//-------  Start of Drive Parameter Block.
	uint16_t sec_size;		// Sector size in bytes.                  (dpb)
	uint8_t  alloc;			// Number of sectors per alloc. block.    (dpb)
	uint16_t res_sec;		// Reserved sectors.                      (dpb)
	uint8_t  fats;			// Number of FAT's.                       (dpb)
	uint16_t max_dir;		// Number of root directory entries.      (dpb)
	uint16_t sectors;		// Number of sectors per diskette.        (dpb)
	uint8_t  media_id;		// Media byte ID.                         (dpb)
	uint16_t fat_sec;		// Number of FAT Sectors.                 (dpb)
//-------  End of Drive Parameter Block.
	uint16_t sec_trk;		// Number of Sectors per track.
} DPB;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  medias1;		//Media byte.
	uint8_t  medias2;		//Media status byte flag.
} MEDIAS;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  media;			// MEDIA DESCRIPTOR
	uint16_t bpb2_off;		// DWORD transfer address.
	uint16_t bpb2_seg;
	uint16_t bpb3_off;		// DWORD pointer to BPB
	uint16_t bpb3_seg;
} BPB;

typedef struct {
	uint16_t TIM_DAYS;		//Number of days since 1-01-1980.
	uint8_t  TIM_MINS;		//Minutes.
	uint8_t  TIM_HRS;		//Hours.
	uint8_t  TIM_HSEC;		//Hundreths of a second.
	uint8_t  TIM_SECS;		//Seconds.
} TPB;

//#define TIM20240101	16071	// 16071days from 1980.1.1
#define TIM20250601	16588	// 16588days from 1980.1.1

//I2C
//General Call Address
#define GeneralCallAddr	0
#define module_reset	0x06
#define module_flash	0x0e

//DS1307 Slave address << 1 + R/~W
#define DS1307			0b11010000	// support RTC module client address

//FT200XD Slave address << 1 + R/~W
#define FT200XD			0b01000100	// support USB I2C module client address

#define BUS_NOT_FREE	1
#define NACK_DETECT		2
//
// Global variables and function prototypes
//

extern uint8_t tmp_buf[2][TMP_BUF_SIZE];
extern debug_t debug;

extern int open_fxdsk(FILINFO *);
extern int open_cpmdsk(FILINFO *);
extern void clr_int(uint8_t);
extern void io_init(void);
extern void clr_uart_rx_buf(void);
extern unsigned int rx_cnt;
extern void cpmio_init(void);

extern uint16_t chk_i2cdev(void);
extern void setup_clk_aux(void);
extern void picif_init(void);
extern void mem_init(void);
extern uint16_t chk_leap(uint16_t);

extern void write_sram(uint32_t addr, uint8_t *buf, unsigned int len);
extern void read_sram(uint32_t addr, uint8_t *buf, unsigned int len);
extern void board_event_loop(void);
extern void (*bus_master_operation)(void);
extern void cpm_bus_master_operation(void);
extern void fx_bus_master_operation(void);

// output char, if TXIF= empty. if not empty, return status BUSY(not ZERO)
extern uint8_t out_chr(char);

extern unsigned int get_str(char *buf, uint8_t cnt);

//
// debug macros
//
#ifdef ENABLE_DISK_DEBUG
#define DEBUG_DISK (debug.disk || debug.disk_read || debug.disk_write || debug.disk_verbose)
#define DEBUG_DISK_READ (debug.disk_read)
#define DEBUG_DISK_WRITE (debug.disk_write)
#define DEBUG_DISK_VERBOSE (debug.disk_verbose)
#else
#define DEBUG_DISK 0
#define DEBUG_READ 0
#define DEBUG_WRITE 0
#define DEBUG_DISK_VERBOSE 0
#endif

// Time device
extern TPB tim_pb;					// TIME device parameter block
extern uint8_t rtc[7];				//// DS1307 low data
extern uint16_t time_dev;			// 0:Timer0, 1:DS1307

extern uint8_t nmi_sig;				// NMI request flag
extern uint8_t irq_flg;				// irq flag

extern const uint16_t mtod[12];
extern void timer0_init(void);
extern void sys_init(void);
extern void start_M68K(void);
extern void setup_sd(void);
extern void setup_I2C(void);
extern void I2C1_Init(void);
extern uint16_t chk_i2cdev(void);
extern int cnv_rtc_tim(void);
extern void datcnv_tim_rtc(void);
extern void cvt_bcd_bin(void);

extern uint8_t I2C_ByteWrite(uint8_t client_addr, uint8_t data);
extern uint8_t read_I2C(uint8_t client_addr, uint8_t wordAdd, uint8_t count, uint8_t *buff );
extern uint8_t write_I2C(uint8_t client_addr, uint8_t wordAdd, uint8_t count, uint8_t *buff);

extern uint8_t I2C_ByteRead(uint8_t client_addr, uint8_t *buf);
extern uint8_t I2C_ByteRead_WSA(uint8_t client_addr, uint8_t address, uint8_t *buf);

extern uint8_t I2C_Check_BusFree(void);

extern uint16_t days_from_1980(uint16_t year, uint16_t month, uint16_t day);
extern uint8_t cnv_bcd(uint8_t bval);
extern uint8_t cnv_byte(uint8_t bval);
extern void cnv_ymd(uint16_t n_date, uint16_t *year, uint16_t *month, uint16_t *date );

#endif  // __SUPERMEZ80_H__
