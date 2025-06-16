/* from unimon */
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

/* shared memory address */
#define SMA_ADDR	0x100;
#define INVOKE_PIC	0x80000
#define CIN_BUFFER	0x200;		/* console buffer */
#define CON_ST		0x0280
#define CON_IN		0x0284
#define CON_OUT		0x0288

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
	uint8_t	crd_cn;			/* console input buffer count */
	uint8_t	crd_rp;			/* console input buffer read pointer */
	uint8_t	crd_wp;			/* console input buffer write pointer */
} sma;
