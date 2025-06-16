#include <kernel.h>
#include <version.h>
#include <kdata.h>
#include <tinydisk.h>
#include "printf.h"
#include <devsys.h>
#include <tty.h>
#include <mez68k.h>

sma *pic_sma;
uint8_t *pic;

/* Current device (for simple setups always 0 and ignored) */
static uint8_t tinysd_unit;
static uint8_t volatile dummy;

struct devsw dev_tab[] =	/* The device driver switch table */
{
// minor    open         close        read      write       ioctl
// -----------------------------------------------------------------
	/* 0: /dev/hd         Disc block devices  */
	{ td_open, no_close, td_read, td_write, td_ioctl },
	/* 1: /dev/fd         Hard disc block devices (absent) */
	{ nxio_open, no_close, no_rdwr, no_rdwr, no_ioctl },
	/* 2: /dev/tty        TTY devices */
	{ tty_open, tty_close, tty_read, tty_write, tty_ioctl },
	/* 3: /dev/lpr        Printer devices */
	{ no_open, no_close, no_rdwr, no_rdwr, no_ioctl },
	/* 4: /dev/mem etc    System devices (one offs) */
	{ no_open, no_close, sys_read, sys_write, sys_ioctl },
	/* Pack to 7 with nxio if adding private devices and start at 8 */
};

bool validdev(uint16_t dev)
{
	/* This is a bit uglier than needed but the right hand side is
	   a constant this way */
//debug
//	kprintf("sizeof(dev_tab) = %x sizeof(struct devsw) = %x\n",sizeof(dev_tab), sizeof(struct devsw));
//	kprintf("dev = %x\n",((sizeof(dev_tab) / sizeof(struct devsw)) << 8) - 1);
//debug
	if (dev > ((sizeof(dev_tab) / sizeof(struct devsw)) << 8) - 1)
		return false;
	else
		return true;
}

/* change 16bit endian */
uint16_t word_b2l( uint32_t data )
{
	uint16_t p1, p2;

	p1 = (data & 0x00ff) << 8;
	p2 = (data & 0xff00) >> 8;

	return( p1 | p2 );
}

/* change 32bit endian */
uint32_t long_b2l( uint32_t data )
{
	uint32_t p1, p2, p3, p4, res;

	p1 = ((uint32_t)data & 0x000000ff) << 24;
	p2 = ((uint32_t)data & 0x0000ff00) << 8;
	p3 = ((uint32_t)data & 0x00ff0000) >> 8;
	p4 = ((uint32_t)data & 0xff000000) >> 24;

	res = (p1 | p2 | p3 | p4);
//debug
//kprintf("long_b2l:\ndata(%lx)\n", data);
//kprintf("res = (%lx)\n", res);
//debug
	return res;
}

int wakeup_pic()
{
	uint8_t volatile c;

	dummy = *pic;						/* wake uo PIC */
	do {
		c = pic_sma->CREQ_COM;
	} while( c );
	return( (int)pic_sma->CBI_CHR );	/* return status */
}

int pic_sd_read()
{
//debug
//kprintf("pic_sd_read:\n");
//debug
	pic_sma->CREQ_COM = REQ_DREAD;
	return wakeup_pic();
}

int pic_sd_write()
{
	pic_sma->CREQ_COM = REQ_DWRITE;
	return wakeup_pic();
}
// is_read = false : write
// is_read = true  : read
int pic_sd_xfer(uint_fast8_t dev, bool is_read, uint32_t lba, uint8_t *dptr)
{
	int res;
//debug
//kprintf("pic_xfer:\n");
//kprintf("dev(%x),is_read(%x),lba(%lx),dptr(%lx)\n",dev,is_read,lba,dptr);
//kprintf("pic_sma(%lx)\n",pic_sma);
//debug

	tinysd_unit = dev;

	__hard_di();		/* Disable interrupt */
	pic_sma->disk_drive = 0;
	pic_sma->blocks =1;
	pic_sma->lba = long_b2l(lba);
	pic_sma->data_dma = long_b2l((uint32_t)dptr);

	if ( is_read ) res = pic_sd_read();
	else res = pic_sd_write();

	__hard_ei();		/* Enable interrupt*/
//debug
//kprintf("res(%x)\n",res);
//debug
	if ( !res ) return 1;	// no error
	else return 0;			// error
}

void device_init(void)
{
	// set PIC I/F shared memory pointer
	// unit : only 0
	pic_sma = (sma *)SMA_ADDR;
	pic = (uint8_t *)INVOKE_PIC;	/* Set the PIC wake up address */

	td_register((uint_fast8_t)0, pic_sd_xfer, td_ioctl_none, 1);
}
