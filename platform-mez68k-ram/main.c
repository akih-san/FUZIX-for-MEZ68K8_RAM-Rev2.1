#include <kernel.h>
#include <timer.h>
#include <kdata.h>
#include <printf.h>
#include <devtty.h>
#include <flat_small.h>
#include <mez68k.h>

uint16_t swap_dev = 0xFFFF;

/*
 *	MMU initialize
 */

void map_init(void)
{
}

uaddr_t ramtop;
uint8_t need_resched;

uint8_t plt_param(char *p)
{
	return 0;
}

/* FIXME: for this platform we definitely want a proper discard */
void plt_discard(void)
{
}

void pagemap_init(void)
{
	/* Linker provided end of kernel */
	/* TODO: create a discard area at the end of the image and start
	   there */

	extern uint8_t _end;
	uint32_t e = (uint32_t) & _end;

	/* Allocate the rest of memory to the userspace */
	/* Set up the memory range available */
//	pagemap_setup(e, 0x30000 - e);
	pagemap_setup(0x10000, 0x80000 - e);
	/* The disk scan will have set up the paging space */
	/* We assume paging space and that all RAM pages have a swap
	   backing */
	kprintf("Motorola 680%s%d processor detected.\n", sysinfo.cpu[1] ? "" : "0", sysinfo.cpu[1]);
	enable_icache();
}

/* Udata and kernel stacks */
/* We need an initial kernel stack and udata so the slot for init is
   set up at compile time */
u_block udata_block[PTABSIZE];

/* This will belong in the core 68K code once finalized */

void install_vdso(void)
{
	extern uint8_t vdso[];
	/* Should be uput etc */
	memcpy((void *) udata.u_codebase, &vdso, 0x20);
}

uint8_t plt_udata_set(ptptr p)
{
	p->p_udata = &udata_block[p - ptab].u_d;
	return 0;
}
