#include <kernel.h>
#include <kdata.h>
#include <printf.h>
#include <stdbool.h>
#include <devtty.h>
#include <tty.h>
#include <mez68k.h>

#undef  DEBUG			/* Undefine to delete debug code sequences */
static unsigned char tbuf1[TTYSIZ];

struct s_queue ttyinq[NUM_DEV_TTY + 1] = {	/* ttyinq[0] is never used */
	{ NULL, NULL, NULL, 0, 0, 0 },
	{ tbuf1, tbuf1, tbuf1, TTYSIZ, 0, TTYSIZ / 2 },
};

tcflag_t termios_mask[NUM_DEV_TTY + 1] = {
	0,
	CSIZE | CBAUD | CSTOPB | PARENB | PARODD | _CSYS,
};

//static volatile struct acia *const acia = (struct acia *) 0x30001;
//static volatile struct via6522 *const via = (struct via6522 *) 0x30000;
//	key_st = (uint8_t *)CON_ST;
//	key_in = (char *)CON_IN;
//	prt_out = (void *)CON_OUT;

typedef uint8_t (*key_st)(void);
typedef char (*key_in)(void);
typedef void (*prt_out)(uint_fast8_t);

static key_st keyst = (key_st)CON_ST;
static key_in keyin = (key_in)CON_IN;
static prt_out prtout = (prt_out)CON_OUT;

/* Output for the system console (kprintf etc) */
void kputchar(uint_fast8_t c)
{
	if (c == '\n') kputchar('\r');
	tty_putc(TTYDEV & 0xff, c);
}

ttyready_t tty_writeready(uint_fast8_t minor)
{
	return TTY_READY_NOW;
}

void tty_putc(uint_fast8_t minor, uint_fast8_t c)
{
	(*prtout)(c);
}

void tty_sleeping(uint_fast8_t minor)
{
	used(minor);
}

/*
 *	This function is called whenever the terminal interface is opened
 *	or the settings changed. It is responsible for making the requested
 *	changes to the port if possible. Strictly speaking it should write
 *	back anything that cannot be implemented to the state it selected.
 */
void tty_setup(uint_fast8_t minor, uint_fast8_t flags)
{
}

int tty_carrier(uint_fast8_t minor)
{
	return 1;
}

void tty_data_consumed(uint_fast8_t minor)
{
}

void tty_interrupt(void)
{
	if ((*keyst)()) tty_inproc(1, (*keyin)());
}

void plt_interrupt(void)
{
	tty_interrupt();
	timer_interrupt();
}
