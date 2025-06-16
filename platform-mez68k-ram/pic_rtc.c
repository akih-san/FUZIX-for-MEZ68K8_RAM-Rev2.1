#include <kernel.h>
#include <kdata.h>
#include <printf.h>
#include <rtc.h>
#include <mez68k.h>
#include <pic_rtc.h>

#ifdef CONFIG_RTC_PIC

uint_fast8_t plt_rtc_secs(void)
{
    /* We don't bind to the seconds as we don't need too and reading it
       is slow */
    return 0xFF;
}

/* Full RTC support (for read - no write yet) */
int plt_rtc_read(void)
{
	uint8_t len = sizeof(struct cmos_rtc);
	struct cmos_rtc cmos;
	uint8_t *p = cmos.data.bytes;

	if ((uint8_t)udata.u_count < len) len = (uint8_t)udata.u_count;
	pic_sma->CBI_CHR = len;
	pic_sma->data_dma = long_b2l( (uint32_t)p );	// change endian
	pic_sma->CREQ_COM = READ_TIME;

	if ( wakeup_pic() ) return -1;
	cmos.type = CMOS_RTC_BCD;
	__hard_di();		/* Disable interrupt */
	if (uput(&cmos, udata.u_base, len) == -1) return -1;
	__hard_ei();		/* Enaable interrupt */
	return len;
}

int plt_rtc_write(void)
{
	struct cmos_rtc cmos;

	if (udata.u_count != sizeof(struct cmos_rtc)) {
		udata.u_error = EINVAL;
		return -1;
	}
	__hard_di();		/* Disable interrupt */
	if (uget(udata.u_base, &cmos, sizeof(struct cmos_rtc)) == -1) return -1;
	__hard_ei();		/* Enable interrupt */

	if (cmos.type != CMOS_RTC_BCD) {
		udata.u_error = EINVAL;
		return -1;
	}
	pic_sma->CBI_CHR = sizeof(struct cmos_rtc);
	pic_sma->data_dma = long_b2l( (uint32_t)&cmos );	// change endian
	pic_sma->CREQ_COM = SET_TIME;

	if ( wakeup_pic() ) return -1;

	return 0;
}

#ifdef CONFIG_RTC_EXTENDED

uint_fast8_t rtc_nvread(uint_fast8_t r)
{
    return 0;
}

void rtc_nvwrite(uint_fast8_t r, uint_fast8_t v)
{
}

int plt_rtc_ioctl(uarg_t request, char *data)
{
	return 0;
}

#endif
#endif
