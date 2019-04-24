#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include "v68.h"
#include "m68k.h"
#include "m68kcpu.h"
#include "ide.h"

/* 16MB RAM except for the top 32K which is I/O */

static uint8_t ram[(16 << 20) - 32768];
/* IDE controller */
static struct ide_controller *ide;

static int rc2014;

uint8_t fc;

/* Read/write macros */
#define READ_BYTE(BASE, ADDR) (BASE)[ADDR]
#define READ_WORD(BASE, ADDR) (((BASE)[ADDR]<<8) | \
			(BASE)[(ADDR)+1])
#define READ_LONG(BASE, ADDR) (((BASE)[ADDR]<<24) | \
			((BASE)[(ADDR)+1]<<16) | \
			((BASE)[(ADDR)+2]<<8) | \
			(BASE)[(ADDR)+3])

#define WRITE_BYTE(BASE, ADDR, VAL) (BASE)[ADDR] = (VAL)&0xff
#define WRITE_WORD(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>8) & 0xff; \
			(BASE)[(ADDR)+1] = (VAL)&0xff
#define WRITE_LONG(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>24) & 0xff; \
			(BASE)[(ADDR)+1] = ((VAL)>>16)&0xff; \
			(BASE)[(ADDR)+2] = ((VAL)>>8)&0xff; \
			(BASE)[(ADDR)+3] = (VAL)&0xff


/* Simple polled serial port */

static unsigned int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, NULL, NULL, &tv) == -1) {
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}

static unsigned int next_char(void)
{
	char c;
	if (read(0, &c, 1) != 1) {
		printf("(tty read without ready byte)\n");
		return 0xFF;
	}
	return c;
}

/* Interrupt controller */

static unsigned int irq_pending;

static void irq_compute(void)
{
	int i;
	if (irq_pending) {
		for (i = 7; i >= 0; i--) {
			if (irq_pending & (1 << i)) {
				m68k_set_irq(i);
				return;
			}
		}
	} else
		m68k_set_irq(0);
}

/* Until we plug any RC2014 emulation into this */

static uint8_t rc2014_inb(uint8_t address)
{
	return 0xFF;
}

static void rc2014_outb(uint8_t address, uint8_t data)
{
}

/* 68681 DUART */

struct duart_port {
	uint8_t mr1;
	uint8_t mr2;
	uint8_t sr;
	uint8_t csr;
	uint8_t rx;
	uint8_t mrp;
	uint8_t txdis;
	uint8_t rxdis;
};

struct duart {
	struct duart_port port[2];
	uint8_t ipcr;
	uint8_t isr;
	int32_t ct;		/* We overflow this temporarily */
	uint16_t ctr;
	uint8_t ctstop;
	uint8_t imr;
	uint8_t ivr;
	uint8_t opcr;
	uint8_t acr;
	uint8_t irq;
};

struct duart duart;

/* DUART is on IPL1 */

static void duart_irq_calc(struct duart *d)
{
	d->irq = d->isr & d->imr;
	if (d->irq)
		irq_pending |= 1 << 2;
	else
		irq_pending &= ~(1 << 2);
	irq_compute();
}

static void duart_irq_raise(struct duart *d, uint8_t m)
{
	if (!(d->isr & m)) {
		d->isr |= m;
		duart_irq_calc(d);
	}
}

static void duart_irq_lower(struct duart *d, uint8_t m)
{
	if (d->isr & m) {
		d->isr &= ~m;
		duart_irq_calc(d);
	}
}

static uint8_t duart_input(struct duart *d, int port)
{
	if (d->port[port].sr & 0x01) {
		d->port[port].sr ^= 0x01;
		duart_irq_lower(d, 2 << (4 * port));
	}
	return d->port[port].rx;
}

static void duart_output(struct duart *d, int port, int value)
{
	duart_irq_lower(d, 1 << (4 * port));
	if (d->port[port].txdis)
		return;
	if (d->port[port].sr & 0x04) {
		if (port == 0) {
			uint8_t v = value & 0xFF;
			write(1, &v, 1);
		}
		d->port[port].sr &= 0xF3;
		duart_irq_calc(d);
	}
}

static void duart_command(struct duart *d, int port, int value)
{
	switch ((value & 0xE0) >> 4) {
	case 0:
		break;
	case 1:
		d->port[port].mrp = 0;
		break;
	case 2:
		/* Reset RX A */
		d->port[port].rxdis = 1;
		break;
	case 3:
		break;
	case 4:
		d->port[port].sr &= 0x0F;
		break;
	case 5:
		duart_irq_lower(d, 4 << (4 * port));
		duart_irq_calc(d);
		break;
	case 6:
		break;		/* Literally start break */
	case 7:
		break;		/* Stop break */
	}
	if (value & 1)
		d->port[port].rxdis = 0;
	if (value & 2)
		d->port[port].rxdis = 1;
	if (value & 4)
		d->port[port].txdis = 0;
	if (value & 8)
		d->port[port].txdis = 1;
}

/* Simulate 1/100th second of action */
static void duart_count(struct duart *d, int n)
{
	/* We are clocked at ??? so divide as needed */
	uint16_t clock = 184;	/* 1843200 so not entirely accurate
				   FIXME: we could track partial clocks */
	if (n == 16)
		clock /= 16;	/* Again needs accuracy sorting */

	/* Counter mode can be stopped */
	if (!(d->acr & 0x40))
		if (d->ctstop)
			return;

	d->ct -= clock;
	while (d->ct < 0) {
		d->ct += d->ctr;
		duart_irq_raise(d, 0x08);
	}
}

static void duart_tick(void)
{
	uint8_t r = check_chario();
	if ((r & 1) && duart.port[0].rxdis == 0) {
		duart.port[0].rx = next_char();
		duart.port[0].sr |= 0x01;
		duart_irq_raise(&duart, 0x02);
	}
	if (r & 2) {
		if (!duart.port[0].txdis && !(duart.port[0].sr & 0x04)) {
			duart.port[0].sr |= 0x0C;
			duart_irq_raise(&duart, 0x01);
		}
		if (!duart.port[1].txdis && !(duart.port[1].sr & 0x04)) {
			duart.port[1].sr |= 0x0C;
			duart_irq_raise(&duart, 0x10);
		}
	}
	switch ((duart.acr & 0x70) >> 4) {
		/* Clock and timer modes */
	case 0:		/* Counting IP2 */
		break;
	case 1:		/* Counting TxCA */
		break;
	case 2:		/* Counting TxCB */
		break;
	case 3:		/* Counting EXT/x1 clock  / 16 */
		duart_count(&duart, 16);
		break;
	case 4:		/* Timer on IP2 */
		break;
	case 5:		/* Timer on IP2/16 */
		break;
	case 6:		/* Timer on X1/CLK */
		duart_count(&duart, 1);
		break;
	case 7:		/* Timer on X1/CLK / 16 */
		duart_count(&duart, 16);
		break;
	}
}

static void duart_reset(struct duart *d)
{
	d->ctr = 0xFFFF;
	d->ct = 0x0000;
	d->acr = 0xFF;
	d->isr = 0;
	d->imr = 0;
	d->port[0].mrp = 0;
	d->port[0].sr = 0x00;
	d->port[1].mrp = 0;
	d->port[1].sr = 0x00;
}

static unsigned int duart_read(unsigned int address)
{
	if (!(address & 1))
		return 0x00;
	switch (address >> 1) {
	case 0x00:		/* MR1A/MR2A */
		if (duart.port[0].mrp)
			return duart.port[0].mr2;
		duart.port[0].mrp = 1;
		return duart.port[0].mr1;
	case 0x01:		/* SRA */
		return duart.port[0].sr;
	case 0x02:		/* BRG test */
	case 0x03:		/* RHRA */
		return duart_input(&duart, 0);
	case 0x04:		/* IPCR */
		return duart.ipcr;
	case 0x05:		/* ISR */
		return duart.isr;
	case 0x06:		/* CTU */
		return duart.ct >> 8;
	case 0x07:		/* CTL */
		return duart.ct & 0xFF;
	case 0x08:		/* MR1B/MR2B */
		if (duart.port[1].mrp)
			return duart.port[1].mr2;
		duart.port[1].mrp = 1;
		return duart.port[1].mr1;
	case 0x09:		/* SRB */
		return duart.port[1].sr;
	case 0x0A:		/* 1x/16x Test */
	case 0x0B:		/* RHRB */
		return duart_input(&duart, 1);
	case 0x0C:		/* IVR */
		return duart.ivr;
	case 0x0D:		/* IP */
		return 0xff;	/* duart.ip; */
	case 0x0E:		/* START */
		duart.ct = duart.ctr;
		duart.ctstop = 0;
		return 0xFF;
	case 0x0F:		/* STOP */
		duart.ctstop = 1;
		duart_irq_lower(&duart, 0x08);
		return 0xFF;
	}
	return 0xFF;
}

static void duart_write(unsigned int address, unsigned int value)
{
	if (!(address & 1))
		return;
	value &= 0xFF;

	switch (address >> 1) {
	case 0x00:
		if (duart.port[0].mrp)
			duart.port[0].mr2 = value;
		else
			duart.port[0].mr1 = value;
		break;
	case 0x01:
		duart.port[0].csr = value;
		break;
	case 0x02:
		duart_command(&duart, 0, value);
		break;
	case 0x03:
		duart_output(&duart, 0, value);
		break;
	case 0x04:
		duart.acr = value;
		duart_irq_calc(&duart);
		break;
	case 0x05:
		duart.imr = value;
		duart_irq_calc(&duart);
		break;
	case 0x06:
		duart.ctr &= 0xFF;
		duart.ctr |= value << 8;
		break;
	case 0x07:
		duart.ctr &= 0xFF00;
		duart.ctr |= value;
		break;
	case 0x08:
		if (duart.port[1].mrp)
			duart.port[1].mr2 = value;
		else
			duart.port[1].mr1 = value;
		break;
	case 0x09:
		duart.port[1].csr = value;
		break;
	case 0x0A:
		duart_command(&duart, 1, value);
		break;
	case 0x0B:
		duart_output(&duart, 1, value);
		break;
	case 0x0C:
		duart.ivr = value;
		break;
	case 0x0D:
		duart.opcr = value;
		break;
	case 0x0E:
		duart.opcr |= value;
		break;
	case 0x0F:
		duart.opcr &= ~value;
		break;
	}
}


int cpu_irq_ack(int level)
{
	if (!(irq_pending & (1 << level)))
		return M68K_INT_ACK_SPURIOUS;
	if (level == 2)
		return duart.ivr;
	return M68K_INT_ACK_SPURIOUS;
}


/* TODO v2 board added an RTC */

static unsigned int do_io_readb(unsigned int address)
{
	if (rc2014 && address >= 0xFF8000 && address <= 0xFF8FFF)
		return rc2014_inb(((address - 0xFF8000) >> 1) & 0xFF);
	/* SPI is not modelled */
	if (address >= 0xFFD000 && address <= 0xFFDFFF)
		return 0xFF;
	/* ATA CF */
	/* FIXME: FFE010-01F sets CS1 */
	if (address >= 0xFFE000 && address <= 0xFFEFFF)
		return ide_read8(ide, (address & 31) >> 1);
	/* DUART */
	return duart_read(address & 31);
}

static void do_io_writeb(unsigned int address, unsigned int value)
{
	if (address == 0xFFFFFF) {
		printf("<%c>", value);
		return;
	}
	if (rc2014 && address >= 0xFF8000 && address <= 0xFF8FFF) {
		rc2014_outb(((address - 0xFF8000) >> 1) & 0xFF, value);
		return;
	}
	/* SPI is not modelled */
	if (address >= 0xFFD000 && address <= 0xFFDFFF)
		return;
	/* ATA CF */
	if (address >= 0xFFE000 && address <= 0xFFEFFF) {
		ide_write8(ide, (address & 31) >> 1, value);
		return;
	}
	/* DUART */
	duart_write(address & 31, value);
}

/* Read data from RAM, ROM, or a device */
unsigned int cpu_read_byte(unsigned int address)
{
	address &= 0xFFFFFF;
	if (rc2014) {
		/* Musashi can't emulate this properly it seems */
		if (address >= 0x800000 && address <= 0xFF8000) {
			fprintf(stderr, "R: bus error at %d\n", address);
			return 0xFF;
		}
		if (address <= 0xFF8000)
			address &= 0x1FFFFF;
	}
	if (address < sizeof(ram))
		return ram[address];
	return do_io_readb(address);
}

unsigned int cpu_read_word(unsigned int address)
{
	address &= 0xFFFFFF;

	if (rc2014) {
		if (address >= 0x800000 && address < 0xFF8000) {
			fprintf(stderr, "R: bus error at %d\n", address);
			return 0xFFFF;
		}
		/* RAM wraps four times */
		if (address <= 0xFF8000)
			address &= 0x1FFFFF;
	}
	if (address < sizeof(ram) - 1)
		return READ_WORD(ram, address);
	else if (address >= 0xFFE000 && address <= 0xFFEFFF)
		return htons(ide_read16(ide, (address & 31) >> 1));
	return (cpu_read_byte(address) << 8) | cpu_read_byte(address + 1);
}

unsigned int cpu_read_word_dasm(unsigned int address)
{
	return cpu_read_word(address);
}

unsigned int cpu_read_long(unsigned int address)
{
	return (cpu_read_word(address) << 16) | cpu_read_word(address + 2);
}

unsigned int cpu_read_long_dasm(unsigned int address)
{
	return cpu_read_long(address);
}

void cpu_write_byte(unsigned int address, unsigned int value)
{
	if (rc2014) {
		if (address >= 0x800000 && address <= 0xFF8000) {
			fprintf(stderr, "W: bus error at %d\n", address);
			return;
		}
		if (address <= 0xFF8000)
			address &= 0x1FFFFF;
	}
	if (address < sizeof(ram))
		ram[address] = value;
	else
		do_io_writeb(address, (value & 0xFF));
}

void cpu_write_word(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	if (rc2014) {
		if (address >= 0x800000 && address <= 0xFF8000) {
			fprintf(stderr, "W: bus error at %d\n", address);
			return;
		}
		if (address <= 0xFF8000)
			address &= 0x1FFFFF;
	}
	if (address < sizeof(ram) - 1) {
		WRITE_WORD(ram, address, value);
	} else if (address >= 0xFFE000 && address <= 0xFFEFFF)
		ide_write16(ide, (address & 31) >> 1, ntohs(value));
	else {
		/* Corner cases */
		cpu_write_byte(address, value >> 8);
		cpu_write_byte(address + 1, value & 0xFF);
	}
}

void cpu_write_long(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	cpu_write_word(address, value >> 16);
	cpu_write_word(address + 2, value & 0xFFFF);
}

void cpu_write_pd(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	cpu_write_word(address + 2, value & 0xFFFF);
	cpu_write_word(address, value >> 16);
}

void cpu_instr_callback(void)
{
}

static void device_init(void)
{
	irq_pending = 0;
	ide_reset_begin(ide);
	duart_reset(&duart);
}

static struct termios saved_term, term;

static void cleanup(int sig)
{
	tcsetattr(0, 0, &saved_term);
	exit(1);
}

static void exit_cleanup(void)
{
	tcsetattr(0, 0, &saved_term);
}


static void take_a_nap(void)
{
	struct timespec t;
	t.tv_sec = 0;
	t.tv_nsec = 100000;
	if (nanosleep(&t, NULL))
		perror("nanosleep");
}

void cpu_pulse_reset(void)
{
	device_init();
}

void usage(void)
{
	fprintf(stderr, "tiny68k [-0][-1][-2][-e][-r].\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int fd;
	int cputype = M68K_CPU_TYPE_68000;
	int fast = 0;
	int opt;

	while((opt = getopt(argc, argv, "012er")) != -1) {
		switch(opt) {
		case '0':
			cputype = M68K_CPU_TYPE_68000;
			break;
		case '1':
			cputype = M68K_CPU_TYPE_68010;
			break;
		case '2':
			cputype = M68K_CPU_TYPE_68020;
			break;
		case 'e':
			cputype = M68K_CPU_TYPE_68EC020;
			break;
		case 'r':
			rc2014 = 1;
			break;
		case 'f':
			fast = 1;
			break;
		default:
			usage();
		}
	}

	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, SIG_IGN);
		signal(SIGQUIT, cleanup);
		signal(SIGTSTP, SIG_IGN);
		term.c_lflag &= ~ICANON;
		term.c_iflag &= ~(ICRNL | IGNCR);
		term.c_cc[VMIN] = 1;
		term.c_cc[VTIME] = 0;
		term.c_cc[VINTR] = 0;
		term.c_cc[VSUSP] = 0;
		term.c_cc[VEOF] = 0;
		term.c_lflag &= ~(ECHO | ECHOE | ECHOK);
		tcsetattr(0, 0, &term);
	}

	if (optind < argc)
		usage();

	memset(ram, 0xA7, sizeof(ram));

	/* Boot data into memory */
	fd = open("tiny68k.rom", O_RDONLY);
	if (fd == -1) {
		perror("tiny68k.rom");
		exit(1);
	}
	if (read(fd, ram, 0x8000) < 0x1000) {
		fprintf(stderr, "tiny68k.rom: too short.\n");
		exit(1);
	}
	close(fd);

	m68k_init();
	m68k_set_cpu_type(cputype);
	m68k_pulse_reset();

	fd = open("tiny68k.img", O_RDWR);
	if (fd == -1) {
		perror("tiny68k.img");
		exit(1);
	}
	ide = ide_allocate("hd0");
	if (ide == NULL)
		exit(1);
	if (ide_attach(ide, 0, fd))
		exit(1);

	/* Init devices */
	device_init();

	m68k_pulse_reset();

	while (1) {
		/* A 10MHz 68000 should do 1000 cycles per 1/10000th of a
		   second. We do a blind 0.01 second sleep so we are actually
		   emulating a bit under 10Mhz - which will do fine for
		   testing this stuff */
		m68k_execute(1000);
		duart_tick();
		if (!fast)
			take_a_nap();
	}
}
