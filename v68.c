#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#ifdef __MACH__
#include <mach/clock_types.h>
#include <mach/clock.h>
#include <mach/mach.h>
#endif
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


static int logging = 0;

#define NBANK 		4
#define BANK_BASE 	0x00200000
#define BANK_SIZE	(1024 * 1024)

/* Low RAM memory - 512K for now */
static uint8_t ram[512 * 1024];
/* Banked memory - N8VEM 68K style for development */
static uint8_t bankram[NBANK][BANK_SIZE];
static uint8_t curbank;

static struct ide_controller *ide;
static uint8_t timer_v = 1;
static uint32_t low;
uint8_t fc;

#define IOBASE		0x00F00000
#define IOIDE_START	0x00F01000
#define IOIDE_END	0x00F0100F

#define MMU_MASK	0x00F02000
#define MMU_ROOT	0x00F02010
#define MMU_FAULT	0x00F02020

#define SERIO_IN	0x00F03000
#define SERIO_OUT	0x00F03000
#define SERIO_STATUS	0x00F03010

#define TIMER_IO	0x00F04000

#define BANK_IO		0x00F05000

#define IRQ_MMU 	7
#define IRQ_TIMER	6

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

static void char_out(unsigned int c)
{
	putchar(c);
	fflush(stdout);
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

int cpu_irq_ack(int level)
{
	if (!(irq_pending & (1 << level)))
		return M68K_INT_ACK_SPURIOUS;
	if (level == IRQ_MMU)
		return M68K_INT_ACK_AUTOVECTOR;
	if (level == IRQ_TIMER)
		return M68K_INT_ACK_AUTOVECTOR;
	return M68K_INT_ACK_SPURIOUS;
}

/* Minimal MMU */

static uint32_t mmu_mask, mmu_root, mmu_fault;
static int mmu_type;

static void mmu_trap(uint32_t addr, int is_wr)
{
	static const char *names[] = {"Read", "Write" };

	fprintf(stderr, "%s fault 0x%08X at 0x%08X\n",
		names[is_wr], addr, REG_PC);
	/* bus_error */
	if (mmu_fault == 0) {
		mmu_fault = 1;
		irq_pending |= 1 << IRQ_MMU;
		irq_compute();
	}
}

static int bank_range(unsigned int addr, unsigned int len)
{
	/* Don't get confused by wraps at the top of memory */
	if (addr + len < addr)
		return 0;
	if (addr >= BANK_BASE && addr + len <= BANK_BASE + BANK_SIZE)
		return 1;
	return 0;
}

static unsigned int translate(unsigned int addr, unsigned int is_wr)
{
	unsigned int pa = (addr & mmu_mask) ^ mmu_root;
	switch (mmu_type) {
		/* Simple limits */
		case 0:
		default:
			if (low && (addr < low || 
				(addr > sizeof(ram) && !bank_range(addr,1)))) {
				if (is_wr)
					mmu_trap(addr, is_wr);
				else
					fprintf(stderr, "Stray read 0x%08X at 0x%08X\n",
						addr, REG_PC);
				return 0xFFFFFFFF;
			}
			return addr;
		/* Proposed Atari MMU */
		case 1:
			if ((addr & ~mmu_mask) && (addr & ~mmu_mask) != ~mmu_mask) {
				mmu_trap(addr, is_wr);
				return 0xFFFFFFFF;
			}
			return pa;
		/* Base / Limit */
		case 2:
			addr += mmu_root;
			if (addr >= mmu_mask) {
				mmu_trap(addr, is_wr);
				return 0xFFFFFFFF;
			}
			return addr;
	}
}

/* CPU bus decodes */

static unsigned int do_io_readb(unsigned int address)
{
	unsigned int r;

	if (address >= IOIDE_START && address <= IOIDE_END)
		return ide_read8(ide, (address - IOIDE_START) / 2);

	if (address >= TIMER_IO + 0x10 && address <= TIMER_IO + 0x13) {
		time_t t;
		time(&t);
		t >>= ((TIMER_IO + 0x17 - address) * 8);
		return t;
	}
	switch(address) {
	case MMU_MASK:
		return mmu_mask >> 20;
	case MMU_MASK + 1:
		return mmu_mask >> 12;
	case MMU_ROOT:
		return mmu_root >> 20;
	case MMU_ROOT+1:
		return mmu_root >> 12;
	case MMU_FAULT:
		r = mmu_fault;
		mmu_fault = 0;
		irq_pending &= ~ (1 << IRQ_MMU);
		irq_compute();
		return r;
	case SERIO_IN:
		return next_char();
	case SERIO_STATUS:
		return check_chario();
	case TIMER_IO:
		irq_pending &= ~ (1 << IRQ_TIMER);
		irq_compute();
		return 0x00;
	case TIMER_IO+0x20:
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		return tv.tv_usec/100000;
	}
	case BANK_IO:
		return curbank;
	}
	/* bus error ? */
	return 0xFF;
}

static void do_io_writeb(unsigned int address, unsigned int value)
{
//	printf("IOwriteB %x = %d\n", address, value);
	if (address >= IOIDE_START && address <= IOIDE_END)
		ide_write8(ide, (address - IOIDE_START) / 2, value);
	else switch(address) {
	case MMU_MASK:
		mmu_mask &= (0xFF << 20);
		mmu_mask |= (value << 20);
		break;
	case MMU_MASK + 1:
		mmu_mask &= (0xFF << 12);
		mmu_mask |= (value << 12);
		break;
	case MMU_ROOT:
		mmu_root &= (0xFF << 20);
		mmu_root |= (value << 20);
		break;
	case MMU_ROOT+1:
		mmu_root &= (0xFF << 12);
		mmu_root |= (value << 12);
		break;
	case MMU_FAULT:
		break;
	case SERIO_OUT:
		char_out(value);
		break;
	case TIMER_IO:
		timer_v = value;
		break;
	case BANK_IO:
		curbank = value & (NBANK - 1);
		printf("[Bank %d]\n", curbank);
		break;
	default: ;
		/* bus error ? */
	}
}

/* Read data from RAM, ROM, or a device */
unsigned int cpu_read_byte(unsigned int address)
{
	address &= 0xFFFFFF;

	if (!(fc & 4))
		address = translate(address, 0);

	if (address < sizeof(ram))
		return ram[address];
	if (bank_range(address, 1))
		return bankram[curbank][address - BANK_BASE];
	if (address >= IOBASE)
		return do_io_readb(address);
	/* Bus error ?? */
	printf("%x: BUS %d\n", REG_PC, address);
	return 0xFF;
}

unsigned int cpu_read_word(unsigned int address)
{
	unsigned int vaddress;

	address &= 0xFFFFFF;

	vaddress = address;
	/* We can do this as one because we know the address is even
	   aligned so the two bytes translate adjacent */
	if (!(fc & 4))
		vaddress = translate(vaddress, 0);

	if (vaddress < sizeof(ram) - 1) {
		if (logging && address >= 0x188FC) {
			char buf[256];
			fprintf(stderr, "RW %x/%x ", address, address - 0x188FC);
			fprintf(stderr,"=%x [%x]", READ_WORD(ram, vaddress),
				m68k_get_reg(NULL, M68K_REG_A7));
			logging = 0;
			m68k_disassemble(buf, address, M68K_CPU_TYPE_68000);
			logging = 1;
			fprintf(stderr, "%s\n", buf);
		}
		return READ_WORD(ram, vaddress);
	}
	if (bank_range(vaddress, 2))
		return READ_WORD(bankram[curbank], vaddress - BANK_BASE);
	if (address >= IOIDE_START && address <= IOIDE_END)
		return ide_read16(ide, (address - IOIDE_START) / 2);
	/* Corner cases */
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
	if (!(fc & 4))
		address = translate(address, 1);
	if (address < sizeof(ram))
		ram[address] = value;
	else if (bank_range(address, 1))
		bankram[curbank][address - BANK_BASE] = value;
	else if (address >= IOBASE)
		do_io_writeb(address, (value & 0xFF));
	else
		printf("%x: BUS %d\n", REG_PC, address);
}

void cpu_write_word(unsigned int address, unsigned int value)
{
	unsigned int vaddress;

	address &= 0xFFFFFF;
	vaddress = address;

	/* We can do this as one because we know the address is even
	   aligned so the two bytes translate adjacent */
	if (!(fc & 4))
		vaddress = translate(vaddress, 1);

	if (vaddress < sizeof(ram) - 1) {
		WRITE_WORD(ram, vaddress, value);
	} else if (bank_range(vaddress, 2)) {
		WRITE_WORD(bankram[curbank], address - BANK_BASE, value);
	} else if (address >= IOIDE_START && address <= IOIDE_END)
		ide_write16(ide, (address - IOIDE_START) / 2, value);
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

static struct timespec last_time;

static void device_init(void)
{
#ifdef __MACH__
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	last_time.tv_sec = mts.tv_sec;
	last_time.tv_nsec = mts.tv_nsec;
#else 
	clock_gettime(CLOCK_MONOTONIC, &last_time);
#endif
	irq_pending = 0;
	if (mmu_type == 1)
		mmu_mask = 0xFFFFFFFF;
	else
		mmu_mask = 0;
	mmu_root = 0;
	mmu_fault = 0;
	ide_reset_begin(ide);
}

static void device_update(void)
{
	struct timespec tv, tmp;
	unsigned long n;
#ifdef __MACH__
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	tv.tv_sec = mts.tv_sec;
	tv.tv_nsec = mts.tv_nsec;
#else 
	clock_gettime(CLOCK_MONOTONIC, &tv);
#endif
	tmp.tv_sec = tv.tv_sec - last_time.tv_sec;
	tmp.tv_nsec = tv.tv_nsec - last_time.tv_nsec;
	/* Difference in hundredths */
	n = tmp.tv_sec * 100 + tmp.tv_nsec/10000000;
	if (timer_v && n >= timer_v) {
		last_time = tv;
		irq_pending |= 1 << IRQ_TIMER;
		irq_compute();
	}
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
	t.tv_nsec = 10000000;
	if (nanosleep(&t, NULL))
		perror("nanosleep");
}

void cpu_pulse_reset(void)
{
	device_init();
}

int main(int argc, char* argv[])
{
	int fd;

	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, cleanup);
		signal(SIGQUIT, cleanup);
		term.c_lflag &= ~ICANON;
		term.c_cc[VMIN] = 1;
		term.c_cc[VTIME] = 0;
		term.c_cc[VINTR] = 0;
		term.c_cc[VEOF] = 0;
		term.c_lflag &= ~(ECHO|ECHOE|ECHOK);
		tcsetattr(0, 0, &term);
	}

	if (argc >= 2 && strcmp(argv[1], "-p") == 0) {
		argv++;
		low = 0x10000;
		argc--;
	} else if (argc >= 2 && strcmp(argv[1], "-l") == 0) {
		argv++;
		mmu_type = 2;
		argc--;
	} else if (argc >= 2 && strcmp(argv[1], "-a") == 0) {
		argv++;
		mmu_type = 1;
		argc--;
	}

	if(argc > 2) {
		printf("Usage: v68 [-[p|l|s]] <program file>\n");
		exit(-1);
	}

	if (argc == 2) {
		if((fd = open(argv[1], O_RDONLY)) == -1) {
			perror(argv[1]);
			exit(1);
		}

		if(read(fd, ram, sizeof(ram)) < 0) {
			perror("read");
			exit(1);
		}
		close(fd);
	} else {
		/* Boot data into memory */
		fd = open("boot.dat", O_RDONLY);
		if (fd == -1) {
			perror("boot.dat");
			exit(1);
		}
		if (read(fd, ram, 0x1000) != 0x1000) {
			perror("boot.dat");
			exit(1);
		}
		close(fd);
	}
	
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_pulse_reset();

	fd = open("disk.img", O_RDWR);
	if (fd == -1) {
		perror("disk.img");
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

	while(1) {
		/* A 10MHz 68000 should do 100,000 cycles per 1/100th of a
		   second. We do a blind 0.01 second sleep so we are actually
		   emulating a bit under 10Mhz - which will do fine for
		   testing this stuff */
		m68k_execute(100000);
		device_update();
		take_a_nap();
	}
}

