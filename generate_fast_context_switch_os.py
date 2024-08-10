#!/usr/bin/python3
# RISC-V fast context switch threads example: by Brent Hartshorn
# install: sudo apt-get install gcc-riscv64-unknown-elf qemu-system-riscv64
import os, sys, subprocess
from random import choice


def gen_trap_s_slower():
	s = ['''
.equ REGSZ, 8
.global trap_entry
trap_entry:
	csrrw tp, mscratch, tp
	la tp, trap_cpu

	sd ra, (1 * REGSZ)(tp)
	sd sp, (2 * REGSZ)(tp)
	## save program counter
	csrr t0, mepc
	sd t0, (32 * REGSZ)(tp)

	la t0, trap_stack_top
	ld sp, 0(t0)
	call trap_handler

	## restore program counter
	ld t0, (32 * REGSZ)(tp)
	csrw mepc, t0

	ld ra, (1 * REGSZ)(tp)
	ld sp, (2 * REGSZ)(tp)

	mret
	''']
	return '\n'.join(s)

def gen_trap_s( ra=True ):
	s = '''
.equ REGSZ, 8
.global trap_entry
trap_entry:
	csrrw sp, mscratch, sp
	la tp, trap_cpu
	#sd ra, (1 * REGSZ)(tp)
	## save program counter
	csrr t0, mepc
	sd t0, (32 * REGSZ)(tp)
	## call trap_handler C function
	la t0, trap_stack_top
	ld sp, 0(t0)
	call trap_handler
	## restore program counter
	ld t0, (32 * REGSZ)(tp)
	csrw mepc, t0
	csrr sp, mscratch
	#ld ra, (1 * REGSZ)(tp)
	mret
	'''
	if ra: s = s.replace('#sd', 'sd').replace('#ld','ld')
	return s


def image2c( path, name=None, colors=16, debug=False ):
	from PIL import Image
	if name is None:
		p,name = os.path.split(path)
		name = name.replace(' ', '_').replace('.', '_').replace('-','_')

	img = Image.open(path).convert('RGB')
	img = img.resize( (320, 200) )
	pimg = Image.new('P', (320,200) )
	pimg.putpalette(vga_pal)
	img = img.quantize(palette=pimg)
	img = img.convert('P', colors=colors)
	if debug:
		img.save('/tmp/%s.gif' % name)

	pix = [str(v) for v in img.getdata()]
	c = [
		'for (int i=0; i<sizeof(%s); i++){' % name,
		'	((volatile u8*)0x50000000)[i] = %s[i];' % name,
		'}',
	]
	o = {
		'data'  : 'const unsigned char %s[%s] = {%s};'  % (name, len(pix), ','.join(pix)),
		'name'  : name,
		'redraw': '\n'.join(c),
		'len'   : len(pix)
	}
	return o


def gen_proc_header(images):
	out = [
		PROC_H,
		'extern i32 active_pid;',
		'extern struct proc proc_list[PROC_NAME_MAXLEN];',
	]
	for o in images:
		out.append(o['data'])
	return out

def gen_procs(images, strings, stack_mb=1):
	assert images
	out = []
	for p, o in enumerate(images):
		out += [
			'u8 __proc_%s_stack[%s];' % (p, int(1024*1024*stack_mb) ),
			'void *__proc_%s_stack_top = &__proc_%s_stack[sizeof(__proc_%s_stack) - 1];' % (p,p,p),
			'void __proc_entry_%s(){' % p,
			#'	uart_print("[PID = %s] proc entry!\\n");' % p,
			'	while (true){',
			#'		uart_print("%s");' % strings[p],
			o['redraw'],
			'	}',
			#'	uart_print("[PID = %s] Hello, Process Shceduler!\\n");' % p,
			'}',
		]
	return out

PROC_H = '''
#define PROC_NAME_MAXLEN 64
#define PROC_TOTAL_COUNT 16

enum proc_state {
  PROC_STATE_NONE = 0,
  PROC_STATE_READY,
  PROC_STATE_RUNNING,
};

struct proc {
  enum proc_state state;
  u32 pid;
  u8 name[PROC_NAME_MAXLEN];
  struct cpu cpu;
  u64 hartid;
};
'''

VGA_S = r'''
	# PCI is at 0x30000000
	# VGA is at 00:01.0, using extended control regs (4096 bytes)
	# TODO: Scan for Vendor ID / Product ID
	la t0, 0x30000000|(1<<15)|(0<<12)
	# Set up frame buffer
	la t1, 0x50000008
	sw t1, 0x10(t0)
	# Set up I/O
	la t2, 0x40000000
	sw t2, 0x18(t0)
	# Enable memory accesses for this device
	lw a0, 0x04(t0)
	ori a0, a0, 0x02
	sw a0, 0x04(t0)
	lw a0, 0x04(t0)
	# Set up video mode somehow
	li t3, 0x60 # Enable LFB, enable 8-bit DAC
	sh t3, 0x508(t2)
	# Set Mode 13h by hand
	la a0, mode_13h_regs
	addi a1, t2, 0x400-0xC0
	la t3, 0xC0
	1:
		# Grab address
		lbu a3, 0(a0)
		beq a3, zero, 2f
		add a2, a1, a3
		# Grab index and data
		lb a4, 1(a0)
		lbu a5, 2(a0)
		# Advance a0
		addi a0, a0, 3
		# If this is for the attribute controller, treat it specially.
		blt a3, t3, 3f
		# If this is an external register, also treat it specially.
		blt a4, zero, 4f
			# Normal case
			sb a4, 0(a2)
			sb a5, 1(a2)
			j 1b
		3:
			# The attribute controller is a special case
			lb zero, 0xDA(a1)
			sb a4, 0(a2)
			sb a5, 0(a2)
			j 1b
		4:
			# External registers are also special but not as special as the attribute controller
			sb a5, 0(a2)
			j 1b
	2:
	# Set up a palette
	li t3, 0
	sb t3, 0x408(t2)
	li t3, 0
	li t4, 256*3
	la a0, initial_palette
	1:
		lb t5, 0(a0)
		sb t5, 0x409(t2)
		addi a0, a0, 1
		addi t3, t3, 1
		bltu t3, t4, 1b

'''

START_VGA_S = f'''
.text
.global _start
_start:
  bne a0, x0, _start # loop if hartid is not 0
  li sp, 0x80200000 # setup stack pointer
  {VGA_S}
  j firmware_main # jump to c entry
'''


TRAP_C = r'''
struct cpu trap_cpu;
u8 trap_stack[1 << 20];
void *trap_stack_top = &trap_stack[sizeof(trap_stack) - 1];
i32 active_pid;
struct proc proc_list[PROC_NAME_MAXLEN] = {};

__attribute__((optimize("no-tree-loop-distribute-patterns")))
void trap_handler() {
	u64 mcause = csrr_mcause();
	if (mcause==MCAUSE_INTR_M_TIMER){
		if (active_pid < 0){
			active_pid = 0;
			trap_cpu = proc_list[0].cpu;
		}
		proc_list[active_pid].cpu = trap_cpu; // save cpu state for the active process
		proc_list[active_pid].state = PROC_STATE_READY; // suspend the active process
		for (int ring_index = 1; ring_index <= PROC_TOTAL_COUNT; ring_index++){
			int real_index = (active_pid + ring_index) % PROC_TOTAL_COUNT;
			struct proc *proc = &proc_list[real_index];
			if (proc->state == PROC_STATE_READY){
				trap_cpu = proc->cpu;
				active_pid = proc->pid;
				break;
			}
		}
		kernel_timeout();
	}
}
'''


ARCH = '''
#define MACHINE_BITS 64
#define BITS_PER_LONG MACHINE_BITS
#define bool _Bool
#define true 1
#define false 0
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long u64;
typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef signed long i64;
typedef u64 size_t;
'''

CPU = 'struct cpu {%s} __attribute__((packed));' % '\n'.join(['u64 x%s;' % i for i in range(32)]+['u64 pc;'])

TIMER = '''
#define MTIME 0x200bff8
#define MTIMECMP_0 0x2004000
static inline u64 mtime() { return readu64(MTIME); }
static inline u64 mtimecmp_0() { return readu64(MTIMECMP_0); }
static inline u64 set_timeout(u64 timeout) { writeu64(MTIMECMP_0, mtime() + timeout); }
static inline kernel_timeout(void) { writeu64(MTIMECMP_0, mtime() + 100); }
'''

ARCH_ASM = '''
#define readu8(addr) (*(const u8 *)(addr))
#define readu16(addr) (*(const u16 *)(addr))
#define readu32(addr) (*(const u32 *)(addr))
#define readu64(addr) (*(const u64 *)(addr))
#define writeu8(addr, val) (*(u8 *)(addr) = (val))
#define writeu16(addr, val) (*(u16 *)(addr) = (val))
#define writeu32(addr, val) (*(u32 *)(addr) = (val))
#define writeu64(addr, val) (*(u64 *)(addr) = (val))

//GOTCHA::BREAKS-ASM-PARSER//static inline void csrw_mtvec(const volatile u64 val) { asm volatile("csrw mtvec, %0" :: "r"(val)); } // note the space
static inline void csrw_mtvec(const volatile u64 val) { asm volatile("csrw mtvec,%0" :: "r"(val)); }
static inline void csrw_mie(const volatile u64 val) { asm volatile("csrw mie,%0" :: "r"(val)); }
static inline void csrs_mstatus(const volatile u64 val) { asm volatile("csrs mstatus,%0" :: "r"(val)); }
static inline u64 csrr_mcause(){
  volatile u64 val;
  asm volatile("csrr %0,mcause" : "=r"(val) :);
  return val;
}
'''

INTERRUPTS = '''
#define MSTAUTS_MIE (0x1L << 3)
#define MIE_MTIE (0x1L << 7)
#define MIE_MEIE (0x1L << 11)
#define MCAUSE_INTR_M_TIMER ((0x1L << (MACHINE_BITS - 1)) | 7)
#define MCAUSE_INTR_M_EXTER ((0x1L << (MACHINE_BITS - 1)) | 11)
#define MCAUSE_INNER_M_ILLEAGEL_INSTRUCTION (0x2L)
'''

UART = '''
#define UART_BASE 0x10000000
#define UART_RBR_OFFSET 0  /* In:  Recieve Buffer Register */
#define UART_THR_OFFSET 0  /* Out: Transmitter Holding Register */
#define UART_DLL_OFFSET 0  /* Out: Divisor Latch Low */
#define UART_IER_OFFSET 1  /* I/O: Interrupt Enable Register */
#define UART_DLM_OFFSET 1  /* Out: Divisor Latch High */
#define UART_FCR_OFFSET 2  /* Out: FIFO Control Register */
#define UART_IIR_OFFSET 2  /* I/O: Interrupt Identification Register */
#define UART_LCR_OFFSET 3  /* Out: Line Control Register */
#define UART_MCR_OFFSET 4  /* Out: Modem Control Register */
#define UART_LSR_OFFSET 5  /* In:  Line Status Register */
#define UART_MSR_OFFSET 6  /* In:  Modem Status Register */
#define UART_SCR_OFFSET 7  /* I/O: Scratch Register */
#define UART_MDR1_OFFSET 8 /* I/O:  Mode Register */
#define PLATFORM_UART_INPUT_FREQ 10000000
#define PLATFORM_UART_BAUDRATE 115200
static u8 *uart_base_addr = (u8 *)UART_BASE;
static void set_reg(u32 offset, u32 val){ writeu8(uart_base_addr + offset, val);}
static u32 get_reg(u32 offset){ return readu8(uart_base_addr + offset);}
static void uart_putc(u8 ch){ set_reg(UART_THR_OFFSET, ch);}
static void uart_print(char *str){ while (*str) uart_putc(*str++);}

static inline void uart_init(){
  u16 bdiv = (PLATFORM_UART_INPUT_FREQ + 8 * PLATFORM_UART_BAUDRATE) / (16 * PLATFORM_UART_BAUDRATE);
  set_reg(UART_IER_OFFSET, 0x00); /* Disable all interrupts */
  set_reg(UART_LCR_OFFSET, 0x80); /* Enable DLAB */
  if (bdiv) {
    set_reg(UART_DLL_OFFSET, bdiv & 0xff); /* Set divisor low byte */
    set_reg(UART_DLM_OFFSET, (bdiv >> 8) & 0xff); /* Set divisor high byte */
  }
  set_reg(UART_LCR_OFFSET, 0x03); /* 8 bits, no parity, one stop bit */
  set_reg(UART_FCR_OFFSET, 0x01); /* Enable FIFO */
  set_reg(UART_MCR_OFFSET, 0x00); /* No modem control DTR RTS */
  get_reg(UART_LSR_OFFSET); /* Clear line status */
  get_reg(UART_RBR_OFFSET); /* Read receive buffer */  
  set_reg(UART_SCR_OFFSET, 0x00); /* Set scratchpad */
}
'''

LIBC = r'''
void *memset(void *s, int c, size_t n){
  unsigned char *p = s;
  while (n--) *p++ = (unsigned char)c;
  return s;
}
void *memcpy(void *dest, const void *src, size_t n){
  unsigned char *d = dest;
  const unsigned char *s = src;
  while (n--) *d++ = *s++;
  return dest;
}
#define VRAM ((volatile u8 *)0x50000000)
void putpixel(int x, int y, char c){
	VRAM[y*320 + x] = c;
}
'''

LINKER_SCRIPT = '''
ENTRY(_start)
MEMORY {} /* default */
. = 0x80000000;
SECTIONS {}
'''

MODE_13H = '''
mode_13h_regs:
	# Miscellaneous Output Register:
	# Just a single port. But bit 0 determines whether we use 3Dx or 3Bx.
	# So we need to set this early.
	.byte 0xC2, 0xFF, 0x63

	# Sequencer:
	# Disable reset here.
	.byte 0xC4, 0x00, 0x00

	# Attributes:
	# - Read 3DA to reset flip-flop
	# - Write 3C0 for address
	# - Write 3C0 for data
	.byte 0xC0, 0x00, 0x00
	.byte 0xC0, 0x01, 0x02
	.byte 0xC0, 0x02, 0x08
	.byte 0xC0, 0x03, 0x0A
	.byte 0xC0, 0x04, 0x20
	.byte 0xC0, 0x05, 0x22
	.byte 0xC0, 0x06, 0x28
	.byte 0xC0, 0x07, 0x2A
	.byte 0xC0, 0x08, 0x15
	.byte 0xC0, 0x09, 0x17
	.byte 0xC0, 0x0A, 0x1D
	.byte 0xC0, 0x0B, 0x1F
	.byte 0xC0, 0x0C, 0x35
	.byte 0xC0, 0x0D, 0x37
	.byte 0xC0, 0x0E, 0x3D
	.byte 0xC0, 0x0F, 0x3F

	.byte 0xC0, 0x30, 0x41
	.byte 0xC0, 0x31, 0x00
	.byte 0xC0, 0x32, 0x0F
	.byte 0xC0, 0x33, 0x00
	.byte 0xC0, 0x34, 0x00

	# Graphics Mode
	.byte 0xCE, 0x00, 0x00
	.byte 0xCE, 0x01, 0x00
	.byte 0xCE, 0x02, 0x00
	.byte 0xCE, 0x03, 0x00
	.byte 0xCE, 0x04, 0x00
	.byte 0xCE, 0x05, 0x40
	.byte 0xCE, 0x06, 0x05
	.byte 0xCE, 0x07, 0x00
	.byte 0xCE, 0x08, 0xFF

	# CRTC
	.byte 0xD4, 0x11, 0x0E # Do this to unprotect the registers

	.byte 0xD4, 0x00, 0x5F
	.byte 0xD4, 0x01, 0x4F
	.byte 0xD4, 0x02, 0x50
	.byte 0xD4, 0x03, 0x82
	.byte 0xD4, 0x04, 0x54
	.byte 0xD4, 0x05, 0x80
	.byte 0xD4, 0x06, 0xBF
	.byte 0xD4, 0x07, 0x1F
	.byte 0xD4, 0x08, 0x00
	.byte 0xD4, 0x09, 0x41
	.byte 0xD4, 0x0A, 0x20
	.byte 0xD4, 0x0B, 0x1F
	.byte 0xD4, 0x0C, 0x00
	.byte 0xD4, 0x0D, 0x00
	.byte 0xD4, 0x0E, 0xFF
	.byte 0xD4, 0x0F, 0xFF
	.byte 0xD4, 0x10, 0x9C
	.byte 0xD4, 0x11, 0x8E # Registers are now reprotected
	.byte 0xD4, 0x12, 0x8F, 0xD4, 0x13, 0x28, 0xD4, 0x14, 0x40, 0xD4, 0x15, 0x96, 0xD4, 0x16, 0xB9, 0xD4, 0x17, 0xA3

'''

vga_pal = [0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF,0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF,0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF,0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF]
def gen_pal():
	asm = [
		'initial_palette:',
		'	.byte %s' % ','.join( str(b) for b in vga_pal)
	]
	return '\n'.join(asm)

FIRMWARE_MAIN = r'''
extern void trap_entry();
void firmware_main(){
  uart_init();
  procs_init();
  uart_print("[firmware_main memset proc_list]\n");
  for (int i = %s; i < PROC_TOTAL_COUNT; i++) {
    memset(&proc_list[i], 0, sizeof(proc_list[i]));
    proc_list[i].state = PROC_STATE_NONE;
  }
  active_pid = -1;
  set_timeout(10000); // setup M-mode trap vector
  csrw_mtvec((u64)trap_entry); // enable M-mode timer interrupt  
  csrw_mie(MIE_MTIE);
  csrs_mstatus(MSTAUTS_MIE); // enable MIE in mstatus
  uart_print("[firmware_main waiting...]\n");
  while(1) {
  	uart_putc('<');
    %s
  	uart_putc('>');
  }
}
'''

def gen_firmware(images, strings):
	out = ['void procs_init(){']
	for p,o in enumerate(images):
		out += [
			'struct proc test_proc_%s = {' % p,
			'  .name = "test_proc_%s",' % p,
			'  .pid = %s,' % (p+0),
			'  .hartid = 1,',
			'  .state = PROC_STATE_READY,',
			'  .cpu = {',
			'      .pc = (u64)__proc_entry_%s,' % p,
			'      .x2 = (u64)__proc_%s_stack_top,' % p,
			'  }};',
			'uart_print("[proc_init] proc_list:%s");' % p,
			f'proc_list[{p}] = test_proc_{p};',
		]
	out.append('}')
	print_meme = ['uart_print("%s");' % m.replace('"', "'") for m in strings]
	out.append(FIRMWARE_MAIN % (len(images), '\n'.join(print_meme) ) )
	return out

REMAP_TRAP = {
	'a0' : 't0',  ## t0 is x5
	'a1' : 't1',
	'a2' : 't2',
	'a3' : 't3',
	'a4' : 't4',
	'a5' : 't5',
	'a6' : 't6',
	'a7' : 'gp',
}

def meme(images, strings):
	assert images
	assert strings
	assert len(images) == len(strings)
	assert len(images) == 2

	a = c2asm(ARCH+ARCH_ASM+ CPU+PROC_H+INTERRUPTS+ TIMER+ TRAP_C, opt=1)
	ta = asm2asm(a, reg_replace=REMAP_TRAP )

	out = [ARCH, ARCH_ASM, UART, CPU, TIMER, LIBC]
	out += gen_proc_header(images) + gen_procs(images,strings) + [ INTERRUPTS ] + gen_firmware(images, strings)
	c = '\n'.join(out)

	func_reg_replace = {
		'__proc_entry_0' : REMAP_A,
		'__proc_entry_1' : REMAP_B,
	}

	a = c2asm(c, opt=0)  ## this must be zero for reg remapping to work
	a = asm2asm(a, func_reg_replace)
	print_asm(a, '__proc_entry_0', '__proc_entry_1')
	print_asm(ta)
	ob = asm2o( a, 'kernel' )
	tob = asm2o( ta, 'kernel_trap' )

	tmpld = '/tmp/linker.ld'
	open(tmpld,'wb').write(LINKER_SCRIPT.encode('utf-8'))
	tmps = '/tmp/asm.s'

	asm = [
		START_VGA_S,
		gen_trap_s(),
		'.section .rodata', 
		MODE_13H, gen_pal()
	]
	open(tmps,'wb').write('\n'.join(asm).encode('utf-8'))

	elf = '/tmp/test.elf'
	cmd = [
		'riscv64-unknown-elf-gcc', '-mcmodel=medany', '-ffunction-sections',
		'-ffreestanding', '-nostdlib', '-nostartfiles', '-nodefaultlibs',
		'-Wl,--no-relax', '-T',tmpld, '-O0', '-g', '-o', elf, ob, tob, tmps,
	]
	print(cmd)
	subprocess.check_call(cmd)
	if '--run' in sys.argv:
		cmd = 'riscv64-unknown-elf-objcopy -O binary -S %s /tmp/firmware.bin' % elf
		print(cmd)
		subprocess.check_call(cmd.split())
		cmd = 'qemu-system-riscv64 -machine virt -smp 2 -m 2G -serial stdio -bios /tmp/firmware.bin -s -device VGA'
		print(cmd)
		subprocess.check_call(cmd.split())
	return elf

def parse_objdump(filename):
	objdump_output = subprocess.check_output(["riscv64-unknown-elf-objdump", "-d", filename]).decode("utf-8")
	sects = {}
	sect  = None
	current_section = None
	for ln in objdump_output.splitlines():
		if ln.startswith("Disassembly of section"):
			current_section = ln.split()[-1]
			sect = {}
			sects[current_section]=sect
			continue

		if current_section:
			if ln.startswith('  ') and ':' in ln:
				ops = None
				if '<' in ln:
					ln, comment = ln.split('<')
				if '#' in ln:
					ln, comment2 = ln.split('#')

				if ln.count(':')==1:
					mem, ln = ln.split(':')

				a = ln.split()
				if len(a)==3:
					mem2, inst, ops = a
				elif len(a)==2: ## some inst have no ops, like ret,mret,nop..
					mem2, inst = a
				else:
					raise RuntimeError(len(a))
				#print(a)
				if ops:
					for b in ops.split(','):
						if b in REGS:
							if b not in sect: sect[b] = {'count':0,'asm':[]}
							sect[b]['count'] += 1
							sect[b]['asm'].append(ln)
						## only checking the first operand
						break


	return sects

REGS = ['x%s' % i for i in range(32)]
REGS += [
	'zero',
	'ra',  ## return addr
	'sp',  ## stack pointer
	'gp',  ## global pointer
	'tp',  ## thread pointer
	't0', 't1', 't2',
	's0', 'fp',  ## s0 is sometimes fp
	's1'   ## saved reg1
] + ['a%s' % i for i in range(8)] + ['s%s' % i for i in range(2,12)] + ['t%s' % i for i in range(3,7)]
print('RISC-V registers:',REGS)

S_COLORS = { 's0': 22, 's1': 28, 's2': 64, 's3': 34, 's4': 70, 's5': 40, 's6': 76, 's7': 46, 's8': 47, 's9': 48}
A_COLORS = {'a0': 19,'a1': 20, 'a2': 21, 'a3': 57, 'a4': 56, 'a5': 92, 'a6': 93, 'a7': 129, 'a8': 165, 'a9': 201}
T_COLORS = {'t0' : 226,'t1' : 190,'t2' : 227,'t3' : 191,'t4' : 228,'t5' : 192,'t6' : 229}
reg_colors = { 'tp' : '31', 'sp' : '31', 'gp' : '31', 'ra' : '31', 'zero' : '31' }

REMAP_A = {
	#'s0' : 'tp', -fomit-frame-pointer removes the use of s0
	'a7' : 's7',
	'a6' : 's6',
	'a5' : 's5',
	'a4' : 's4',
	'a3' : 's3',
	'a2' : 's2',
	'a1' : 's1',
	'a0' : 's0',
}
REMAP_B = {
	#'s0' : 'a0',
}

asm_help = {
	'lw' : 'load word',
	'sw' : 'store word',
	'li' : 'load value',
	'sext.w' : 'convert i32 to i64',
	'mulw' : 'multiply word',
	'subw' : 'subtract word',
	'addw' : 'add word',
}

def parse_asm(ln, debug=False):
	r = {}
	if ln.strip().startswith('.'):
		r['data'] = ln
		return r
	elif ln.strip().startswith('#'):
		r['comment'] = ln.strip()
		return r
	if debug: print(ln)
	a = ln.strip().split()
	ops = None
	if len(a)==1:
		if a[0].endswith(':'):
			label = a[0][:-1]
			r['label'] = label
		else:
			r['inst']  = a[0]
		return r
	elif len(a)==2:
		inst, ops = a
	else:
		raise RuntimeError(ln)
	if not ops:
		return r

	r['inst'] = inst
	r['ops']  = ops
	r['regs'] = []
	vis = []
	for b in ops.split(','):
		index = None
		if '(' in b:
			index = b.split('(')[0]
			b = b.split('(')[-1][:-1]

		if b in REGS:
			if b not in r['regs']:
				r['regs'].append(b)

			if b in reg_colors:
				b = '\033[%sm%s\033[0m' % (reg_colors[b], b)
			elif b.startswith('s'):
				if b in S_COLORS:
					COLOR_S = '48;5;%s' % S_COLORS[b]
				else:
					COLOR_S = '30;43'
				b = '\033[%sm %s \033[0m' % (COLOR_S, b)
			elif b.startswith('a'):
				if b in A_COLORS:
					COLOR_A = '48;5;%s' % A_COLORS[b]
				else:
					COLOR_A = '30;44'
				b = '\033[%sm %s \033[0m' % (COLOR_A, b)
			elif b.startswith('t'):
				#b = '\033[38;5;%sm %s \033[0m' % (T_COLORS[b], b)  #fg color
				b = '\033[38;5;0;48;5;%sm %s \033[0m' % (T_COLORS[b], b)

		if index is not None:
			vis.append('%s[%s]' %(b,index))
		else:
			vis.append(b)

	vis = tuple(vis)
	if inst in ('sret', 'sbreak'):
		r['vis'] = 'system< %s >' % inst
	elif inst in ('call', 'tail'):
		r['vis'] = '%s(...)' % ops
	elif inst == 'ble':
		r['vis'] = 'if %s <= %s: goto %s' % vis
	elif inst.startswith('sext.'):  ## sign extend
		if inst.endswith('.w'):  ## 32bit word to 64bit
			r['vis'] = '%s =(i64*)%s' % vis
	else:
		map = {'add':'+', 'div':'/',  'sll' : '<<', 'slr' : '>>', 
			'l':'<-', 's':'->', 'neg':'-', 'rem':'%', 'mul':'*',
			'mv':'◀═┅┅',  ## atomic copy from reg to reg
			'j':'goto',
		}
		for tag in map:
			if inst.startswith(tag):
				if len(vis)==1:
					x = vis[0]
					r['vis'] = '%s %s' % (map[tag], x)
				elif len(vis)==2:
					x,y = vis
					if inst.startswith('neg'):
						r['vis'] = '%s = %s%s' % (x, map[tag], y)
					else:
						r['vis'] = '%s %s %s' % (x, map[tag], y)
				else:
					x,y,z = vis
					r['vis'] = '%s = %s %s %s' % (x, y, map[tag], z)
				break

	if 'vis' in r:
		r['vis'] += '\t\t\t %s : %s' %(inst, ops)
		if inst in asm_help:
			r['vis'] += '\t\t\t : %s' % asm_help[inst] 

	return r


def print_asm(asm, *labels):
	lab = None
	for ln in asm.splitlines():
		a = parse_asm(ln)
		if 'label' in a:
			lab = a['label']
		if 'data' in a: continue
		if labels:
			if lab in labels:
				if 'vis' in a: print(a['vis'])
				else: print(a)
		else:
			if 'vis' in a: print(a['vis'])
			else: print(a)

def asm2asm(path, func_reg_replace={}, reg_replace={}, debug=False, skip_calls=False):
	data = open(path,'rb').read().decode('utf-8')
	if debug: print_asm(data)
	sects = {}
	sect = {}
	label = None
	out = []
	funcs = {}
	func = None
	for ln in data.splitlines():
		if ln.strip().startswith('.'):
			out.append(ln)
			continue
		if ln.strip().startswith('#'): continue
		a = parse_asm(ln)
		if 'label' in a:
			label = a['label']
			sect = {}
			sects[label] = sect
			func = {'lines':[],'ast':[], 'reps':[]}
			funcs[label] = func
			out.append(ln)
			continue

		if 'inst' in a and a['inst']=='call' and skip_calls:
			continue

		if func:
			func['lines'].append(ln)
			func['ast'].append(a)

		if 'regs' in a:
			for b in a['regs']:
				if b not in sect: sect[b] = {'count':0,'asm':[]}

				sect[b]['count'] += 1
				sect[b]['asm'].append('%s :: %s' % (a['inst'],a['ops']))

				if label in func_reg_replace and b in func_reg_replace[label]:
					c = func_reg_replace[label][b]
					ln = ln.replace(b, c)
					reps = func['reps']
					if c not in reps: reps.append(c)
				elif b in reg_replace:
					ln = ln.replace(b, reg_replace[b])

		out.append(ln)

	for fname in funcs:
		for a in funcs[fname]['ast']:
			if 'regs' in a:
				for b in a['regs']:
					if b in funcs[fname]['reps']:
						print(fname)
						print('\n'.join(funcs[fname]['lines']))
						print(func_reg_replace[fname])
						raise SyntaxError('reg replace error: %s %s' % (a,b))

	if debug:
		for ln in out:
			if not ln.strip().startswith('.'):
				print(ln)

	return '\n'.join(out)

def c2asm( c, opt=0 ):
	tmp = '/tmp/tmp.c'
	open(tmp, 'wb').write(c.encode('utf-8'))
	if not opt: opt = '-O0'
	else: opt = '-O%s' % opt
	asm = '/tmp/tmp.S'
	cmd = [
		'riscv64-unknown-elf-gcc', '-mcmodel=medany', '-fomit-frame-pointer', '-ffunction-sections',
		'-ffreestanding', '-nostdlib', '-nostartfiles', '-nodefaultlibs', '-fno-tree-loop-distribute-patterns', 
		'-fno-optimize-register-move', '-fno-sched-pressure', '-fno-sched-interblock',
		'-ffixed-t0', '-ffixed-t1', '-ffixed-t2', '-ffixed-t3', '-ffixed-t4', '-ffixed-t5', '-ffixed-t6',
		opt, '-g', '-S', '-o', asm, tmp
	]
	print(cmd)
	subprocess.check_call(cmd)
	return asm

def asm2o(s, name='asm2o'):
	asm = '/tmp/asm2o.s'
	open(asm,'wb').write(s.encode('utf-8'))
	o = '/tmp/%s.o' % name
	cmd = [ 'riscv64-unknown-elf-as', '-g', '-o',o, asm]
	print(cmd)
	subprocess.check_call(cmd)
	return o

def print_regs(register_usage):
	for section, registers in register_usage.items():
		print(section)
		for reg in registers:
			if reg in reg_colors:
				print('\033[%sm' % reg_colors[reg], end='')
			elif reg.startswith('a'):
				if reg in A_COLORS:
					clr = '48;5;%s' % A_COLORS[reg]
				else:
					clr = '30;44'
				print('\033[%sm' % clr, end='')
			elif reg.startswith('s'):
				if reg in S_COLORS:
					clr = '48;5;%s' % S_COLORS[reg]
				else:
					clr = '30;43'
				print('\033[%sm' % clr, end='')
			print('	%s : %s' %(reg, registers[reg]['count']), end='')
			print('\033[0m')
			if registers[reg]['count'] <= 10:
				for asm in registers[reg]['asm']:
					print('		: %s' %(asm))

if __name__ == '__main__':
	images = []; strings = []
	for arg in sys.argv:
		if arg.endswith(('.jpg', '.png', '.gif')) and os.path.isfile(arg):
			c = image2c(arg)
			images.append(c)
		elif arg.endswith(('.', '?', '!')):
			strings.append(arg)

	elf = meme(images, strings)
	if '--debug' in sys.argv:
		register_usage = parse_objdump(elf)
		print_regs(register_usage)

