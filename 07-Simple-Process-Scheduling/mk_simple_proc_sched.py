#!/usr/bin/python3
import os, sys, subprocess

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

TRAP_S = r'''
.equ REGSZ, 8

.text
.global _start
_start:
  bne a0, x0, _start # loop if hartid is not 0

  li sp, 0x80200000 # setup stack pointer
  j firmware_main # jump to c entry

.global trap_entry
trap_entry:

# swap x5/mscratch
csrrw x5, mscratch, x5
# use x5 as cpu state base address register
la x5, trap_cpu
# save general purpose registers
# x0 ~ x4
sd x0, (0 * REGSZ)(x5)
sd x1, (1 * REGSZ)(x5)
sd x2, (2 * REGSZ)(x5)
sd x3, (3 * REGSZ)(x5)
sd x4, (4 * REGSZ)(x5)
# save origin x5 by x1, which has been saved
csrr x1, mscratch
sd x1, (5 * REGSZ)(x5)
# x6 ~ x31
sd x6, (6 * REGSZ)(x5)
sd x7, (7 * REGSZ)(x5)
sd x8, (8 * REGSZ)(x5)
sd x9, (9 * REGSZ)(x5)
sd x10, (10 * REGSZ)(x5)
sd x11, (11 * REGSZ)(x5)
sd x12, (12 * REGSZ)(x5)
sd x13, (13 * REGSZ)(x5)
sd x14, (14 * REGSZ)(x5)
sd x15, (15 * REGSZ)(x5)
sd x16, (16 * REGSZ)(x5)
sd x17, (17 * REGSZ)(x5)
sd x18, (18 * REGSZ)(x5)
sd x19, (19 * REGSZ)(x5)
sd x20, (20 * REGSZ)(x5)
sd x21, (21 * REGSZ)(x5)
sd x22, (22 * REGSZ)(x5)
sd x23, (23 * REGSZ)(x5)
sd x24, (24 * REGSZ)(x5)
sd x25, (25 * REGSZ)(x5)
sd x26, (26 * REGSZ)(x5)
sd x27, (27 * REGSZ)(x5)
sd x28, (28 * REGSZ)(x5)
sd x29, (29 * REGSZ)(x5)
sd x30, (30 * REGSZ)(x5)
sd x31, (31 * REGSZ)(x5)
# save privilege registers
# save mepc by x1, which has been saved
csrr x1, mepc
sd x1, (32 * REGSZ)(x5)

# call trap_handler
# Need set stack pointer?
la t0, trap_stack_top
ld sp, 0(t0)
call trap_handler

# use x5 as cpu state base address register
la x5, trap_cpu
# restore privilege registers
# restore mepc by x1, which will be restored later
ld x1, (32 * REGSZ)(x5)
csrw mepc, x1
# restore general purpose registers
# x0 ~ x4
ld x0, (0 * REGSZ)(x5)
ld x1, (1 * REGSZ)(x5)
ld x2, (2 * REGSZ)(x5)
ld x3, (3 * REGSZ)(x5)
ld x4, (4 * REGSZ)(x5)
# postpone the restoration of x5
# because it is being used as the base address register
# x6 ~ x31
ld x6, (6 * REGSZ)(x5)
ld x7, (7 * REGSZ)(x5)
ld x8, (8 * REGSZ)(x5)
ld x9, (9 * REGSZ)(x5)
ld x10, (10 * REGSZ)(x5)
ld x11, (11 * REGSZ)(x5)
ld x12, (12 * REGSZ)(x5)
ld x13, (13 * REGSZ)(x5)
ld x14, (14 * REGSZ)(x5)
ld x15, (15 * REGSZ)(x5)
ld x16, (16 * REGSZ)(x5)
ld x17, (17 * REGSZ)(x5)
ld x18, (18 * REGSZ)(x5)
ld x19, (19 * REGSZ)(x5)
ld x20, (20 * REGSZ)(x5)
ld x21, (21 * REGSZ)(x5)
ld x22, (22 * REGSZ)(x5)
ld x23, (23 * REGSZ)(x5)
ld x24, (24 * REGSZ)(x5)
ld x25, (25 * REGSZ)(x5)
ld x26, (26 * REGSZ)(x5)
ld x27, (27 * REGSZ)(x5)
ld x28, (28 * REGSZ)(x5)
ld x29, (29 * REGSZ)(x5)
ld x30, (30 * REGSZ)(x5)
ld x31, (31 * REGSZ)(x5)
# x5
ld x5, (6 * REGSZ)(x5)

mret
# j trap_entry

'''

TRAP_C = r'''
struct cpu trap_cpu;
u8 trap_stack[1 << 20];
void *trap_stack_top = &trap_stack[sizeof(trap_stack) - 1];

void trap_handler() {
  uart_print("[Trap Handler]\n");

  u64 mcause = csrr_mcause();
  switch (mcause)
  {
  case MCAUSE_INTR_M_TIMER:
  {
    // there exists runnable processes
    if (proc_list[0].state != PROC_STATE_NONE)
    {
      // assume proc-0 is the active process if there is no active process
      if (active_pid < 0)
      {
        active_pid = 0;
        trap_cpu = proc_list[0].cpu;
        uart_print("[Trap - M-mode Timer] Scheduler Init.\n");
      }

      // save cpu state for the active process
      proc_list[active_pid].cpu = trap_cpu;
      // suspend the active process
      proc_list[active_pid].state = PROC_STATE_READY;

      // iterate the processes from the next process, ending with the active process
      for (int ring_index = 1; ring_index <= PROC_TOTAL_COUNT; ring_index++)
      {
        int real_index = (active_pid + ring_index) % PROC_TOTAL_COUNT;
        struct proc *proc = &proc_list[real_index];
        // run this process if it is ready
        if (proc->state == PROC_STATE_READY)
        {
          uart_print("[Trap - M-mode PROC_STATE_READY]\n");
          trap_cpu = proc->cpu;
          active_pid = proc->pid;
          break;
        }
      }
    }
    set_timeout(10000000);
    break;
  }

  case MCAUSE_INTR_M_EXTER:
  {
    uart_print("[Trap - M-mode Exter]\n");
    break;
  }

  case MCAUSE_INNER_M_ILLEAGEL_INSTRUCTION:
  {
    uart_print("[Trap - M-mode Illeagel Instruction]\n");
    break;
  }

  default:
  {
    uart_print("[Trap - Default]\n");
    break;
  }
  }
}
'''

def gen_proc_header(procs=2):
	out = [
		PROC_H,
		'i32 active_pid;',
		'struct proc proc_list[%s] = {};' %(procs+3),

	]
	return out

def gen_procs(procs=2, interval=100000000, stack_mb=1):
	out = []
	for p in range(procs):
		out += [
			'u8 __proc_%s_stack[%s];' % (p, int(1024*1024*stack_mb) ),
			'void *__proc_%s_stack_top = &__proc_%s_stack[sizeof(__proc_%s_stack) - 1];' % (p,p,p),

			'void __proc_entry_%s(){' % p,
			'	uart_print("[PID = %s] proc entry!\\n");' % p,

			'while (true){',
			'	for (size_t i = 0; i < %s; i++){' % interval,
			'		uart_print("%s");' % p,
			'	}',
			'	uart_print("[PID = %s] Hello, Process Shceduler!\\n");' % p,
			'}}',
		]

	return out

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
  uart_print("[firmware_main set_timeout]\n");
  set_timeout(10000000);

  // setup M-mode trap vector
  csrw_mtvec((u64)trap_entry);
  // enable M-mode timer interrupt
  csrw_mie(MIE_MTIE);
  // enable MIE in mstatus
  csrs_mstatus(MSTAUTS_MIE);

  uart_print("[firmware_main waiting...]\n");
  while (1) {}
}
'''

def gen_firmware(procs=2):
	out = ['void procs_init(){']
	for p in range(procs):
		out += [
			'struct proc test_proc_%s = {' % p,
			'  .name = "test_proc_%s",' % p,
			'  .pid = %s,' % p,
			'  .hartid = 0,',
			'  .state = PROC_STATE_READY,',
			'  .cpu = {',
			'      .pc = (u64)__proc_entry_%s,' % p,
			'      .x2 = (u64)__proc_%s_stack_top,' % p,
			'  }};',
			'uart_print("[proc_init] proc_list:%s");' % p,
			#f'uart_printf(": pc is 0x%lx, stack_top is 0x%lx\\n", &test_proc_{p}, __proc_{p}_stack_top);',
			f'proc_list[{p}] = test_proc_{p};',
		]
	out.append('}')
	out.append(FIRMWARE_MAIN % procs)
	return out


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

static inline u64 mtime() {
  return readu64(MTIME);
}

static inline u64 mtimecmp_0() {
  return readu64(MTIMECMP_0);
}

static inline u64 set_timeout(u64 timeout) {
  writeu64(MTIMECMP_0, mtime() + timeout);
}

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

static inline void csrw_mtvec(const volatile u64 val) {
  asm volatile("csrw mtvec, %0" :: "r"(val));
}

static inline void csrw_mie(const volatile u64 val) {
  asm volatile("csrw mie, %0" :: "r"(val));
}

static inline u64 csrr_mstatus() {
  volatile u64 val;
  asm volatile("csrr %0, mstatus" : "=r"(val) :);
  return val;
}

static inline void csrw_mstatus(const volatile u64 val) {
  asm volatile("csrw mstatus, %0" :: "r"(val));
}

static inline void csrs_mstatus(const volatile u64 val) {
  asm volatile("csrs mstatus, %0" :: "r"(val));
}

static inline void csrc_mstatus(const volatile u64 val){
  asm volatile("csrc mstatus, %0" :: "r"(val));
}

static inline u64 csrr_mcause(){
  volatile u64 val;
  asm volatile("csrr %0, mcause" : "=r"(val) :);
  return val;
}

static inline u64 csrr_mepc(){
  volatile u64 val;
  asm volatile("csrr %0, mepc" : "=r"(val) :);
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

static void set_reg(u32 offset, u32 val){
  writeu8(uart_base_addr + offset, val);
}
static u32 get_reg(u32 offset){
  return readu8(uart_base_addr + offset);
}
static void uart_putc(u8 ch){
  set_reg(UART_THR_OFFSET, ch);
}
static void uart_print(char *str){
  while (*str) uart_putc(*str++);
}

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
'''

LINKER_SCRIPT = '''
ENTRY(_start)
MEMORY {} /* default */
. = 0x80000000;
SECTIONS {}
'''

def test():
	out = [ARCH, ARCH_ASM, UART, CPU, TIMER, LIBC
		gen_proc_header(),
		gen_procs(),
		INTERRUPTS,
		TRAP_C,
		gen_firmware()
	]
	c = '\n'.join(out)
	print(c)

	tmpld = '/tmp/linker.ld'
	open(tmpld,'wb').write(LINKER_SCRIPT.encode('utf-8'))

	tmps = '/tmp/trap.s'
	open(tmps,'wb').write(TRAP_S.encode('utf-8'))

	tmp = '/tmp/test.c'
	open(tmp,'wb').write(c.encode('utf-8'))
	cmd = [
		'riscv64-unknown-elf-gcc', '-mcmodel=medany', 
		'-ffreestanding', '-nostdlib', '-nostartfiles', '-nodefaultlibs',
		'-Wl,--no-relax', '-T',tmpld, '-Os', '-g', '-o', '/tmp/test.elf',
		tmps, tmp
	]
	print(cmd)
	subprocess.check_call(cmd)

	cmd = 'riscv64-unknown-elf-objcopy -O binary -S /tmp/test.elf /tmp/firmware.bin'
	print(cmd)
	subprocess.check_call(cmd.split())

	cmd = 'qemu-system-riscv64 -machine virt -smp 1 -m 2G -serial stdio -bios /tmp/firmware.bin -s -display none'
	if '--elf' in sys.argv:
		cmd = 'qemu-system-riscv64 -machine virt -smp 2 -m 2G -serial stdio -bios /tmp/test.elf -s -display none'
	print(cmd)
	subprocess.check_call(cmd.split())

if __name__ == '__main__':
	test()
