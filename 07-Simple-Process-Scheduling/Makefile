CROSS_COMPILE := riscv64-unknown-elf-
CC := ${CROSS_COMPILE}gcc
AS := ${CROSS_COMPILE}as
LD := ${CROSS_COMPILE}ld
OBJCOPY := ${CROSS_COMPILE}objcopy

CLANG_SRC := firmware.c trap_handler.c proc.c kstring.c test_processes.c
ASM_SRC := clang_env_init.s trap.s
HEADERS := riscv_arch.h riscv_asm.h uart.h timer.h interrupts.h \
					proc.h kstring.h riscv_cpu.h

OBJECTS = $(addprefix bin/,$(ASM_SRC:.s=.asmo))
OBJECTS += $(addprefix bin/,$(CLANG_SRC:.c=.clango))

all: bin/firmware.bin

bin/firmware.bin: ${OBJECTS} ${HEADERS} clang_env_init.ld
	${LD} --no-relax -T clang_env_init.ld ${OBJECTS} -o bin/firmware
	${OBJCOPY} -O binary -S bin/firmware bin/firmware.bin

bin/%.asmo: %.s
	${AS} -g $< -o $@

bin/%.clango: %.c ${HEADERS}
	${CC} -mcmodel=medany -ffreestanding -c -O0 -g $< -o $@

run: bin/firmware.bin
	qemu-system-riscv64 -machine virt -smp 2 -m 2G -serial stdio -bios bin/firmware.bin -s -display none

debug: bin/firmware.bin
	qemu-system-riscv64 -machine virt -smp 2 -m 2G -serial stdio -bios bin/firmware.bin -s -S

clean:
	rm -rf bin/

$(shell mkdir -p bin)
