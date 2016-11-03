CFLAGS = -mcpu=cortex-a8 -g -Wall
LDFLAGS = -Map=test.map
OPTIMIZATION_FLAGS= -O2
TOOLCHAIN_PREFIX =$(HOME)/bbb/cmpt433/linaro-gcc/bin/arm-none-eabi-
BIN = bin
TFTP_PATH = $(HOME)/bbb/cmpt433/public/baremetal

all: clean bin

compile: code1.c startup.s
	$(TOOLCHAIN_PREFIX)gcc $(CFLAGS) $(OPTIMIZATION_FLAGS) -c code1.c -o code1.o
	$(TOOLCHAIN_PREFIX)as $(CFLAGS) startup.s -o startup.o

link: compile
	
	$(TOOLCHAIN_PREFIX)ld $(LDFLAGS) -Tlinker.lds code1.o startup.o -o $(BIN)/code1.elf 

bin: link
	
	$(TOOLCHAIN_PREFIX)objcopy --gap-fill=0xff -O binary $(BIN)/code1.elf $(TFTP_PATH)/download.bin

deploy: check-env bin
	scp code.bin $(REMOTEUSER)@$(REMOTEHOST):$(REMOTEPATH)

check-env:
	@if test -z "$$REMOTEUSER"; \
		then echo "[!] environment variabel REMOTEUSER is not defined, enter something linke this: export REMOTEUSER=user)"; \
		exit 1; \
	fi; \

	@if test -z "$$REMOTEHOST"; \
		then echo "[!] environment variabel REMOTEHOST is not defined, enter something linke this: export REMOTEHOST=10.9.27.42)"; \
		exit 1; \
	fi; \

	@if test -z "$$REMOTEPATH"; \
		then echo "[!] environment variabel REMOTEPATH is not defined, enter something linke this: export REMOTEPATH=/tftpboot/code.bin)"; \
		exit 1; \
	fi; \

clean:
	rm -f *.map *.o *.bin *.elf $(BIN)/*.elf $(TFTP_PATH)/*.bin
