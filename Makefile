#
# Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
#
# 
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions 
#  are met:
#
#    Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the   
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
# Contents
# 1. Common paths and options
# 2. Settings for GCC + makefiles
#******************************************************************************

#
# Toolchain/library path. LIB_PATH is an exported environmental variable which
# shall point to the installation of toolchain
#
ifneq (${LIB_PATH},)
LIB_GCC=${LIB_PATH}/lib/gcc/arm-none-eabi/4.7.3/
LIB_C=${LIB_PATH}/arm-none-eabi/lib/
else
LIB_GCC=$(dir $(shell $(CC) $(CFLAGS) -print-libgcc-file-name))
LIB_C=$(dir $(shell $(CC) $(CFLAGS) -print-file-name=libc.a))
endif

#
# Target and Compiler definitions (Device and EVM specified by makefile)
#
TARGET=armv7a
COMPILER=gcc
BOOT=MMCSD

#
# Default Console type is set to UART to redirect all I/O operations
# to UART console. User can override this option to select semiHosting
# console while building the example by providing the following
# option "make CONSOLE=SEMIHOSTING" during compilation.
#
CONSOLE=UARTCONSOLE

#
# Convert Windows native style filenames to POSIX style filenames
#
CYGPATH=cypath
# Current path
ROOT = .
#
# Target directories to be built
#
#DRIVERS_BLD=${ROOT}/build/${TARGET}/${COMPILER}/${DEVICE}/drivers
#PLATFORM_BLD=${ROOT}/build/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/platform
#SYSCONFIG_BLD=${ROOT}/build/${TARGET}/${COMPILER}/${DEVICE}/system_config
#IPCLIB_BLD=${ROOT}/build/${TARGET}/${COMPILER}/${DEVICE}/ipclite
#UTILITY_BLD=${ROOT}/build/${TARGET}/${COMPILER}/utils
#USBLIB_BLD=${ROOT}/build/${TARGET}/${COMPILER}/${DEVICE}/usblib
#GRLIB_BLD=${ROOT}/build/${TARGET}/${COMPILER}/grlib
#NANDLIB_BLD=${ROOT}/build/${TARGET}/${COMPILER}/nandlib
#MMCSDLIB_BLD=${ROOT}/build/${TARGET}/${COMPILER}/mmcsdlib
#NORLIB_BLD=${ROOT}/build/${TARGET}/${COMPILER}/norlib

#
# Pre/recompiled library paths
#
#DRIVERS_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/drivers
#PLATFORM_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/platform
#SYSCONFIG_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/system_config
#IPCLIB_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/ipclite
#UTILITY_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/utils
#USBLIB_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/usblib
#GRLIB_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/grlib
#NANDLIB_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/nandlib
#MMCSDLIB_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/mmcsdlib
#NORLIB_BLD=${ROOT}/binary/${TARGET}/${COMPILER}/norlib

#
# Source code paths
#
#DRIVERS_SRC=${ROOT}/drivers
#PLATFORM_SRC=${ROOT}/platform/${EVM}
#SYSCONFIG_SRC=${ROOT}/system_config/${TARGET}
#IPCLIB_SRC=${ROOT}/ipclite/src
#UTILITY_SRC=${ROOT}/utils
#USBLIB_SRC=${ROOT}/usblib
#GRLIB_SRC=${ROOT}/grlib
#NANDLIB_SRC=${ROOT}/nandlib
#MMCSDLIB_SRC=${ROOT}/mmcsdlib
#FATFS_SRC=${ROOT}/third_party/fatfs
#NORLIB_SRC=${ROOT}/norlib
SYS_SRC = ${ROOT}/sys/*.c
#
# Include paths
#
IPATH=-I${ROOT}/include 
#      -I${ROOT}/include/hw \
#      -I${ROOT}/include/${TARGET}\
#      -I${ROOT}/include/${TARGET}/${DEVICE} \
#      -I${ROOT}/grlib/include \
#      -I${ROOT}/usblib/include \
#      -I${ROOT}/ipclite/include \
#      -I${ROOT}/nandlib/include \
#      -I${ROOT}/mmcsdlib/include \
#      -I${ROOT}/bootloader/include \
#      -I${ROOT}third_party/fatfs/src \
#      -I${ROOT}/norlib/include \
#      -I"${LIB_PATH}/include"

#
# Library paths
#
LPATH=-L"${LIB_C}" \
      -L"${LIB_GCC}" \
 #     -L${DRIVERS_BIN}/${TARGET_MODE} \
 #     -L${PLATFORM_BIN}/${TARGET_MODE} \
 #     -L${SYSCONFIG_BIN}/${TARGET_MODE} \
 #     -L${IPCLIB_BIN}/${TARGET_MODE} \
 #     -L${UTILITY_BIN}/${TARGET_MODE} \
 #     -L${USBLIB_BIN}/${TARGET_MODE} \
 #     -L${GRLIB_BIN}/${TARGET_MODE} \
 #     -L${NANDLIB_BIN}/${TARGET_MODE} \
 #     -L${MMCSDLIB_BIN}/${TARGET_MODE} \
 #     -L${NORLIB_BIN}/${TARGET_MODE}
 
# TFTP server path 
TFTP_PATH = $(HOME)/bbb/cmpt433/public/baremetal
# Object files path.
OBJ_PATH = /obj

#
# C compilation options
#
Debug_FLAG=-g
Release_FLAG=-g -O2
ifdef DEVICE
DEVICE_D=-D${DEVICE}
endif
ifdef EVM
EVM_D=-D${EVM}
endif

#
# C runtime library linker option
#
RUNTIMELIB = -lc -lgcc

#
#  rdimon runtime library linker option for semiHosting support
#
ifeq ($(CONSOLE), SEMIHOSTING)
CFLAGS+=--specs=rdimon.specs
RUNTIMELIB = -lrdimon -lc -lgcc
endif

CFLAGS=-mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -Wall ${IPATH}
CFLAGS+=-c ${${TARGET_MODE}_FLAG} -mlong-calls -fdata-sections -funsigned-char \
            -ffunction-sections -Wall ${IPATH} -Dgcc ${DEVICE_D} ${EVM_D} \
            -D SUPPORT_UNALIGNED -D ${BOOT} -D${CONSOLE}
#
# Defining the cross compiler tool prefix
#
ifndef PREFIX
PREFIX=$(HOME)/bbb/cmpt433/linaro-gcc/bin/arm-none-eabi-
endif

#
# Compiler, Linker and Archiver with respect to the toolchain
#
#TOOLCHAIN_PREFIX =$(HOME)/bbb/cmpt433/linaro-gcc/bin/arm-none-eabi-
CC=${PREFIX}gcc
LD=${PREFIX}ld
AR=${PREFIX}ar
BIN=$(PREFIX)objcopy

#
# Archiver options
#
ARFLAGS=-c -r

#
# Linker options
#
LDFLAGS=-e Entry -u Entry -u __aeabi_uidiv -u __aeabi_idiv --gc-sections

#
# Binary options
#
BINFLAGS=-O binary
                          

all: clean compile

compile:
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mv *.o* $(OBJ_PATH)
	$(LD) ${LDFLAGS} ${LPATH} -o $(BIN)/code1.out \
          -Map test.map $(OBJ_PATH)/*.o* \
          -T linker.lds
	$(BIN) $(BINFLAGS) $(BIN)/code1.out \
               $(BIN)/code1.bin
	cp $(BIN)/code1.bin $(TFTP_PATH)/download.bin

clean:
	rm -f *.map *.o *.bin *.elf $(BIN)/*.elf $(TFTP_PATH)/*.bin




















#CFLAGS = -mcpu=cortex-a8 -g -Wall
#LDFLAGS = -Map=test.map
#OPTIMIZATION_FLAGS= -O2
#TOOLCHAIN_PREFIX =$(HOME)/bbb/cmpt433/linaro-gcc/bin/arm-none-eabi-
#BIN = bin
#TFTP_PATH = $(HOME)/bbb/cmpt433/public/baremetal
#SYS_PATH = /sys
#OBJ_PATH = /obj
#SYS_SRC = $(SYS_PATH)/*.c

# all: clean bin

# compile: code1.c startup.s
	# $(TOOLCHAIN_PREFIX)gcc $(CFLAGS) $(OPTIMIZATION_FLAGS) -c code1.c -o $(OBJ_PATH)/code1.o
	# $(TOOLCHAIN_PREFIX)as $(CFLAGS) $(SYS_PATH)/startup.s -o $(OBJ_PATH)/startup.o

# link: compile
	
	# $(TOOLCHAIN_PREFIX)ld $(LDFLAGS) -Tlinker.lds code1.o startup.o -o $(BIN)/code1.elf 

# bin: link
	
	# $(TOOLCHAIN_PREFIX)objcopy --gap-fill=0xff -O binary $(BIN)/code1.elf $(TFTP_PATH)/download.bin

# deploy: check-env bin
	# scp code.bin $(REMOTEUSER)@$(REMOTEHOST):$(REMOTEPATH)

# check-env:
	# @if test -z "$$REMOTEUSER"; \
		# then echo "[!] environment variabel REMOTEUSER is not defined, enter something linke this: export REMOTEUSER=user)"; \
		# exit 1; \
	# fi; \

	# @if test -z "$$REMOTEHOST"; \
		# then echo "[!] environment variabel REMOTEHOST is not defined, enter something linke this: export REMOTEHOST=10.9.27.42)"; \
		# exit 1; \
	# fi; \

	# @if test -z "$$REMOTEPATH"; \
		# then echo "[!] environment variabel REMOTEPATH is not defined, enter something linke this: export REMOTEPATH=/tftpboot/code.bin)"; \
		# exit 1; \
	# fi; \

# clean:
	# rm -f *.map *.o *.bin *.elf $(BIN)/*.elf $(TFTP_PATH)/*.bin
