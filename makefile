#################################################
# MAKEFILE For STM32F3xxx Devices 				#
# (c) 20120930 Nemui Trinomius					#
# http://nemuisan.blog.bai.ne.jp				#
#################################################

# Environment Dependent!!! This Environment assure under WINDOWS !!
# Throw path into YOUR environments
export PATH = %SYSTEMROOT%;$(TOOLDIR)/bin;$(OCDIR);$(DFUDIR);$(MAKEDIR)

# Toolchain prefix (i.e arm-none-eabi -> arm-none-eabi-gcc.exe)
TCHAIN  = arm-none-eabi

# OpenOCD prefix
OCD		= openocd
# Select OpenOCD Transport
OCDMODE = SWD

# Select SWD Debug Adapter
DBG_ADAPTER  = STLINKV2

# Development Tools based on GNU Compiler Collection
DEVTOOL = LAUNCHPAD
#DEVTOOL = BLEEDING_EDGE
#DEVTOOL = YAGARTO
#DEVTOOL = DEVKITARM
#DEVTOOL = SOURCERY

# Check BuildTools
ifeq ($(DEVTOOL),LAUNCHPAD)
 TOOLDIR = C:/Program Files (x86)/GNU Tools ARM Embedded/4.9 2015q1
 NANOLIB = --specs=nano.specs
 NANOLIB += -u _printf_float
#  NANOLIB += -u _scanf_float
 REMOVAL = rm
else ifeq ($(DEVTOOL),BLEEDING_EDGE)
 TOOLDIR = C:/Devz/ARM/Bleeding-edge
 REMOVAL = rm
else ifeq ($(DEVTOOL),YAGARTO)
 TOOLDIR = C:/Devz/ARM/Yagarto
 REMOVAL = rm
else ifeq ($(DEVTOOL),DEVKITARM)
 TOOLDIR = C:/Devz/ARM/devkitARM
 REMOVAL = rm
else ifeq ($(DEVTOOL),SOURCERY)
 TOOLDIR = C:/Devz/ARM/Sourcery
 REMOVAL = rm
else
 $(error SET BUILD-TOOLS AT FIRST!!)
endif

# Set UNIX-Like Tools(CoreUtils) Directory
MAKEDIR = C:/ARM_dev/coreutils-5.3.0/bin

# Set Flasher and Debugger
OCDIR	= C:/Devz/ARM/OCD
#OCD_DBG = -c "debug_level 3"
ifeq ($(DBG_ADAPTER),VERSALOON)
OCD_ARG = -s $(OCDIR)/tcl						\
		  -f interface/vsllink_swd.cfg			\
		  -f target/stm32f3x_flash.cfg
else ifeq ($(DBG_ADAPTER),JTAGKEY2_SWD)
OCD_ARG = -s $(OCDIR)/tcl						\
		  -f interface/jtagkey2_swd.cfg 		\
		  -f target/stm32f3x_flash.cfg
else ifeq ($(DBG_ADAPTER),JLINK_SWD)
OCD_ARG = -s $(OCDIR)/tcl						\
		  -f interface/jlink_swd.cfg 			\
		  -f target/stm32f3x_flash.cfg
else ifeq ($(DBG_ADAPTER),STLINKV2)
OCD_ARG = -s $(OCDIR)/tcl						\
		  -f target/stm32f3discovery_flash.cfg
endif
OCD_CMD = $(OCD_DBG) $(OCD_ARG)


#Set DFUse Directory
DFUDIR	= C:/Devz/ARM/ST/DFUse/BIN
#Set Shell Definitions
WSHELL  = cmd
MSGECHO = echo.exe
#Set GDB/Insight Directory
GDBDIR  = C:/Devz/ARM/insight/bin
INSIGHT = $(GDBDIR)/arm-none-eabi-insight
# Environment Dependent!!!


# OPTIMIZE Definition
OPTIMIZE		= s
#OPTIMIZE		= 1

# FPU Definition
ifeq ($(DEVTOOL),LAUNCHPAD)
 # Launchpad has hard-float library in free!
 USING_FPU		= -mfloat-abi=hard  -mfpu=fpv4-sp-d16
 #USING_FPU		= -mfloat-abi=softfp  -mfpu=fpv4-sp-d16
 #USING_FPU		= -mfloat-abi=soft
else ifeq ($(DEVTOOL),BLEEDING_EDGE)
 # Bleeding-edge has hard-float library in free!
 USING_FPU		= -mfloat-abi=hard  -mfpu=fpv4-sp-d16
 #USING_FPU		= -mfloat-abi=softfp  -mfpu=fpv4-sp-d16
 #USING_FPU		= -mfloat-abi=soft
else
 USING_FPU		= -mfloat-abi=softfp  -mfpu=fpv4-sp-d16
 #USING_FPU		= -mfloat-abi=soft
endif

# GCC LTO Specific Option
ifneq ($(OPTIMIZE),0)
USE_LTO			= -flto-partition=none -fipa-sra
#USE_LTO			= -flto -fipa-sra
endif
# GCC Version Specific Options
ALIGNED_ACCESS	= -mno-unaligned-access
#FPREC_CONST	    = -fsingle-precision-constant
#ARMV7M_BOOST    = -mslow-flash-data


# Semihosting Definition
#USING_HOSTAGE   = USE_SEMIHOSTING
ifeq ($(USING_HOSTAGE),USE_SEMIHOSTING)
SEMIHOST_LIB = --specs=rdimon.specs -lrdimon
else
START_LIB    = -nostartfiles
endif


# Apprication Version
APP_VER = W.I.P


# Basic definition
EVAL_BOARD    	= USE_NUCLEO_F303K8
MPU_CLASS		= STM32F3XX
MPU_MODEL		= STM32F303x8
SUBMODEL		= STM32F303K8
HSE_CLOCK 		= 8000000
PERIF_DRIVER    = USE_HAL_DRIVER
#USE_TOUCH_SENCE = USE_ADS7843


# Use Display Device?
#Software/Hardware SPI
#USE_DISPLAY		= USE_ILI9163x_SPI_TFT

# For JPEG Support
#USE_JPEG_LIB    = USE_TINYJPEG_LIB
#USE_JPEG_LIB    = USE_IJG_LIB

# For PNG Support
#USE_PNG_LIB		= USE_LIBPNG

# For GIF Support
#USE_GIF_LIB		= USE_GIFLIB


# Use Display Fonts? 
#USE_FONTSIZE    = FONT8x8
#USE_FONTSIZE    = FONT10x10
#USE_FONTSIZE    = FONT_DIGIT
#USE_KANJI		= USE_KANJIFONT

# Use FreeRTOS?
OS_SUPPORT		= BARE_METAL
#OS_SUPPORT		= USE_FREERTOS


# Synthesis makefile Defines
DEFZ = $(MPU_CLASS) $(MPU_MODEL) $(SUBMODEL) $(EVAL_BOARD) $(PERIF_DRIVER) $(VECTOR_START) \
	   $(USING_HOSTAGE) $(OS_SUPPORT) $(USE_EXT_SRAM) $(USE_EXT_SDRAM) $(USE_EXT_HEAP)
# Defines if Display and misc Drivers
DEFZ += $(USE_DISPLAY) $(USE_FONTSIZE) $(USE_KANJI) $(USE_TOUCH_SENCE)  $(USE_XMSTN)	   \
        $(USE_JPEG_LIB) $(USE_PNG_LIB) $(USE_GIF_LIB) $(USE_AUDIO_LIB)  				   \
		$(USE_SOUND_MP3)  $(USE_SOUND_WAV)
SYNTHESIS_DEFS	= $(addprefix -D,$(DEFZ)) 							\
				 -DARM_MATH_CM4										\
				 -DPACK_STRUCT_END=__attribute\(\(packed\)\) 		\
				 -DALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
				 -DMPU_SUBMODEL=\"$(SUBMODEL)\"						\
				 -DAPP_VERSION=\"$(APP_VER)\"						\
				 -DHSE_VALUE=$(HSE_CLOCK)UL

# TARGET definition
TARGET 		= main
TARGET_ELF  = $(TARGET).elf
TARGET_SREC = $(TARGET).s19
TARGET_HEX  = $(TARGET).hex
TARGET_BIN  = $(TARGET).bin
TARGET_LSS  = $(TARGET).lss
#TARGET_DFU  = $(TARGET).dfu
TARGET_SYM  = $(TARGET).sym

# define CMSIS LIBRARY PATH
FWLIB  			= ./lib/STM32F3xx_HAL_Driver
USBLIB 			= ./lib/STM32_USB_Device_Library
CMSISLIB 		= ./lib/CMSIS
CMSIS_DEVICE 	= $(CMSISLIB)/Device/ST/STM32F3xx
CMSIS_CORE		= $(CMSISLIB)/Include

# include PATH
INCPATHS	 = 	./							\
				./inc						\
				$(FWLIB)/inc  				\
				$(USBLIB)/Core/inc			\
				$(CMSIS_DEVICE)/Include		\
				$(CMSIS_CORE)				\
				$(LIBINCDIRS)
INCLUDES     = $(addprefix -I ,$(INCPATHS))

# Set library PATH
LIBPATHS     = $(FWLIB) $(USBLIB)
LIBRARY_DIRS = $(addprefix -L,$(LIBPATHS))
# if you use math-library, put "-lm" 
MATH_LIB	 =	-lm

# LinkerScript PATH
LINKER_PATH =  ./lib/linker
LINKER_DIRS = $(addprefix -L,$(LINKER_PATH)) 

# Object definition
OBJS 	 = $(CFILES:%.c=%.o) 
LIBOBJS  = $(LIBCFILES:%.c=%.o) $(SFILES:%.s=%.o)

# C code PATH
SOURCE  = ./src
CFILES = \
 $(SOURCE)/$(TARGET).c 				\
 $(SOURCE)/syscalls.c				\
 $(SOURCE)/stm32f3xx_it.c			\
 $(SOURCE)/hw_config.c				\
 $(SOURCE)/uart_support.c			\
 $(SOURCE)/timer.c					\
 $(SOURCE)/myMath.c					\
 $(SOURCE)/MPU9250.c				\
 $(SOURCE)/AHRS.c					\
 $(SOURCE)/motor.c


#/*----- Display library PATH -----*/	
DISPLAY_LIB	= ./lib/display
ifneq ($(USE_DISPLAY),)
include $(DISPLAY_LIB)/display_cfg.mk
endif

#/*----- FONX2 Driver library PATH -----*/	
FONTX2_LIB	= ./lib/FONTX2
ifneq ($(USE_FONTSIZE),)
include $(FONTX2_LIB)/fontx2_drv.mk
endif

#/*----- xMSTN Display library PATH -----*/	
xMSTN_LIB	= ./lib/xMSTN
ifneq ($(USE_XMSTN),)
include $(xMSTN_LIB)/xmstn_drv.mk
endif


#/*----- FatFs library PATH -----*/	
FATFS = ./lib/ff
LIBINCDIRS += $(FATFS)
#CFILES += \
# $(FATFS)/ff.c 						\
# $(FATFS)/mmc_stm32f4.c 				\
# $(FATFS)/ff_rtc_if.c 					\
# $(FATFS)/option/cc932.c

#/*----- STARTUP code PATH -----*/
STARTUP_DIR = $(CMSIS_DEVICE)/Source/Templates/gcc
ifeq ($(OS_SUPPORT),USE_FREERTOS)
SFILES += \
	$(SOURCE)/startup_$(MPU_MODEL)_rtos.s
else
SFILES += \
	$(STARTUP_DIR)/startup_$(MPU_MODEL).s
endif



#/*----- STM32 library PATH -----*/
LIBCFILES = \
 $(FWLIB)/src/stm32f3xx_hal.c			\
 $(FWLIB)/src/stm32f3xx_hal_cortex.c	\
 $(FWLIB)/src/stm32f3xx_hal_rcc_ex.c	\
 $(FWLIB)/src/stm32f3xx_hal_rcc.c		\
 $(FWLIB)/src/stm32f3xx_hal_gpio.c		\
 $(FWLIB)/src/stm32f3xx_hal_uart.c		\
 $(FWLIB)/src/stm32f3xx_hal_tim.c		\
  $(FWLIB)/src/stm32f3xx_hal_i2c.c		\
 ./src/system_stm32f3xx.c
 
#/*----- STM32 Debug library -----*/
ifeq ($(OPTIMIZE),0)
CFILES += \
 ./lib/IOView/stm32f3xx_io_view.c
else
endif

# TOOLCHAIN SETTING
CC 			= $(TCHAIN)-gcc
CPP 		= $(TCHAIN)-g++
OBJCOPY 	= $(TCHAIN)-objcopy
OBJDUMP 	= $(TCHAIN)-objdump
SIZE 		= $(TCHAIN)-size
AR 			= $(TCHAIN)-ar
LD 			= $(TCHAIN)-gcc
NM 			= $(TCHAIN)-nm
REMOVE		= $(REMOVAL) -f
REMOVEDIR 	= $(REMOVAL) -rf

# C and ASM FLAGS
CFLAGS  = -MD -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mthumb -mlittle-endian $(ALIGNED_ACCESS) $(ARMV7M_BOOST) $(FPREC_CONST)
CFLAGS += -mno-sched-prolog $(USING_FPU)
CFLAGS += -std=gnu99
CFLAGS += -gdwarf-2 -O$(OPTIMIZE) $(USE_LTO) $(NANOLIB) $(SEMIHOST_LIB)
CFLAGS += -fno-strict-aliasing -fsigned-char
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-schedule-insns2
CFLAGS += --param max-inline-insns-single=1000
CFLAGS += -fno-common -fno-hosted
CFLAGS += -Wall -Wno-array-bounds -Wno-unused-but-set-variable
#CFLAGS += -Wdouble-promotion
#CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CFLAGS += $(SYNTHESIS_DEFS)  

# Linker FLAGS
LDFLAGS  = -mcpu=cortex-m4 -march=armv7e-m -mthumb
LDFLAGS += -u g_pfnVectors -Wl,-static -Wl,--gc-sections $(START_LIB)
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += $(LIBRARY_DIRS) $(LINKER_DIRS) $(MATH_LIB)
LDFLAGS +=-T$(LINKER_PATH)/$(SUBMODEL).ld

# Object Copy and dfu generation FLAGS
OBJCPFLAGS = -O
OBJDUMPFLAGS = -h -S -C
DFU	  = hex2dfu
DFLAGS = -w


# Build Object
all: gccversion clean build buildinform sizeafter
build: $(TARGET_ELF) $(TARGET_LSS) $(TARGET_SYM) $(TARGET_HEX) $(TARGET_SREC) $(TARGET_BIN)

.SUFFIXES: .o .c .s   

$(TARGET_LSS): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Disassemble: $@
	$(OBJDUMP) $(OBJDUMPFLAGS) $< > $@ 
$(TARGET_SYM): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Symbol: $@
	$(NM) -n $< > $@
$(TARGET).hex: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) ihex $^ $@    
$(TARGET).s19: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) srec $^ $@ 
$(TARGET).bin: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) binary $< $@ 
$(TARGET).dfu: $(TARGET).hex
	@$(MSGECHO)
	@$(MSGECHO) Make STM32 dfu: $@
	$(DFU) $(DFLAGS) $< $@
	@$(MSGECHO)
$(TARGET).elf: $(OBJS) $(SUBMODEL)_lib.a
	@$(MSGECHO) Link: $@
	$(LD) $(CFLAGS) $(LDFLAGS) $^ -o $@
	@$(MSGECHO)

$(SUBMODEL)_lib.a: $(LIBOBJS)
	@$(MSGECHO) Archive: $@
	$(AR) cr $@ $(LIBOBJS)    
	@$(MSGECHO)
.c.o:
	@$(MSGECHO) Compile: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)
.s.o:
	@$(MSGECHO) Assemble: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)

# Object Size Informations
sizeafter:
	@$(MSGECHO) 
	@$(MSGECHO) Built Object Informations:
	@$(MSGECHO) === Total Binary Size ===
	@$(SIZE) $(TARGET).hex
	@$(MSGECHO) === Verbose ELF Size ===
	@$(SIZE) $(TARGET).elf
	@$(SIZE) -A -x $(TARGET).elf

# Display compiler version information.
gccversion : 
	@$(CC) --version
	@$(MSGECHO) 

buildinform :
	@$(MSGECHO) 
	@$(MSGECHO) 
	@$(MSGECHO) Built Informations:
	@$(MSGECHO) USING_SYSTEM = $(OS_SUPPORT)
	@$(MSGECHO) USING_DISPLAY = $(USE_DISPLAY)
	@$(MSGECHO) USING_DEVBOARD = $(EVAL_BOARD)

# Flash and Debug Program
debug :
	$(WSHELL) /c start /B $(INSIGHT) $(TARGET).elf
	$(OCD) $(OCD_CMD) -c "reset halt" -c"arm semihosting enable"
program :
	$(OCD) $(OCD_CMD) -c "mt_flash $(TARGET).elf"
#	$(OCD) $(OCD_CMD) -c "eraser"
#	$(OCD) $(OCD_CMD) -c "mt_flash_bin $(TARGET).bin 0x08000000"


# Drop files into dust-shoot
.PHONY clean:
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(SUBMODEL)_lib.a
	$(REMOVE) $(wildcard *_lib.a)
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).s19
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).dfu
	$(REMOVE) $(wildcard *.stackdump)
	$(REMOVE) $(OBJS)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(LIBOBJS)
	$(REMOVE) $(LST)
	$(REMOVE) $(CFILES:.c=.lst)
	$(REMOVE) $(CFILES:.c=.d)
	$(REMOVE) $(LIBCFILES:.c=.lst)
	$(REMOVE) $(LIBCFILES:.c=.d)
	$(REMOVE) $(SFILES:.s=.lst)
	$(REMOVE) $(wildcard ./lib/IOView/*.d)
	$(REMOVE) $(wildcard ./lib/IOView/*.lst)
	$(REMOVE) $(wildcard ./lib/IOView/*.o)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.d)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.lst)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.o)
	$(REMOVE) $(wildcard $(DISPLAY_MCU_SRC)/*.d)
	$(REMOVE) $(wildcard $(DISPLAY_MCU_SRC)/*.lst)
	$(REMOVE) $(wildcard $(DISPLAY_MCU_SRC)/*.o)
	$(REMOVE) $(wildcard $(FATFS)/*.d)
	$(REMOVE) $(wildcard $(FATFS)/*.lst)
	$(REMOVE) $(wildcard $(FATFS)/*.o)
	$(REMOVE) $(wildcard $(CMSIS_DEVICE)/*.d)
	$(REMOVE) $(wildcard $(CMSIS_DEVICE)/*.lst)
	$(REMOVE) $(wildcard $(CMSIS_DEVICE)/*.o)
	$(REMOVEDIR) .dep
	@$(MSGECHO)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program
