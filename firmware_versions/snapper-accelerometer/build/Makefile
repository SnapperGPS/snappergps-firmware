#****************************************************************************
# Makefile
# SnapperGPS
# November 2020
#****************************************************************************

# Set the toolpath parameter to the location of the GNU Arm Embedded Toolchain

TOOLPATH = /Applications/ARM/bin/

# Determine the path to the shared code

SHARED = ../../../shared_library/

# These are the locations of the source and header files

INC = $(SHARED)/cmsis $(SHARED)/device/inc $(SHARED)/emlib/inc $(SHARED)/emusb/inc $(SHARED)/inc ../inc 
SRC = $(SHARED)/device/src $(SHARED)/emlib/src $(SHARED)/emusb/src $(SHARED)/src ../src

# Set the name of the output files

FILENAME = acceleration-snapper

# This is the location of the resulting object files

OBJPATH = ./objects/

# Set the target EFM32HG to that used in Snapper

TARGET = EFM32HG310F64

# The following code generates the list of objects and the search path of source and header files

VPATH = $(SRC)

IFLAGS = $(foreach d, $(INC), -I$d)

_CSRC = $(notdir $(foreach d, $(SRC), $(wildcard $d/*.c)))
_SSRC = $(notdir $(foreach d, $(SRC), $(wildcard $d/*.s)))

_OBJ = $(_CSRC:.c=.o) $(_SSRC:.s=.o)

OBJ = $(sort $(foreach d, $(_OBJ), $(OBJPATH)$d))

DEP = $(OBJ:.o=.d)

# These are the compilation settings

CC = $(TOOLPATH)arm-none-eabi-gcc

CSIZE = $(TOOLPATH)arm-none-eabi-size

COBJCOPY = $(TOOLPATH)arm-none-eabi-objcopy

COBJDUMP = $(TOOLPATH)arm-none-eabi-objdump

CFLAGS = -mcpu=cortex-m0plus -mthumb -g0 -Wall '-D$(TARGET)=1' '-D__STACK_SIZE=512' '-D__HEAP_SIZE=128'

DFLAGS = -MMD

# Finally the build rules

$(OBJPATH)%.o: %.c
	@mkdir -p $(OBJPATH)
	@echo 'Building' $@
	@$(CC) $(CFLAGS) $(DFLAGS) -O2 -ffunction-sections -fdata-sections -std=c99 -c -o "$@" "$<" $(IFLAGS)

$(OBJPATH)%.o: %.s
	@mkdir -p $(OBJPATH)
	@echo 'Building' $@
	@$(CC) $(CFLAGS) $(DFLAGS) -x assembler-with-cpp -c -o "$@" "$<" $(IFLAGS)

$(FILENAME).bin: $(OBJ)
	@echo 'Building' $(FILENAME).axf
	@$(CC) $(CFLAGS) $(DFLAGS) -T "snapper.ld" -Xlinker --gc-sections -Xlinker -Map="$(FILENAME).map" --specs=nano.specs -o $(FILENAME).axf $(OBJ) -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@$(COBJDUMP) -d -t -S "$(FILENAME).axf" > "$(FILENAME).lst" 
	@echo 'Building' $(FILENAME).hex
	@$(COBJCOPY) -O ihex "$(FILENAME).axf" "$(FILENAME).hex"
	@echo 'Building' $(FILENAME).bin
	@$(COBJCOPY) -O binary "$(FILENAME).axf" "$(FILENAME).bin"
	@$(CSIZE) -A "$(FILENAME).axf"

-include $(DEP)

.PHONY: clean
clean:
	rm -f $(OBJPATH)*.o
	rm -f $(OBJPATH)*.d
	rm -f $(FILENAME).axf
	rm -f $(FILENAME).bin
	rm -f $(FILENAME).hex
	rm -f $(FILENAME).map
