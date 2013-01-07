##############################################################################
APP_NAME = simplemesh

#HAL = ATMEGA128RFA1
#HAL = ATMEGA328P
HAL = LPC1114

#PHY = ATMEGA128RFA1
PHY = AT86RF230
#PHY = AT86RF231

#FREQ = 8_MHz
FREQ = 12_MHz

#BOARD = RCB_MEGARF
#BOARD = TRT_DISK
BOARD = RADIOBLOCKS

##############################################################################
.PHONY: all clean size

STACK_PATH = ./../../..
APP_PATH = ./..

include $(STACK_PATH)/Makerules

SRCS += \
  $(APP_PATH)/src/serial.c \
  $(APP_PATH)/src/command.c \
  $(APP_PATH)/src/ib.c

INCLUDES += \
  -I$(APP_PATH)/include

ifeq ($(HAL), LPC1114)
  LDFLAGS += -Wl,--script=$(APP_PATH)/ldscript/lpc1114.ld
endif

OBJS = $(notdir %/$(subst .c,.o, $(SRCS)))

all: $(APP_NAME).elf $(APP_NAME).hex $(APP_NAME).bin size

$(APP_NAME).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(APP_NAME).hex: $(APP_NAME).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex -R .eeprom $^ $@

$(APP_NAME).bin: $(APP_NAME).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary -R .eeprom $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

size: $(APP_NAME).elf
	@echo
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(OBJS) $(APP_NAME).elf $(APP_NAME).hex $(APP_NAME).bin

