##############################################################################
APP_NAME = demo

#HAL = ATMEGA128RFA1
#HAL = ATMEGA328P
HAL = LPC111X

#PHY = ATMEGA128RFA1
PHY = AT86RF230
#PHY = AT86RF231

#FREQ = 8_MHz
FREQ = 12_MHz

#BOARD = RCB_MEGARF
#BOARD = TRT_DISK

##############################################################################
.PHONY: all clean size

STACK_PATH = ../..
OBJ_PATH = objs

include ../../Makerules

SRCS += \
  demo.c

ifeq ($(HAL), LPC111X)
  LDFLAGS += -Wl,--script=test.ld
endif

OBJS = $(addprefix $(OBJ_PATH)/, $(notdir %/$(subst .c,.o, $(SRCS))))

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

$(OBJ_PATH)/%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

$(OBJS): | $(OBJ_PATH)

$(OBJ_PATH):
	mkdir $(OBJ_PATH)

size: $(APP_NAME).elf
	@echo
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(OBJ_PATH) $(APP_NAME).elf $(APP_NAME).hex $(APP_NAME).bin

