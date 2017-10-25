CROSS_COMPILE := avr-
ARDUINO := ./Arduino

DEFINES = -DARDUINO=180 \
	  -DF_CPU=16000000UL

INCLUDES = -I$(ARDUINO)/hardware/arduino/avr/cores/arduino/ \
	   -I$(ARDUINO)/hardware/arduino/avr/variants/eightanaloginputs \
	   -I$(ARDUINO)/hardware/arduino/avr/libraries/SPI/src/ \
	   -I$(ARDUINO)/hardware/arduino/avr/libraries/EEPROM/src/

INO_INCLUDES = -include iface_nrf24l01.h \
	       -include common.h \
	       -include prototypes.h

CFLAGS = $(DEFINES) $(INCLUDES) -mmcu=atmega328p \
	 -fdata-sections -ffunction-sections \
	 -Os

CXXFLAGS = $(CFLAGS) -std=c++11 -fno-threadsafe-statics -fno-rtti -fno-exceptions
LDFLAGS = -Wl,-gc-sections -Wl,-Map=nRF24_multipro.map -Wl,--warn-common

INO_SRC = $(wildcard *.ino)

ARDUINO_SRC = $(ARDUINO)/hardware/arduino/avr/cores/arduino/hooks.c \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/WMath.cpp \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/WInterrupts.c \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/wiring.c \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/wiring_digital.c \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/wiring_analog.c \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/HardwareSerial.cpp \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/HardwareSerial0.cpp \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/Print.cpp \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/abi.cpp \
	      $(ARDUINO)/hardware/arduino/avr/cores/arduino/main.cpp


OBJS = $(INO_SRC:.ino=.o) $(patsubst %.cpp,%.o,$(ARDUINO_SRC:.c=.o))

nRF24_multipro.hex:

nRF24_multipro.elf: $(OBJS)
	$(CROSS_COMPILE)g++ $(CXXFLAGS) $(LDFLAGS) $(OBJS) -o $@

prototypes.h: $(INO_SRC)
	cat *.ino | tr -d '\r' | \
		sed -rn '/^[a-z]+.*\)[ ]*$$/ {s/$$/;/; p}; /^#define/ p' > prototypes.h || rm prototypes.h

%.o: %.ino prototypes.h common.h
	$(CROSS_COMPILE)g++ $(CXXFLAGS) $(INO_INCLUDES) -c -x c++ $< -o $@

%.o: %.cpp
	$(CROSS_COMPILE)g++ $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

%.hex: %.elf
	$(CROSS_COMPILE)objcopy -O ihex $< $@

clean:
	-rm $(OBJS) *.elf *.hex *.map prototypes.h

.PHONY: clean
