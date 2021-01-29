MCU=attiny2313a
AVRDUDE_MCU=t2313
F_CPU=8000000
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS= -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
TARGET=main
SRCS=main.c

all:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex

flash:
	avrdude -p ${AVRDUDE_MCU} -c arduino -U flash:w:${TARGET}.hex:i -F -P /dev/ttyUSB0 -b 19200

clean:
	rm -f *.bin *.hex
