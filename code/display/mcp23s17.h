/**
 * Copyright (C) 2021 https://github.com/d3vf1x
 * 
 * based on work from: https://github.com/piface/libmcp23s17
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 * MCP Values
 */

// Register addresses
#define IODIRA 0x00  // I/O direction A
#define IODIRB 0x01  // I/O direction B
#define IPOLA 0x02  // I/O polarity A
#define IPOLB 0x03  // I/O polarity B
#define GPINTENA 0x04  // interupt enable A
#define GPINTENB 0x05  // interupt enable B
#define DEFVALA 0x06  // register default value A (interupts)
#define DEFVALB 0x07  // register default value B (interupts)
#define INTCONA 0x08  // interupt control A
#define INTCONB 0x09  // interupt control B
#define IOCON 0x0A  // I/O config (also 0x0B)
#define GPPUA 0x0C  // port A pullups
#define GPPUB 0x0D  // port B pullups
#define INTFA 0x0E  // interupt flag A (where the interupt came from)
#define INTFB 0x0F  // interupt flag B
#define INTCAPA 0x10  // interupt capture A (value at interupt is saved here)
#define INTCAPB 0x11  // interupt capture B
#define GPIOA 0x12  // port A
#define GPIOB 0x13  // port B
#define OLATA 0x14  // output latch A
#define OLATB 0x15  // output latch B

#define BANK_OFF 0x00  // addressing mode
#define BANK_ON 0x80
#define INT_MIRROR_ON 0x40  // interupt mirror (INTa|INTb)
#define INT_MIRROR_OFF 0x00
#define SEQOP_OFF 0x20  // incrementing address pointer
#define SEQOP_ON 0x00
#define DISSLW_ON 0x10  // slew rate
#define DISSLW_OFF 0x00
#define HAEN_ON 0x08  // hardware addressing
#define HAEN_OFF 0x00
#define ODR_ON 0x04  // open drain for interupts
#define ODR_OFF 0x00
#define INTPOL_HIGH 0x02  // interupt polarity
#define INTPOL_LOW 0x00

#define GPIO_INTERRUPT_PIN 25

/* 7-Segment values */
#define DIGIT_0 0xcf
#define DIGIT_1 0x3
#define DIGIT_2 0x5d
#define DIGIT_3 0x5b
#define DIGIT_4 0x93
#define DIGIT_5 0xda
#define DIGIT_6 0xde
#define DIGIT_7 0x43
#define DIGIT_8 0xdf
#define DIGIT_9 0xdb

#define DIGIT_POS_0 0x1
#define DIGIT_POS_1 0x2
#define DIGIT_POS_2 0x4
#define DIGIT_POS_3 0x8

#define DECIMAL_POINT 0x20