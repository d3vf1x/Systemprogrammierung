/**
 * Copyright (C) 2021 https://github.com/d3vf1x
 * 
 * based on work from Galileo53: https://github.com/eBUS/ttyebus
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
 * 
 * This file contains all Addresses of the BMC2835 Hardware.
 */

// Physical base address for the raspberry pi 2 and 3
#define RASPI_23_PERI_BASE   0x3F000000             

// BCM2835 base address
#define SYST_BASE            0x00003000
#define DMA_BASE             0x00007000
#define IRQ_BASE             0x0000B000
#define CLK_BASE             0x00101000
#define GPIO_BASE            0x00200000
#define UART0_BASE           0x00201000
#define PCM_BASE             0x00203000
#define SPI0_BASE            0x00204000
#define I2C0_BASE            0x00205000
#define PWM_BASE             0x0020C000
#define UART1_BASE           0x00215000
#define I2C1_BASE            0x00804000
#define I2C2_BASE            0x00805000
#define DMA15_BASE           0x00E05000

// GPIO register
#define GPIO_INPUT          0
#define GPIO_OUTPUT         1
#define GPIO_ALT_0          4
#define GPIO_ALT_1          5
#define GPIO_ALT_2          6
#define GPIO_ALT_3          7
#define GPIO_ALT_4          3
#define GPIO_ALT_5          2
 
#define GPIO_FSEL0          (GpioAddr+0x00)
#define GPIO_FSEL1          (GpioAddr+0x04)
#define GPIO_FSEL2          (GpioAddr+0x08)
#define GPIO_FSEL3          (GpioAddr+0x0C)
#define GPIO_FSEL4          (GpioAddr+0x10)
#define GPIO_FSEL5          (GpioAddr+0x14)

#define GPIO_PULL           (GpioAddr+0x94)                       // Pull up/pull down
#define GPIO_PULLCLK0       (GpioAddr+0x98)                       // Pull up/pull down clock
#define GPIO_PULLCLK1       (GpioAddr+0x9C)                       // Pull up/pull down clock
#define GPIO_BANK           (Gpio >> 5)
#define GPIO_BIT            (1 << (Gpio & 0x1F))

#define GPIO_PULL_OFF       0
#define GPIO_PULL_DOWN      1
#define GPIO_PULL_UP        2

// PL011 UART register (16C650 type)
#define UART_DATA         (UartAddr+0x00)
#define UART_RX_ERR       (UartAddr+0x04)
#define UART_FLAG         (UartAddr+0x18)
#define UART_ILPR         (UartAddr+0x20)
#define UART_INT_BAUD     (UartAddr+0x24)
#define UART_FRAC_BAUD    (UartAddr+0x28)
#define UART_LINE_CTRL    (UartAddr+0x2C)
#define UART_CTRL         (UartAddr+0x30)
#define UART_FIFO_LEVEL   (UartAddr+0x34)
#define UART_INT_MASK     (UartAddr+0x38)
#define UART_RAW_INT      (UartAddr+0x3C)
#define UART_INT_STAT     (UartAddr+0x40)
#define UART_INT_CLR      (UartAddr+0x44)
#define UART_DMA_CTRL     (UartAddr+0x48)
#define UART_TEST_CTRL    (UartAddr+0x80)
#define UART_TEST_IN      (UartAddr+0x84)
#define UART_IEST_OUT     (UartAddr+0x88)
#define UART_TEST_DATA    (UartAddr+0x8C)
#define UART_MEM_SIZE     0xC0

// UART_FLAG register
#define UART_RX_FIFO_EMPTY (1 << 4)
#define UART_TX_FIFO_FULL  (1 << 5)

// UART Line Control Register
#define UART_LCR_BREAK          (1 << 0)
#define UART_LCR_PARITY_EN      (1 << 1)
#define UART_LCR_EVEN_PARITY    (1 << 2)
#define UART_LCR_2_STOP         (1 << 3) 
#define UART_LCR_FIFO_EN        (1 << 4)
#define UART_LCR_8_BITS         (3 << 5)
#define UART_LCR_STICK_PARITY   (1 << 7)

// UART Control Register
#define UARTCR_UART_ENABLE      (1 << 0)
#define UARTCR_LOOPBACK         (1 << 7)
#define UARTCR_TX_ENABLE        (1 << 8)
#define UARTCR_RX_ENABLE        (1 << 9)
#define UARTCR_RTS              (1 << 11)

// UART Interrupt masks
#define INT_CTS                 (1 << 1)
#define INT_RX                  (1 << 4)
#define INT_TX                  (1 << 5)
#define INT_RX_TIMEOUT          (1 << 6)
#define INT_FRAMING_ERR         (1 << 7)
#define INT_PARITY_ERR          (1 << 8)
#define INT_BREAK_ERR           (1 << 9)
#define INT_OVER_ERR            (1 << 10)