/**
 * Copyright (C) 2021 https://github.com/d3vf1x
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/uaccess.h> 
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h> 
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/time.h>

#include "mcp23s17.h"
#include "segment_display.h"

#define MODULNAME "display_driver"

// Time for each segment of pressure to be displayed
#define PRESSURE_CHANGE_TIME 3000
#define MULTIPLEX_ONTIME_MIN 1500
#define MULTIPLEX_ONTIME_MAX 2000

// Button configuration
#define BTN_GPIO 16
#define BTN_GPIO_DESC           "Button input for display driver"
#define BTN_GPIO_DEVICE_DESC    "display_driver_button"

// SPI data
#define WRITE_CMD 0
#define READ_CMD 1
#define HW_ADDR 0
const uint8_t spi_bus = 0;
const uint8_t spi_cs = 0;
const uint32_t spi_speed_hz = 50000; //50khz
const uint8_t spi_bits_per_word = 8;
const uint8_t spi_mode = 0;
const uint8_t ioconfig = BANK_OFF | INT_MIRROR_OFF | SEQOP_OFF | DISSLW_OFF | HAEN_ON | ODR_OFF | INTPOL_LOW;
struct spi_device *spi_device;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("d3vf1x");
MODULE_DESCRIPTION("Mcp23s17 4-digit 7-segment-display driver");
MODULE_VERSION("1.2");

/**
 * @brief Returns the spi control byte generated from the given r/w bit 
 * and hardware address of the mcp23s17.
 *
 * @param rw_cmd Read or write command (WRITE_CMD or READ_CMD).
 * @param hw_addr The hardware address of the mcp23s17.
 */
uint8_t get_spi_control_byte(uint8_t rw_cmd, uint8_t hw_addr);

/**
 * @brief  Returns the value read from the register specified. 
 *
 * @param rw_cmd Read or write command (WRITE_CMD or READ_CMD).
 */
uint8_t mcp23s17_read_register(uint8_t reg); 

/**
 * @brief writes the value to the given register
 * 
 * @param reg address of the register
 * @param data byte to be written
 */
void mcp23s17_write_register(uint8_t reg, uint8_t data);

/**
 * @brief initialization of the spi driver
 * 
 * @return __init status code
 */
__init int spi_init(void);

/**
 * @brief initialization of the file device driver
 * 
 * @return __init status code 
 */
__init int file_init(void);

/**
 * @brief initialization of the module
 * 
 * @return __init status code
 */
__init int device_driver_init(void);

/**
 * @brief prepares everything to remove the module 
 * 
 */
void device_driver_exit(void);

/**
 * @brief called if data is written to the device file
 * 
 * @param filp pointer to the device file 
 * @param buf buffer with the written data
 * @param len len of the data written
 * @param off offset for the buffer
 * @return ssize_t 
 */
static ssize_t  file_write(struct file *filp, const char *buf, size_t len, loff_t * off);
 
/**
 * @brief write data to the gpio expander to display the given digit at the given position
 * 
 * @param pos 
 * @param digit 
 * @param point 
 */
void set_digit(uint8_t pos, uint8_t digit, bool point);

/**
 * @brief Initialize and starts the kernel thread.
 * 
 * @return int status code 0 = ok, -1 = error
 */
int __init init_thread(void);

/**
 * @brief Function that is executed as a separate kernel thread.
 * 
 * @param arg 
 * @return int 
 */
int kthread_func(void *arg);

/**
 * @brief Returns the current time in ms.
 * 
 * @return unsigned int 
 */
unsigned int millis (void);

/**
 * @brief Interrupt handler to handle the gpio interrupt, when the button is pressed.
 * 
 * @param irq 
 * @param dev_id 
 * @param regs 
 * @return irqreturn_t 
 */
irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs);

/**
 * @brief Prepares the gpio and isr.
 * 
 * @return __init 
 */
__init int init_gpio_isr(void);

/**
 * @brief Frees the interrupt and gpio.
 * 
 */
void cleanup_gpio_isr(void);

/**
 * @brief Removes the i2c device and driver files.
 * 
 */
void display_driver_exit(void);