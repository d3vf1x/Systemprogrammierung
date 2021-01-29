/**
 * Copyright (C) 2021 https://github.com/d3vf1x
 * 
 * Based on work from Galileo53: https://github.com/eBUS/ttyebus
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
 * This is a Kerneldriver for the Raspberry Pi 3 v1.2, that will write data over uart0.
 * After every byte send, there will be a short delay, to let the receiver read the byte.
 * 
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/version.h>

#include "address.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("d3vf1x");
MODULE_DESCRIPTION("Kernel module for accessing uart");
MODULE_VERSION("1.1");
 
#define DEVICE_NAME         "ws2812_uart_driver"

/*********************************************/
/*                                           */
/*              File Operations              */
/*                                           */
/*********************************************/

/**
 * @brief Called when a process, which already opened the dev file, attempts to write to it. 
 *        Bytes written to the dev file will be send via uart. First byte send will indicate the len
 * 
 * @param file_ptr      Pointer to the open file
 * @param user_buffer   Buffer in user space where to receive the data
 * @param Count         Number of bytes to write
 * @param offset        Pointer to a counter that can hold an offset when writing chunks
 * @return ssize_t 
 */
ssize_t file_write(struct file* file_ptr, const char __user* user_buffer, size_t Count, loff_t* offset);
    
/**
 * @brief Called when a process tries to open the device file. Enables uart and prepares everything for the uart transmission.
 * 
 * @param inode
 * @param file
 * @return int
 */
int file_open(struct inode* inode, struct file* file);

/**
 * @brief Called when a process closes the device file. Disables uart.
 * 
 * @param inode 
 * @param file 
 * @return int 
 */
int file_close(struct inode *inode, struct file *file);


/*********************************************/
/*                                           */
/*              GPIO Operations              */
/*                                           */
/*********************************************/

/**
 * @brief Configure the given gpio with the given gpio function.
 * 
 * @param Gpio number
 * @param Function gpio function code
 */
void set_gpio_mode(unsigned int Gpio, unsigned int Function);

/**
 * @brief Configure the gpio pullupdown state
 * 
 * @param Gpio 
 * @param pud 
 */
void set_gpio_pullupdown(unsigned int Gpio, unsigned int pud);


/*********************************************/
/*                                           */
/*             Module functions              */
/*                                           */
/*********************************************/

/**
 * @brief Register the driver to the kernel and initialize everything.
 * 
 * @return int Major Number of the driver
 */
int module_register(void);
        
/**
 * @brief Unmap the I/O and unregister the device.
 * 
 */
void module_unregister(void);

// inline asm delay function
inline void delay(int32_t count);
