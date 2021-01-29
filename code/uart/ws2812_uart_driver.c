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

#include "address.h"
#include "ws2812_uart_driver.h"


// file operations with this kernel module
struct file_operations driver_fops =
{
    .owner          = THIS_MODULE,
    .open           = file_open,
    .release        = file_close,
    .poll           = NULL,
    .read           = NULL,
    .write          = file_write,
    .unlocked_ioctl = NULL
};

// set file access mode
struct miscdevice misc =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &driver_fops,
    .mode = S_IRUSR |   // User read
            S_IWUSR |   // User write
            S_IRGRP |   // Group read
            S_IWGRP |   // Group write
            S_IROTH |   // Other read
            S_IWOTH     // Other write
};

unsigned int MajorNumber;
void* GpioAddr;
void* UartAddr;
unsigned int DeviceOpen;
spinlock_t SpinLock;

// linear buffer used for transmitting data
enum { TX_BUFF_SIZE = 255 };
volatile unsigned int TxTail = TX_BUFF_SIZE;
volatile unsigned int TxHead = TX_BUFF_SIZE;
unsigned char TxBuff[TX_BUFF_SIZE];

/*********************************************/
/*                                           */
/*              File Operations              */
/*                                           */
/*********************************************/
ssize_t file_write(struct file* file_ptr, const char __user* user_buffer, size_t Count, loff_t* offset)
{
    int result;
    int Timeout;
    unsigned long Flags;
    unsigned int DataWord;
    unsigned char len;

    //printk(KERN_NOTICE "Uart: Write request with offset=%d and count=%u", (int)*offset, (unsigned int)Count);

    // if transmission is still in progress, wait until done
    Timeout = 500;
    while (TxTail < TxHead)
        {
        if (--Timeout < 0)
            return -EBUSY;
            udelay(500);
        }

    // copying data from user space requires a special function to be called
    if (Count > TX_BUFF_SIZE)
        Count = TX_BUFF_SIZE;
    result = copy_from_user(TxBuff, user_buffer, Count);
    if (result > 0)             // not all requested bytes copied
        Count = result;         // nuber of bytes copied
    else if (result != 0)
        return -EFAULT;
    len = (unsigned char) Count;
    

    // data beginning with one byte with data len
    spin_lock_irqsave(&SpinLock, Flags); //save interrupts

    DataWord = TxBuff[0];
    TxTail = 1;
    TxHead = len;

    iowrite32(len, UART_DATA);   
  
    //printk(KERN_NOTICE "Uart: Transmitting package len byte. (0x%x)\n", len);   
  

    udelay(200);

    iowrite32(DataWord, UART_DATA);
   
    //printk(KERN_NOTICE "Uart: Transmitting one byte. (0x%x)TxHead=%d, TxTail=%d\n", DataWord, TxHead, TxTail);
   

    // send every byte from the buffer 
    while (TxTail < TxHead)
    {             
        udelay(200); 
        DataWord = TxBuff[TxTail++];        
        iowrite32(DataWord, UART_DATA);
       
        //printk(KERN_NOTICE "Uart: Transmitting one byte. (0x%x)TxHead=%d, TxTail=%d", DataWord, TxHead, TxTail);        
    }

    spin_unlock_irqrestore(&SpinLock, Flags);    

    printk(KERN_NOTICE "Uart: Write exit with %d bytes written", Count);

    return Count;        // the number of bytes actually transmitted
}
   
int file_open(struct inode* inode, struct file* file)
{
    unsigned int UartCtrl;
    unsigned int IntStatus;


    //printk(KERN_NOTICE "Uart: Open at at major %d  minor %d", imajor(inode), iminor(inode));


    // do not allow another open if already open  
	if (DeviceOpen)
		return -EBUSY;
	DeviceOpen++;

	// Disable UART0 
	iowrite32(0, UART_CTRL);

   	// Setup the GPIO pin 14 && 15 to ALTERNATE 0 (connect to UART)
    set_gpio_mode(14, GPIO_ALT_0);      // GPIO14 connected to TxD

	// Set pull-down for the GPIO pin
    set_gpio_pullupdown(14, GPIO_PULL_UP);

	// Clear pending interrupts
	iowrite32(0x7FF, UART_INT_CLR);

    // setting BAUDRATE to 115200
   	iowrite32(3000000 / 115200, UART_INT_BAUD); 
	iowrite32(0, UART_FRAC_BAUD);
 
	// Disable FIFO, Framesize 8 bit, 2 stop bits, even parity
	iowrite32(UART_LCR_2_STOP | UART_LCR_8_BITS | UART_LCR_PARITY_EN | UART_LCR_EVEN_PARITY, UART_LINE_CTRL);
    
    // Read data register to clear overflow error bit. In addition, clear any other receiver error   
    do{
        ioread32(UART_DATA);
        IntStatus = ioread32(UART_INT_STAT);
    }while(IntStatus & INT_RX);
            

	// Enable UART0, transfer part of UART  
	UartCtrl = UARTCR_UART_ENABLE | UARTCR_TX_ENABLE | UARTCR_RTS;  
	iowrite32(UartCtrl, UART_CTRL);

    //printk(KERN_NOTICE "Uart: Open exit");

	return 0;
}

int file_close(struct inode *inode, struct file *file)
{
    //printk(KERN_NOTICE "Uart: Close at at major %d  minor %d", imajor(inode), iminor(inode));

	DeviceOpen--;

	// Disable UART0
    // =============
	iowrite32(0, UART_CTRL);

    //printk(KERN_NOTICE "Uart: Close exit");

	return 0;
}

/*********************************************/
/*                                           */
/*              GPIO Operations              */
/*                                           */
/*********************************************/
void set_gpio_mode(unsigned int Gpio, unsigned int Function)
{
    unsigned int RegOffset = (Gpio / 10) << 2;
    unsigned int Bit = (Gpio % 10) * 3;
    volatile unsigned int Value = ioread32(GpioAddr + RegOffset);
    iowrite32((Value & ~(0x7 << Bit)) | ((Function & 0x7) << Bit), GpioAddr + RegOffset);
}

void set_gpio_pullupdown(unsigned int Gpio, unsigned int pud)
{
    // fill the new value for pull up or down
    // ======================================
    iowrite32(pud, GPIO_PULL);
    delay(150);     // provide hold time for the control signal

    // transfer the new value to the GPIO pin
    // ======================================
    iowrite32(GPIO_BIT, GPIO_PULLCLK0 + GPIO_BANK);
    delay(150);     // provide hold time for the control signal

    // remove the control signal to make it happen
    // ===========================================
    iowrite32(0, GPIO_PULL);
    iowrite32(0, GPIO_PULLCLK0 + GPIO_BANK);
}
 
/*********************************************/
/*                                           */
/*             Module functions              */
/*                                           */
/*********************************************/
int module_register(void)
{
    int result;

    //printk(KERN_NOTICE "Uart: register_device() is called");

    // Dynamically allocate a major number for the device
    MajorNumber = register_chrdev(0, DEVICE_NAME, &driver_fops);
    if (MajorNumber < 0)
    {
        printk(KERN_WARNING "Uart: can\'t register character device with errorcode = %i", MajorNumber);
        return MajorNumber;
    }

    //printk(KERN_NOTICE "Uart: registered character device with major number = %i and minor numbers 0...255", MajorNumber);


    // Register the device driver. We are using misc_register instead of
    // device_create so we are able to set the attributes to rw for everybody
    result = misc_register(&misc);
    if (result)
        {
        unregister_chrdev(MajorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Uart: Failed to create the device");
        return result;
        }

    // remap the I/O registers
    GpioAddr = ioremap(RASPI_23_PERI_BASE + GPIO_BASE, SZ_4K);
    UartAddr = ioremap(RASPI_23_PERI_BASE + UART0_BASE, SZ_4K);

    // initialize the spinlock
    spin_lock_init(&SpinLock);
  
    DeviceOpen = 0;

    printk(KERN_INFO "Uart: device driver created correctly.");

    return result;
}
    
void module_unregister(void)
{
    // release the mapping
    if (GpioAddr)
        iounmap(GpioAddr);
    if (UartAddr)
        iounmap(UartAddr);

    GpioAddr = 0;
    UartAddr = 0;

    misc_deregister(&misc);
    unregister_chrdev(MajorNumber, DEVICE_NAME);

    pr_info("Uart: device driver removed successfully.\n");
    MajorNumber = 0;
}

// inline asm delay function
inline void delay(int32_t count)
{
	asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
	    : "=r"(count): [count]"0"(count) : "cc");
}

module_init(module_register);
module_exit(module_unregister);