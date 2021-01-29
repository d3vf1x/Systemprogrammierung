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
 * 
 * This is the kernel driver for 4 digit 7-segment display driven via a mcp23s17.
 * 
 */

#include "display_driver.h"

/*
 * holds the content for the display 
 */
uint8_t temp[4];
uint8_t press[7];
uint8_t time[4];
volatile uint8_t content_nr = 0; //time, temp, pressure

struct task_struct *task;

/*
 *  file operations 
 */
dev_t dev = 250; // Majornumber
static struct class *dev_class;
static struct cdev display_cdev;
static struct file_operations fops =
{
    .owner      = THIS_MODULE,
    .read       = NULL,
    .write      = file_write,
    .open       = NULL,
    .release    = NULL,
};

/**
 * @brief called from module init functions
 * 
 * @return __init status code, 0 = ok, -1 = error
 */
__init int file_init(void)
{
    pr_info("Display: -------DISPLAY DEVICE DRIVER-------\n");
    // allocating a Major number for the modul
    if( (alloc_chrdev_region(&dev, 0, 1, MODULNAME)) < 0 )
    {
        pr_alert("Display: Cannot allocate major number!\n");
        goto r_unreg;
    }
    //pr_info("Display: Node Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
 
    // creating cdev structure 
    cdev_init(&display_cdev,&fops);
 
    // adding character device to the system
    if( (cdev_add(&display_cdev,dev,1)) < 0 )
    {
        pr_alert("Display: Cannot add the device to the system!\n" );
        goto r_del;
    }
 
    // creating struct class
    dev_class = class_create(THIS_MODULE,"diplay_driver_class");
    if( dev_class == NULL )
    {
        pr_alert("Display: Cannot create the struct class!\n");
        goto r_class;
    }
 
    // creating device
    if( (device_create(dev_class,NULL,dev,NULL,"display_device")) == NULL )
    {
        pr_alert("Display: Cannot create the device!\n");
        goto r_device;
    }

    pr_info("Display: Display device driver insert...done\n");
    return 0;
    
    //cleanup if error occurs
    r_device:
        class_destroy(dev_class);
    r_class:
        unregister_chrdev_region(dev,1);
    r_del:
        cdev_del(&display_cdev);
    r_unreg:
        unregister_chrdev_region(dev,1);
    return -1;

}

/**
 * @brief   Called if data is written to the device driver file. 
 *          To display temp and press write 4 byte (int) temp followed by 4 byte (int) pressure
 * 
 * @param filp file pointer
 * @param buf userspace buffer
 * @param len len
 * @param off offset
 * @return ssize_t len of data read
 */
static ssize_t file_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    uint32_t ret;
    int data[2]; //data buffer
    int t; // temperature
    int p; //pressure

    if ( len < 8 )
    {
        pr_alert("Display: Less than 8 byte was written to the display device file!\n");
        return -1;
    }
    
    ret = copy_from_user(data, buf, 8);   
    if (ret != 0) {
        pr_info("Display: Failed to write temperature!\n");
        return len;
    }
          
    // determine digits for temperature
    t = data[0];
    //pr_info("Display: Data read: temp: %d\n",t );
    temp[0] = t%10;
    t = t/10;
    temp[1] = t%10;
    t = t/10;
    temp[2] = t%10;
    t = t/10;
    temp[3] = t%10;
    pr_info("Display: Temp digits: %d %d %d %d\n", (int) temp[3], (int) temp[2],(int) temp[1],(int) temp[0]);  

    // determine digits for pressure
    p = data[1];
    //pr_info("Display: Data read: pressure: %u\n",p); //%lu
    press[0] = p%10;  
    p = p/10;     
    press[1] = p%10;
    p = p/10;
    press[2] = p%10;
    p = p/10;
    press[3] = p%10;
    p = p/10;
    press[4] = p%10;
    p = p/10;
    press[5] = p%10;   
    pr_info("Display: Pressure digits: %d %d %d %d %d %d\n",(int) press[5], (int) press[4], (int) press[3], (int) press[2],(int) press[1],(int) press[0]);    
    return len;  
}


/************************************/
/*          SPI functions           */
/************************************/

/**
 * @brief   Called from the module init functions. 
 *          Prepares everything for spi connection with mcp23s17. 
 *          Sets all required GPIOS for mcp23s17, init pins with 0.
 * 
 * @return __init status code, 0 = ok, -1 = err
 */
__init int spi_init(void) 
{
    struct spi_board_info spi_device_info = {
        .modalias = "7-segment display",
        .max_speed_hz = spi_speed_hz,
        .bus_num = spi_bus,
        .chip_select = spi_cs,
        .mode = spi_mode,
    };
    struct spi_master *master;
    int ret;

    // get the master device, given SPI the bus number
    master = spi_busnum_to_master( spi_device_info.bus_num );
    if( !master )
    {
        pr_alert("Display: Cannot get the master device!\n");
        return -1;
    }

    // create a new slave device, given the master and device info
    spi_device = spi_new_device( master, &spi_device_info );
    if( !spi_device )
    {
        pr_alert("Display: Cannot create spi slave device!");
        return -1;
    }

    spi_device->bits_per_word = spi_bits_per_word;

    ret = spi_setup( spi_device );
    if( ret )
    {       
        pr_alert("Display: SPI setup failed!\n");
        goto remove_spi_device;
    }

    /**
     * init the gpio expander 
     */
    mcp23s17_write_register(IOCON,ioconfig);
    mcp23s17_write_register(IODIRA,0x00);
    mcp23s17_write_register(IODIRB,0x00);
    mcp23s17_write_register(GPIOA,0x0);
    mcp23s17_write_register(GPIOB,0x0);
    
    pr_info("Display: SPI init done.");
    return 0;

     //cleanup if error occurs
    remove_spi_device:
        spi_unregister_device( spi_device );
        return -1;
}

/**
 * @brief creates the spi control byte
 * 
 * @param rw_cmd 
 * @param hw_addr 
 * @return uint8_t 
 */
uint8_t get_spi_control_byte(uint8_t rw_cmd, uint8_t hw_addr)
{
    hw_addr = (hw_addr << 1) & 0xe;
    rw_cmd &= 1;
    return 0x40 | hw_addr | rw_cmd;
}

/**
 * @brief Reads byte from the given register address from the mcp23s17.
 * 
 * @param reg byte address
 * @return uint8_t value of register
 */
uint8_t mcp23s17_read_register(uint8_t reg)
{
    uint8_t control_byte = get_spi_control_byte(READ_CMD, HW_ADDR);
    uint8_t tx_buf[3] = {control_byte, reg, 0};
    uint8_t rx_buf[sizeof tx_buf];
    
    if( spi_write_then_read(spi_device, &tx_buf, sizeof tx_buf, &rx_buf, sizeof rx_buf) < 0 )
    {
        pr_alert("Display: Error while reading from mcp23s17 register: %x \n",reg);
        return 0;
    }
	return rx_buf[0];   
}

/**
 * @brief Writes the given data byte to the given address.
 * 
 * @param reg 
 * @param data 
 */
void mcp23s17_write_register(uint8_t reg, uint8_t data)
{
    uint8_t control_byte = get_spi_control_byte(WRITE_CMD, HW_ADDR);
    uint8_t tx_buf[3] = {control_byte, reg, data};
    if( spi_write(spi_device, &tx_buf, sizeof tx_buf) < 0 )
    {
        pr_alert("Display: Error while reading from mcp23s17 register: %x \n",reg);
    }
}

/**
 * @brief Sets a given digit value to a given display position.
 * 
 * @param pos from 0 - 3
 * @param digit value from 0 - 9
 * @param point enables the decimal point at the given position
 */
void set_digit(uint8_t pos, uint8_t digit, bool point)
{
    //pr_info("Display: Set digit: %d %d\n", pos,digit);

    if(pos == 0){
        pos = DIGIT_POS_0;
    }else if(pos == 1){
        pos = DIGIT_POS_1;
    }else if(pos == 2){
        pos = DIGIT_POS_2;
    }else if(pos == 3){    
        pos = DIGIT_POS_3;
    }else{
        pr_info("Display: Value pos not found: %d\n", pos);
        return;
    }        

    if(digit == 0){
        digit = DIGIT_0;
    }else if(digit == 1){    
        digit = DIGIT_1;
    }else if(digit == 2){   
        digit = DIGIT_2;
    }else if(digit == 3){    
        digit = DIGIT_3;
    }else if(digit == 4){   
        digit = DIGIT_4;
    }else if(digit == 5){    
        digit = DIGIT_5;
    }else if(digit == 6){   
        digit = DIGIT_6;
    }else if(digit == 7){   
        digit = DIGIT_7;
    }else if(digit == 8){   
        digit = DIGIT_8;
    }else if(digit == 9){   
        digit = DIGIT_9;
    }else{
        pr_info("Display: Value digit not found: %d\n", digit);
        return;
    }
        
    if( point )
    {
        digit |= DECIMAL_POINT; 
    }
    mcp23s17_write_register(GPIOB,0);  
    mcp23s17_write_register(GPIOA,pos);
    mcp23s17_write_register(GPIOB,digit);
}

/************************************/
/*       THREAD functions           */
/************************************/

/**
 * @brief function that will run inside the seperate thread.
 * 
 * @param arg 
 * @return int 
 */
int kthread_func(void *arg)
{    
    unsigned int start_time;
    unsigned int time_now;
    
    pr_info("Display: Kernel Thread display multiplexing started: [PID = %d]\n", current->pid);
    start_time = millis();
    content_nr = 0;
    pr_info("Display: Time %d%d:%d%d\n",time[0],time[1],time[2],time[3]);
    while(!kthread_should_stop())
    {
        schedule();  
        time_now = millis();     
        //pr_info("Display: temperature %d°C\n", temperature);
        if(content_nr == 0) // time
        {            
            //pr_alert("%d:%d\n",time[0],ti);
            set_digit(0, time[3], false);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
            set_digit(1, time[2], false);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
            set_digit(2, time[1], true);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);          
            set_digit(3, time[0], false);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX); 
        }
        else if(content_nr == 1) // temperature
        {
            set_digit(0, temp[0], false);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
            set_digit(1, temp[1], false);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
            set_digit(2, temp[2], true);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);          
            set_digit(3, temp[3], false);
            usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX); 
        }
        else
        { // pressure
            if(time_now - start_time < PRESSURE_CHANGE_TIME)
            {
                set_digit(0, press[0], false);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
                set_digit(1, press[1], false);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
                set_digit(2, press[2], true);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);          
                set_digit(3, press[3], false);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);               
            }
            else if(time_now - start_time  < PRESSURE_CHANGE_TIME * 2)
            {
                set_digit(0, press[2], true);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
                set_digit(1, press[3], false);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);        
                set_digit(2, press[4], false);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX);          
                set_digit(3, press[5], false);
                usleep_range(MULTIPLEX_ONTIME_MIN, MULTIPLEX_ONTIME_MAX); 
            }
            else
            {
                start_time = time_now;
            }            
        }                   
    }
    return 0;
}

/**
 * @brief starts the kthread function as new thread.
 * 
 * @return int status code, 0 = ok, 1 = err
 */
int __init init_thread(void)
{
    int err;
    //pr_info("Display: Starting thread\n");
   
    task = kthread_run(kthread_func, NULL, "thread");
    if (IS_ERR(task)) 
    {
        pr_info("Display: ERROR: Cannot create thread\n");
        err = PTR_ERR(task);
        task = NULL;
        return err;
    }
   
    return 0;
}


/************************************/
/*    Interrupt/GPIO functions      */
/************************************/
short int irq_any_gpio    = 0;
unsigned int last_interrupt_time = 0;
static uint64_t epochMilli;

/**
 * @brief get current time in milli seconds
 * 
 * @return unsigned int 
 */
unsigned int millis (void)
{
    struct timespec ts ;
    uint64_t now ;
    int min, hour;  

    getnstimeofday(&ts) ;
    now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000) ;
    hour = (ts.tv_sec / 3600 + 1) % (24);
    min = (ts.tv_sec / 60) % (60);
    time[0] = (hour / 10);
    time[1] = (hour % 10); 
    time[2] = (min / 10);
    time[3] = (min % 10);                   
    return (uint32_t)(now - epochMilli) ;
}

/**
 * @brief interrupt handler
 * 
 * @param irq 
 * @param dev_id 
 * @param regs 
 * @return irqreturn_t 
 */
irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {
    unsigned long flags;
    unsigned int interrupt_time = millis();

    // debounce
    if (interrupt_time - last_interrupt_time < 500) 
    {
        //printk(KERN_NOTICE "Ignored Interrupt!!!!! [%d]%s \n", irq, (char *) dev_id);
        return IRQ_HANDLED;
    }
    last_interrupt_time = interrupt_time;

    // disable hard interrupts (remember them in flag 'flags')
    local_irq_save(flags);

    //printk(KERN_NOTICE "Interrupt [%d] for device %s was triggered !.\n", irq, (char *) dev_id);

    // restore hard interrupts
    local_irq_restore(flags);

    // toggle display
    content_nr = ++content_nr % 3;
    pr_info("content: %d", content_nr);
    return IRQ_HANDLED;
}

/**
 * @brief Init everything for the button interrupt.
 * 
 * @return __init status code, 0 = ok, -1 = err
 */
__init int init_gpio_isr(void) 
{
    // init the reference for millis functions
    struct timespec ts ;
    getnstimeofday(&ts) ;
    epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    + (uint64_t)(ts.tv_nsec / 1000000) ;
    
    if (gpio_request(BTN_GPIO, BTN_GPIO_DESC))
    {
        pr_alert("Display: GPIO request failure %s\n", BTN_GPIO_DESC);
        return -1;
    }

    irq_any_gpio = gpio_to_irq(BTN_GPIO);
    if ( irq_any_gpio < 0 )
    {
        pr_alert("Display: GPIO to IRQ mapping failure %s\n", BTN_GPIO_DESC);
        return -1;
    }

    //printk(KERN_NOTICE "Mapped int %d\n", irq_any_gpio);

    if (request_irq(irq_any_gpio, (irq_handler_t ) r_irq_handler, IRQF_TRIGGER_FALLING, BTN_GPIO_DESC, BTN_GPIO_DEVICE_DESC)) {
        pr_alert("Display: Irq Request failure!\n");
        return -1;
    }

    return 0;
}

/**
 * @brief cleanup and release the gpio and isr.
 * 
 */
void cleanup_gpio_isr(void)
{
    free_irq(irq_any_gpio, BTN_GPIO_DEVICE_DESC);
    gpio_free(BTN_GPIO);
}


/**
 * @brief modul init function. Called when modul is loaded.
 * 
 * @return __init 
 */
__init int display_driver_init(void)
{
    pr_info("Display: Loading module for display driver\n");

    if(file_init() < 0)  
        goto r_cleanup; 

    if(spi_init() < 0)  
        goto r_cleanup;  

    if(init_thread() < 0)  
        goto r_cleanup;    

    if(init_gpio_isr() < 0)  
        goto r_cleanup;  
    
    return 0;
   
    r_cleanup:
        display_driver_exit();        
        return -1;   
}

/**
 * @brief Called when display is unloaded.
 * 
 */
void display_driver_exit(void)
{
    kthread_stop(task);    

    // reset Display
    mcp23s17_write_register(GPIOA,0x0);
    mcp23s17_write_register(GPIOB,0x0);
   
    spi_unregister_device( spi_device );
    device_destroy( dev_class,dev );
    class_destroy( dev_class );
    cdev_del( &display_cdev );
    unregister_chrdev_region(dev, 1);

    cleanup_gpio_isr();
    pr_info("Display: device driver removed successfully.\n");   
}

module_init(display_driver_init);
module_exit(display_driver_exit);
