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

#include "bme280_driver.h"

unsigned short int dig_T1,dig_P1;
short int   dig_T2,dig_T3,
            dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;

int temperature;
unsigned long pressure;

dev_t dev = 200; // Majornumber
struct i2c_adapter *adapter     = NULL;  // I2C Adapter Structure
struct i2c_client  *sensor      = NULL;  // I2C Cient Structure for the Sensor

struct class *dev_class;
struct cdev sensor_cdev;

const struct file_operations fops =
{
    .owner      = THIS_MODULE,
    .read       = device_file_read,
    .write      = NULL,
    .open       = NULL,
    .release    = NULL,
};

/**
 * Struct that has the slave device id
 */
const struct i2c_device_id sensor_id[] = {
    { SLAVE_DEVICE_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);
 
/**
 * @brief struct for the i2c driver informations
 * 
 */
struct i2c_driver sensor_struct = {
    .driver = {
        .name   = SLAVE_DEVICE_NAME,
        .owner  = THIS_MODULE,
    },
    .probe          = sensor_probe,
    .remove         = sensor_remove,
    .id_table       = sensor_id,
};
 
/**
 * @brief struct fot i2c board informations
 * 
 */
static struct i2c_board_info sensor_i2c_board_info = {
    I2C_BOARD_INFO(SLAVE_DEVICE_NAME, SENSOR_SLAVE_ADDR)
};

ssize_t device_file_read(struct file *filp, char __user *buf, size_t len, loff_t *offset)
{
    int ret;
    int data[3];
    init_sensor();
    read_data();
    
    //create int array to be returned to the reading userspace proccess   
    data[0] = temperature;
    data[1] = (int) pressure;
    data[2] = (int) (pressure >> 16);
   
    ret = copy_to_user(buf, &data, 12);   
    if (ret != 0) 
        pr_alert("Sensor: Failed to write temperature!\n");
   
    return len;
}
 
void read_id(void)
{
    uint8_t res_data[2], result;
       
    result = i2c_smbus_read_i2c_block_data(sensor,0xd0, 2, res_data );
    if(result < 0)
    {
        pr_alert("Sensor: Error connecting to i2c client\n");
    }
    else
    {
        pr_info("Sensor: I2C device connected, id = 0x%x, vers = 0x%x \n", res_data[0],res_data[1]);
    }    
}

uint16_t get_ushort(uint8_t  *data_array, uint8_t index)
{
    return (uint16_t) (data_array[index + 1] << 8) + data_array[index];
}

int16_t get_short(uint8_t  *data_array, uint8_t index)
{
    return (int16_t) (data_array[index + 1] << 8) + data_array[index];
}

void init_sensor(void)
{
    uint8_t buffer[24]; // Databuffer for config register
    uint8_t control,result;
    result = i2c_smbus_write_byte_data(sensor,REG_CONTROL_HUM,OVERSAMPLE_HUM);
    if(result < 0)
    {
        pr_alert("Sensor: Error writing to the humidity setting register\n");
    }
 
    control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE;
    // set oversampling settings: Datasheet page 26
    result = i2c_smbus_write_byte_data(sensor,REG_CONTROL,control);
    if(result < 0)
    {
       pr_alert("Sensor: Error writing to the control register\n");
    } 
    
    // Read calibration data
    result = i2c_smbus_read_i2c_block_data(sensor,0x88, 24, buffer);
    if(result < 24)
    {
       pr_alert("Sensor: Error while reading 24 bytes from 0x88\n");
    } 
   
    // extract data from buffer
    dig_T1 = get_ushort(buffer,0);
    dig_T2 = get_short(buffer,2);
    dig_T3 = get_short(buffer,4);
    
    dig_P1 = get_ushort(buffer, 6);
    dig_P2 = get_short(buffer, 8);
    dig_P3 = get_short(buffer, 10);
    dig_P4 = get_short(buffer, 12);
    dig_P5 = get_short(buffer, 14);
    dig_P6 = get_short(buffer, 16);
    dig_P7 = get_short(buffer, 18);
    dig_P8 = get_short(buffer, 20);
    dig_P9 = get_short(buffer, 22);  
   
}

void read_data(void)
{    
    signed long long var1, var2,p;  
    signed long int pres_raw, temp_raw,result,t_fine, time_wait; 
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    
    time_wait =  1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575);
    msleep(time_wait);
     
    // Read temperature/pressure    
    result = i2c_smbus_read_i2c_block_data(sensor, REG_DATA,8, data);
    if(result < 0)
    {
       pr_alert("Sensor: Error while reading 8 bytes from REG_DATA\n");
    } 
         
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // Compensation of temperature: Datasheet 3.11.3
    var1 = ((((temp_raw >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11;
    var2 = (((((temp_raw >> 4) - (dig_T1)) * ((temp_raw >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;
    t_fine = var1 + var2;
    
    // temp format: "2213" equals 22.13 °C.
    temperature = (((t_fine * 5) + 128) >> 8);    
           
    
    // Compensation of pressure: Datasheet 3.11.3
  	var1 = ((s64)t_fine) - 128000;
	var2 = var1 * var1 * (s64)dig_P6;
	var2 += (var1 * (s64)dig_P5) << 17;
	var2 += ((s64)dig_P4) << 35;
	var1 = ((var1 * var1 * (s64)dig_P3) >> 8) + ((var1 * (s64)dig_P2) << 12);
	var1 = ((((s64)1) << 47) + var1) * ((s64)dig_P1) >> 33;

	if (var1 == 0)
	{
		p =  0;
    }
    else
    {   
	    p = ((((s64)1048576 - pres_raw) << 31) - var2) * 3125;
	    p = div64_s64(p, var1);
	    var1 = (((s64)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	    var2 = ((s64)(dig_P8) * p) >> 19;
	    p = ((p + var1 + var2) >> 8) + (((s64)dig_P7) << 4);
        pressure = (u32) p;
    }

	// pressure format: 24 integer bits, 8 fractional bits
    // example: "25882544" -> 25882544/256 = 101103,6875 Pa / 100 = 1011,036875 hPa   
    pr_info("Sensor: Successfully read data from Sensor \n");
    pr_info("Sensor: Temp:  %d \n", temperature);
    pr_info("Sensor: Pressure:  %ld \n", pressure);
}

int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{   
    read_id();
    init_sensor();
    read_data();    
    return 0;
}
 
int sensor_remove(struct i2c_client *client)
{   
    pr_info("Sensor: i2c sensor removed.\n");
    return 0;
}
 
int __init sensor_module_init(void)
{   
    pr_info("Sensor: -------BME280 I2C-SENSOR DRIVER-------\n");
        
    adapter     = i2c_get_adapter(I2C_BUS);        
    if( adapter != NULL )
    {
        sensor = i2c_new_device(adapter, &sensor_i2c_board_info);            
        if( sensor != NULL )
        {
            if(i2c_add_driver(&sensor_struct) < 0)
            {
                pr_alert("Sensor: Cannot add i2c device driver!\n");
                goto del_i2c_driver;
            }            
        }
        else
        {
            pr_alert("Sensor: Cannot create new i2c device!\n");
            goto unregister_i2c;
        }
        i2c_put_adapter( adapter );
    }
    else
    {
        pr_alert("Sensor: Cannot get i2c driver!\n");
        return -1;
    }
    
    // allocating major number for char device
    if( alloc_chrdev_region(&dev, 0, 1, MODULNAME) < 0 )
    {
        pr_alert("Sensor: Cannot allocate major number\n");
        goto r_unreg;
    }
    //pr_info("Sensor: Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
 
    // Creating cdev structure
    cdev_init(&sensor_cdev,&fops);
 
    // adding character device to the system
    if(cdev_add(&sensor_cdev,dev,1) < 0)
    {
        pr_alert("Sensor: Cannot add the device to the system\n");
        goto r_del;
    }
 
    // creating struct class
    dev_class = class_create(THIS_MODULE,"sensor_class");
    if(dev_class == NULL)
    {
        pr_alert("Sensor: Cannot create the struct class\n");
        goto r_class;
    }
 
    // creating the device
    if(device_create(dev_class,NULL,dev,NULL,MODULNAME) == NULL)
    {
        pr_alert("Sensor: Cannot create the Device-\n");
        goto r_device;
    }
    pr_info("Sensor: init done.\n");
    return 0;

    // error handling
    unregister_i2c:
        i2c_unregister_device(sensor);
    del_i2c_driver:    
        i2c_del_driver(&sensor_struct);
    r_device:
        device_destroy(dev_class,dev);
    r_class:
        class_destroy(dev_class);
    r_del:
        cdev_del(&sensor_cdev);
    r_unreg:
        unregister_chrdev_region(dev,1);
    return -1;   
}
 
void __exit sensor_module_exit(void)
{
    i2c_unregister_device(sensor);
    i2c_del_driver(&sensor_struct);
   
    device_destroy(dev_class,dev);
    class_destroy(dev_class);
    cdev_del(&sensor_cdev);
    unregister_chrdev_region(dev, 1);

    pr_info("Sensor: device driver removed successfully.\n");
} 

module_init(sensor_module_init);
module_exit(sensor_module_exit);