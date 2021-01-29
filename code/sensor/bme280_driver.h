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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h> 

#define MODULNAME "bme280_driver" // name of the device file

// I2C config
#define I2C_BUS             1     
#define SLAVE_DEVICE_NAME   "BME280"         
#define SENSOR_SLAVE_ADDR   0x76  

// Register Address
#define REG_DATA            0xF7           
#define REG_CONTROL         0xF4
#define REG_CONFIG          0xF5

// Sensor Settings
#define REG_CONTROL_HUM     0xF2
#define REG_HUM_MSB         0xFD
#define REG_HUM_LSB         0xFE

// Oversample setting - datasheet page 27
#define OVERSAMPLE_HUM      2
#define OVERSAMPLE_TEMP     2
#define OVERSAMPLE_PRES     2
#define MODE                1

MODULE_LICENSE("GPL");
MODULE_AUTHOR("d3vf1x");
MODULE_DESCRIPTION("BME280 Sensor driver");
MODULE_VERSION("1.0");


/**
 * @brief called when the device file is read
 * 
 * @param filp pointer to the device file 
 * @param buf 
 * @param len 
 * @param off 
 * @return ssize_t 
 */
ssize_t device_file_read(struct file *filp, char __user *buf, size_t len,loff_t * off);

/**
 * @brief Reads 2 bytes from the given array, beginning at the given index and interprets it as an 16bit unsigned short
 * 
 * @param data_array from where to read the 2 byte 
 * @param index where to start
 * @return int16_t 
 */
uint16_t get_ushort(uint8_t *data_array, uint8_t index);

/**
 * @brief Reads 2 bytes from the given array, beginning at the given index and interprets it as an 16bit short
 * 
 * @param data_array from where to read the 2 byte 
 * @param index where to start
 * @return int16_t 
 */
int16_t get_short(uint8_t *data_array, uint8_t index);

/**
 * @brief reads the current temperature and pressure from the sensor
 * 
 */
void read_data(void); 

/**
 * @brief prepares the sensor for reading the temp and pressure
 * 
 */
void init_sensor(void);  

/**
 * @brief callend when the i2c-device is loaded/probed
 * 
 * @param client i2c_client struct
 * @param id i2c_device_id id
 * @return int status code
 */
int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id);

/**
 * @brief called when the i2c-device is removed 
 * 
 * @param client i2c_client struct
 * @return int status code
 */
int sensor_remove(struct i2c_client *client);

/**
 * @brief called when module is loaded
 * 
 * @return int status code
 */
int __init sensor_module_init(void);

/**
 * @brief called when module is unloaded
 * 
 */
void __exit sensor_module_exit(void);