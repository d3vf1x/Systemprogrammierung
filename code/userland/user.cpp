/**
 * Copyright (C) 2021 https://github.com/d3vf1x
 * 
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
 * A small example userspace programm to interact with the Krenel modules for the display, the sonsor and the ws2812 strip. 
 */
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <string>

#include "user.h"

using namespace std;

#define LED_STRIP_LEN 18
#define MAX_TEMP 28
#define MIN_TEMP 10

uint8_t brightness = 50; // 0 - 100
uint8_t colour_gradient[LED_STRIP_LEN][3] = { 
                                         {0,0,255}, {15,0,240}, {30,0,225}, {45,0,210}, {60,0,195}, {75,0,180},
                                         {90,0,165},{105,0,150},{120,0,135},{135,0,120},{150,0,105},{165,0,90},
                                         {180,0,75},{210,0,60}, {225,0,45}, {240,0,30}, {255,0,15}, {255,0,0}
                                       };

struct rgb_led  { uint8_t g; uint8_t r; uint8_t b; };
struct rgb_led led_strip[LED_STRIP_LEN];
int temperature;
int pressure;

int sensor_driver = -1, display_driver = -1, uart_ws2812_driver = -1;

/**
 * @brief Handles the Sig-Interrupt called when CTRL+C is pressed.
 * 
 * @param param 
 */
void siginthandler(int param)
{
  cout << "exit.\n";
  if(uart_ws2812_driver > 0)
  {
    strip_clear();
    strip_update();
  }
  exit(0);
}

/**
 * @brief Reads data from the sensor device driver file.
 * 
 * @return int status code, 0 = ok, -1 = error
 */
int read_data_from_sensor(){
  ssize_t ret;
  char buf[12];

  ret = read(sensor_driver, &buf, 12);
  if(ret >= 0)
  {
    temperature = ((int) buf[1] << 8 | buf[0]);
    
    int p1 = (int) buf[5] << 8 | buf[4];
    int p2 = (int) buf[7] << 8 | buf[6];
    double press = ((int32_t) p2 << 16 | p1) / 256.0;     
    pressure = press + 0.5;       
  }
  else if(errno == EWOULDBLOCK)
  {
    cout << "Read would block.\n";
    return -1;
  }
  else
  {
    cerr << "Error while reading data from sensor.\n";
    return -1;
  }
  return 0;
}

/**
 * @brief Writes the given buffer to the display device driver file.
 * 
 * @param buf pointer to the buffer
 * @param n byte to be written
 * @return int status code, 0 = ok, -1 = error
 */
int send_data_to_display(const void *buf, size_t n)
{ 
  size_t ret;
  if(display_driver < 0)
  {
    cerr << "Error send_data_to_display, filedesc not initialized!\n";   
    return -1;
  } 
  
  ret = write(display_driver, buf, n);
  if(ret < 0)
  {             
    cerr << "Error while writing data to display\n";        
    return -1;
  }
  if(ret < n)
  {
    cerr << "only %d/%d bytes were written to the display driver!n"; 
    return -1;
  }
  return 0;  
}

/**
 * @brief Writes the given buffer to the uart device driver file.
 * 
 * @param buf pointer to the buffer
 * @param n byte to be written
 * @return int status code, 0 = ok, -1 = error
 */
int send_data_to_uart(const void *buf, size_t n)
{ 
  size_t ret;
  if(uart_ws2812_driver < 0)
  {
    cerr << "Error send_data_to_uart, filedesc not initialized!\n";   
    return -1;
  }
 
  ret = write(uart_ws2812_driver, buf, n);
  if(ret < 0)
  {             
    cerr << "Error while writing data to display\n";        
    return -1;
  }
  if(ret < n)
  {
    cerr << "only %d/%d bytes were written to the display driver!n"; 
    return -1;
  }
  return 0;  
}

/**
 * @brief Closes all open file descriptors.
 * 
 */
void close_fds()
{
  if(sensor_driver)
    close(sensor_driver);

  if(display_driver)
    close(display_driver);

  if(uart_ws2812_driver)
    close(uart_ws2812_driver);
}

/**
 * @brief Opens all file descriptors for every driver that is needed. Returns -1 if open failed.
 * 
 * @return int status code, 0 = ok, -1 = error
 */
int open_fds()
{
  // Open the device in non-blocking mode
  sensor_driver = open("/dev/bme280_driver", O_RDWR | O_NONBLOCK);
  if(sensor_driver < 0)
  {
    cout << "Error opening sensor device driver!";  // handle error
    return -1;
  }

  display_driver = open("/dev/display_driver", O_RDWR | O_NONBLOCK);
  if(display_driver < 0)
  {
    cout << "Error opening display device driver!";  // handle error
    return -1;
  }

  uart_ws2812_driver  = open("/dev/ws2812_uart_driver", O_RDWR | O_NONBLOCK);
  if(uart_ws2812_driver < 0)
  {
    cout << "Error opening uart device driver!";  // handle error
    return -1;
  }
  return 0;
}


/**
 * @brief Clears the LED array.
 * 
 */
void strip_clear()
{
  for(int i = 0; i < LED_STRIP_LEN; i++)
  {
    led_strip[i].r = 0;
    led_strip[i].g = 0;
    led_strip[i].b = 0;
  }
}

/**
 * @brief Writes the current led stripe array to the uart driver file.
 * 
 */
void strip_update()
{
  send_data_to_uart((uint8_t*)&led_strip,sizeof led_strip);
}

/**
 * @brief Sets the given colour values at the given led index.
 * 
 * @param nr number of the led
 * @param r red value 
 * @param g green value
 * @param b ble value
 */
void strip_set_led(uint8_t nr, uint8_t r, uint8_t g, uint8_t b)
{
  if(nr >= LED_STRIP_LEN)
    nr = LED_STRIP_LEN - 1;

  led_strip[nr].r = (brightness/100.0) * r;
  led_strip[nr].g = (brightness/100.0) * g;
  led_strip[nr].b = (brightness/100.0) * b;
}

/**
 * @brief This function will display the given temperature on the led stripe (10°C = Blue -> 28°C = red).
 * 
 * @param temp 
 */
void set_led_temp(int temp)
{
  int t = temp - MIN_TEMP;
  if (t > LED_STRIP_LEN)
    t = LED_STRIP_LEN - 1;
  if (t < 0)
    t = 0;
  
  int r = colour_gradient[t][0];
  int g = colour_gradient[t][1];
  int b = colour_gradient[t][2];
  //cout << "Gradient index: " << t << " " << r << " " << g << " " << b << "\n";
  strip_clear();
  for(int i = 0; i <= t; i++)
  {
    strip_set_led(i,r,g,b);    
    //usleep(100000);
  }
  strip_update();
}

/**
 * @brief Function which will handle the data processing. Reads temperature and pressure from the sensor and writes it to the display every 1s.
 * 
 */
void weather_station()
{
  cout << "reading data from sensor and writing data to display an led stripe, exit with CTRL+C.\n";
  int last_temp = 0;
  int temp = 0; 
  while(1){
    if(read_data_from_sensor() == 0)
    { 
      int data[2];     
      data[0] = temperature;
      data[1] = pressure;
      temp = (temperature / 100.0);
      cout << "Temperatur: " << (temperature / 100.0) << "°C, Luftdruck: " << (((double) pressure) / 100.0) << "hPa\n";

      if(temp != last_temp)
      {
        set_led_temp(temp);
        last_temp = temp;  
      }        
      send_data_to_display(&data,sizeof data);
    }
    sleep(1);
  }  
}

/**
 * @brief Writes the given number to the display driver
 * 
 */
void display_number()
{
  int data[2];  
  
  int n;
  cout << "Insert a number: ";
  cin >> n;
  data[0] = n;
  data[1] = n;
  send_data_to_display(&data,sizeof data);
}

/**
 * @brief Reads and displays the temperature and pressure from the bme280 driver.
 * 
 */
void show_sensor_data()
{
  int n;
  n = getchar();
  cout << "Press Enter to continue, any char followed by enter to exit.\n";
  do
  {
    if(read_data_from_sensor() == 0)
    {         
      cout << "Temperatur: " << (temperature / 100.0) << "°C, Luftdruck: " << (((double) pressure) / 100.0) << "hPa";
    }
    else
    {
      cerr << "Reading from Sensor failed!\n";
    }    
    n = getchar();
  }
  while(n == 10);
  cout << endl;
}

/**
 * @brief playes a animation on the led stripe until ctrl+c is pressed.
 * 
 */
void play_led_animation()
{
  cout << "playing a led animation. Exit with CTRL+C.\n";
  while(1){
    for(int i = 0; i < 18; i++)
    {
      int r = colour_gradient[i][0];
      int g = colour_gradient[i][1];
      int b = colour_gradient[i][2];
      strip_clear();
      strip_set_led(i,r,g,b);
      strip_update();
      usleep(100000);
    }
    for(int i = 17; i >= 0; i--)
    {
      int r = colour_gradient[i][0];
      int g = colour_gradient[i][1];
      int b = colour_gradient[i][2];
      strip_clear();
      strip_set_led(i,r,g,b);
      strip_update();
      usleep(100000);
    }
  }  
  strip_clear();
}

int main () 
{
  signal(SIGINT, siginthandler);
  if(open_fds() < 0)
    return -1;
 
  int n;
 
  while(1)
  {
    cout << "\x1B[2J\x1B[H";
    cout << "Welcome to this userspace control programm.\n";
    cout << "===========================================\n";
    cout << " 1: start the waether station\n";
    cout << " 2: write number to display \n";
    cout << " 3: get data from sensor \n";
    cout << " 4: play animation on neopixel \n\n";
    cout << "choose a number: ";
    cin >> n;

    switch(n)
    {
      case 1:
        weather_station();
        break;
      case 2:
        display_number();
        break;
      case 3:
        show_sensor_data();
        break;
      case 4:
        play_led_animation();
        break;
      default:
        goto exit;
    }    
  } 
  exit:
    close_fds();
    return 0;
}