/**
 *
 * Driver for an ATTINY2313A and WS2812. 
 * It will wait until a byte is received over UART. The first byte will be the number of bytes to be received (excl. the size byte).
 * Than it will read all incomming bytes until the given size is reached. When all are received, they will be send out on PD6.
 * 
 */
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>

// UART config
#define USART_BAUDRATE 115200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define MAX_LEDS 20 // currently max 85

struct rgb_led  { uint8_t g; uint8_t r; uint8_t b; };
struct rgb_led led[MAX_LEDS];

uint8_t *led_data_base;
uint16_t led_datalen;
volatile uint8_t status = 1;

/**
 * Prepares the avr to receive data via uart. 
 * UART-mode: 2 Stop bits, 8 bit Frames, even Parity bit
 * 
 */
void uart_init()
{
  UBRRH = (uint8_t)(BAUD_PRESCALE >> 8); // set baud-rate
  UBRRL = (uint8_t) BAUD_PRESCALE; 
  UCSRB = (1 << RXEN); // enablve receiver
  UCSRC = (1<<USBS) | (3<<UCSZ0) | (1<<UPM1); //set the mode: 2 Stop bits + 8bit Frame-size + even Parity mode
}

/**
 * Sends all bytes from the byte-array through pin PD6 to the WS2812 strip.
 */
void inline ws2812_write(uint8_t *data,uint16_t dat_len)
{
  uint8_t current_byte, counter, low_mask, high_mask, sreg_prev;

  // INIT everything for pin PD6 as output  
  DDRD |= (1<<6); // set PD6 as Output    
  high_mask = (1<<6); 
  low_mask	= ~high_mask & PORTD;
  high_mask |=        PORTD; // save the current state of the PORTD

  sreg_prev = SREG;  // save status bits 

  // loop through the buffer
  while (dat_len--) {
    current_byte = *data++;
    
    // inline assembly code to be as fast as possible    
    asm volatile(
      "       ldi   %0,8  " "\n\t"  // init counter
      "       clt         " "\n\t"  // clear interrupts -> no interruption

      "loop%=:            " "\n\t"   // loop through all bits in the current byte
      "       out   %2,%3"  "\n\t"   // set pin high 
     
      "       nop"          "\n\t"   // no operation
     
      "       sbrs  %1,7 "  "\n\t"    
      "       out   %2,%4"  "\n\t"   // set pin low
  
      "       lsl   %1   "  "\n\t"    
      "       nop"          "\n\t"
      "       out   %2,%4"  "\n\t"   // set pin low
    
      "       nop" "\n\t"
    
      "       dec   %0    "   "\n\t" // decrement counter
      "       brne  loop%="   "\n\t"    
        :	"=&d" (counter)   :	"r" (current_byte), "I" (_SFR_IO_ADDR(PORTD)), "r" (high_mask), "r" (low_mask) // use vars from cpp in asm
    );
  }
  
  SREG = sreg_prev; // reset status bits
}

/**
 * Waits for a byte to be received over UART, writes them to led_data_base (memory musst my allocated prior to call this fkt). 
 * led_datalen will hold the number of received bytes.
 */
void read_data(void)
{  
  led_data_base = (uint8_t*)&led;
  uint8_t* current_pos = led_data_base;
  uint8_t len = 0;

  // wait for first byte = total length
  while ( !(UCSRA & (1<<RXC)));
    led_datalen = UDR;
  if(led_datalen > (MAX_LEDS * 3))
    led_datalen = (MAX_LEDS * 3);

  // receive all data
  while(len++ < led_datalen)
  {
    while ( !(UCSRA & (1<<RXC)));   // wait until next byte is in buffer
      *current_pos++ = UDR;
  }

}

int main(void) {
    uart_init();

    // prepare the led data buffer
    uint8_t* p = led_data_base;
    for(int i = 0; i < (MAX_LEDS * 3); i++){
      *p++ = 0;
    }

    // receive and send data
    while(1){       
      read_data();        
      ws2812_write(led_data_base,led_datalen);    
    }
                                    
  return 0;
}
