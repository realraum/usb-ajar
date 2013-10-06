/*
 *  spreadspace avr utils
 *
 *
 *  Copyright (C) 2013 Christian Pointner <equinox@spreadspace.org>
 *
 *  This file is part of spreadspace avr utils.
 *
 *  spreadspace avr utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  spreadspace avr utils is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with spreadspace avr utils. If not, see <http://www.gnu.org/licenses/>.
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdio.h>

#include "util.h"
#include "led.h"
#include "anyio.h"


#define PIN_HIGH(PORT, PIN) PORT |= (1 << PIN)
#define PIN_LOW(PORT, PIN) PORT &= ~(1 << PIN)
#define PINMODE_OUTPUT PIN_HIGH  //just use DDR instead of PORT
#define PINMODE_INPUT PIN_LOW  //just use DDR instead of PORT

#define OP_SETBIT |=
#define OP_CLEARBIT &= ~
#define OP_CHECK &
#define PIN_SW(PORTDDRREG, PIN, OP) PORTDDRREG OP (1 << PIN)

#define HIGHv OP_SETBIT
#define LOWv OP_CLEARBIT

void handle_cmd(uint8_t cmd)
{
  switch(cmd) {
  case 'r': reset2bootloader(); break;
  default: printf("error\r\n"); return;
  }
  printf("ok\r\n");
}

int main(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();
  
  cpu_init();
  led_init();
  anyio_init(115200, 0);
  sei();

  PIN_HIGH(PORTB, PINB1);
  PINMODE_INPUT(DDRB,  PINB1);
  char door_ajar = 0;
  char last_door_ajar = 0;

  led_off();
  led2_on();
  
  for(;;) {
    int16_t BytesReceived = anyio_bytes_received();
    while(BytesReceived > 0) {
      int ReceivedByte = fgetc(stdin);
      if(ReceivedByte != EOF) {
        handle_cmd(ReceivedByte);
      }
      BytesReceived--;
    }
    
    door_ajar = PINB & _BV(PINB1);
    if (door_ajar != last_door_ajar) {
      last_door_ajar = door_ajar;
      if (door_ajar) {
        printf("BackdoorInfo(ajar): ajar\r\n");
        led_off();
        led2_on();
      } else {
        printf("BackdoorInfo(ajar): shut\r\n");
        led_off();
        led2_off();
      }
    }
    
    anyio_task();
    if (door_ajar) {
      _delay_ms(100);
      led_toggle();
      led2_toggle();
    } else 
    {
      _delay_ms(1200);
      led2_toggle();
    }
  }
}
