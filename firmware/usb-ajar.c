/*
 *  usb-ajar
 *
 *
 *  Copyright (C) 2013-2014 Christian Pointner <equinox@spreadspace.org>
 *
 *  This file is part of usb-ajar.
 *
 *  usb-ajar is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  usb-ajar is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with usb-ajar. If not, see <http://www.gnu.org/licenses/>.
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

#include "onewire.h"
#include "ds1820.h"

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

uint8_t num_temp_sensors_ = 0;
uint8_t num_temp_sensors_mirror_ = 0;

void complementUint8(uint8_t *ptr1, uint8_t *complement)
{
  *complement = 0xff ^ *ptr1;
}

void complementCheckUint8(uint8_t *ptr1, uint8_t *complement)
{
  if (( *ptr1 ^ *complement ^ 0xff) > 0 )
  {
      reset2bootloader();
  }
}

void tempToUSB(uint8_t bit_resolution)
{
    uint8_t sensor_index = 0;
    uint16_t raw_temp = 0;

    complementCheckUint8(&num_temp_sensors_, &num_temp_sensors_mirror_);
    if (num_temp_sensors_ == 0)
    {
        printf("No DS1820 sensors ?? running bus discovery... \r\n");
        num_temp_sensors_ = ds1820_discover();
        complementUint8(&num_temp_sensors_, &num_temp_sensors_mirror_);
    }

    for (sensor_index=0; sensor_index < num_temp_sensors_; sensor_index++)
    {
        ds1820_set_resolution(sensor_index, bit_resolution);
        ds1820_start_measuring(sensor_index);
    }

    ds1820_wait_conversion_time(bit_resolution);

    for (sensor_index=0; sensor_index < num_temp_sensors_; sensor_index++)
    {
        raw_temp = ds1820_read_temperature(sensor_index);
        if (raw_temp == DS1820_ERROR)
        {
            printf("CRC comm error\r\n");
        } else {
            printf("temp%d: %d.%d\r\n", sensor_index +1, raw_temp / 16, 10 * (raw_temp % 16) / 16);
        }
    }
}

void handle_cmd(uint8_t cmd)
{
  switch(cmd) {
  case 'r': reset2bootloader(); break;
  case '1': num_temp_sensors_ = ds1820_discover(); complementUint8(&num_temp_sensors_, &num_temp_sensors_mirror_); break;
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
  PIN_HIGH(PORTB, PINB0);
  PINMODE_INPUT(DDRB,  PINB0);
  PIN_LOW(PORTD, PIND7);
  PINMODE_OUTPUT(DDRD,  PIND7);

  owi_init(PINC4, &PINC);

  uint8_t door_ajar = 0;
  uint8_t last_door_ajar = 0;
  uint8_t last_door_ajar_mirror = 0;
  uint8_t gas_leak = 0;
  uint8_t last_gas_leak = 0;
  uint8_t last_gas_leak_mirror = 0;

  complementUint8(&last_door_ajar, &last_door_ajar_mirror);
  complementUint8(&last_gas_leak, &last_gas_leak_mirror);

  num_temp_sensors_ = ds1820_discover();
  complementUint8(&num_temp_sensors_, &num_temp_sensors_mirror_);
  uint16_t ms_elapsed = 0;

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

    complementCheckUint8(&last_door_ajar, &last_door_ajar_mirror);
    if (door_ajar != last_door_ajar) {
      last_door_ajar = door_ajar;
      complementUint8(&last_door_ajar, &last_door_ajar_mirror);
      if (door_ajar == _BV(PINB1)) {
        printf("BackdoorInfo(ajar): ajar\r\n");
        led_off();
        led2_on();
      } else if ((door_ajar & _BV(PINB1)) == 0) {
        printf("BackdoorInfo(ajar): shut\r\n");
        led_off();
        led2_off();
      }
    }
    gas_leak = ! (PINB & _BV(PINB0));
    complementCheckUint8(&last_gas_leak, &last_gas_leak_mirror);
    if (gas_leak != last_gas_leak)
    {
      if (gas_leak)
        printf("GasLeakAlert\r\n");
      last_gas_leak = gas_leak;
      complementUint8(&last_gas_leak, &last_gas_leak_mirror);
    }

    anyio_task();
    if (door_ajar) {
      _delay_ms(100);
      ms_elapsed += 100;
      led_toggle();
      led2_toggle();
    } else
    {
      _delay_ms(1200);
      ms_elapsed += 1200;
      led2_toggle();
    }

    if (ms_elapsed & (1<<14))
    {
        if (gas_leak)
          printf("GasLeakAlert\r\n");
        ms_elapsed = 0;
        tempToUSB(12);
    }
  }
}
