##
##  usb-ajar
##
##
##  Copyright (C) 2013-2014 Christian Pointner <equinox@spreadspace.org>
##
##  This file is part of usb-ajar.
##
##  usb-ajar is free software: you can redistribute it and/or modify
##  it under the terms of the GNU General Public License as published by
##  the Free Software Foundation, either version 3 of the License, or
##  any later version.
##
##  usb-ajar is distributed in the hope that it will be useful,
##  but WITHOUT ANY WARRANTY; without even the implied warranty of
##  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##  GNU General Public License for more details.
##
##  You should have received a copy of the GNU General Public License
##  along with usb-ajar. If not, see <http://www.gnu.org/licenses/>.
##

NAME := usb-ajar
BOARD_TYPE := minimus32
OBJ := $(NAME).o
LIBS := util led lufa-descriptor-usbserial anyio onewire ds1820
EXTERNAL_LIBS := lufa
SPREADAVR_PATH := ../contrib/avr-utils
RESET_FUNC := $(SPREADAVR_PATH)/tools/reset_lufa_cdc
RESET_PARAM := 'r'

LUFA_PATH := $(SPREADAVR_PATH)/contrib/lufa-LUFA-151115
LUFA_OPTS  = -D USB_DEVICE_ONLY
LUFA_OPTS += -D DEVICE_STATE_AS_GPIOR=0
LUFA_OPTS += -D ORDERED_EP_CONFIG
LUFA_OPTS += -D FIXED_CONTROL_ENDPOINT_SIZE=8
LUFA_OPTS += -D FIXED_NUM_CONFIGURATIONS=1
LUFA_OPTS += -D USE_FLASH_DESCRIPTORS
LUFA_OPTS += -D USE_STATIC_OPTIONS="(USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)"
LUFA_OPTS += -D INTERRUPT_CONTROL_ENDPOINT

LUFA_OPTS += -D USB_MANUFACTURER="L\"xro\""
LUFA_OPTS += -D USB_PRODUCT="L\"spreadspace usb-ajar example\""

LUFA_COMPONENTS := USB USBCLASS SERIAL

include $(SPREADAVR_PATH)/include.mk
