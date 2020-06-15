/*
Copyright 2018 Mike Roberts

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */
#define VENDOR_ID       0xFEED
#define PRODUCT_ID      0x0000
#define DEVICE_VER      0x0001
#define MANUFACTURER    jskuby
#define PRODUCT         9KeyFeather
#define DESCRIPTION     9 Key Feather

/* USB parameters */
#define USB_POLLING_INTERVAL_MS 1

/* Matrix values */
#define MATRIX_ROWS 3
#define MATRIX_COLS 3
#define MATRIX_COL_PINS { GPA0, GPA1, GPA2 } // Col pins are read from the I/O expander
#define MATRIX_ROW_PINS { F7,   F6,   F5   } // Row pins are read from the Feather pins, unused: F4, F1, F0
#define DIODE_DIRECTION COL2ROW
#define DEBOUNCE 0

/* SPI Overrides */
#define SPI_SCK_PIN C7
