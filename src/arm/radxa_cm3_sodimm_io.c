/*
 * Copyright (c) 2022 Radxa Computer Co.,Ltd
 * Author: Nascs <nascs@vamrs.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/radxa_cm3_sodimm_io.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_RADXA_CM3_SODIMM_IO "Radxa CM3 SODIMM IO"
#define MAX_SIZE 64

const char* radxa_cm3_sodimm_io_serialdev[MRAA_RADXA_CM3_SODIMM_IO_UART_COUNT] = { "/dev/ttyS2","/dev/ttyS5","/dev/ttyS7" };

void
mraa_radxa_cm3_sodimm_io_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
{
    va_list arg_ptr;
    if (index > board->phy_pin_count)
        return;

    mraa_pininfo_t* pininfo = &board->pins[index];
    va_start(arg_ptr, fmt);
    vsnprintf(pininfo->name, MRAA_PIN_NAME_SIZE, fmt, arg_ptr);

    if( pincapabilities_t.gpio == 1 ) {
        va_arg(arg_ptr, int);
        pininfo->gpio.gpio_chip = va_arg(arg_ptr, int);
        pininfo->gpio.gpio_line = va_arg(arg_ptr, int);
    }

    pininfo->capabilities = pincapabilities_t;

    va_end(arg_ptr);
    pininfo->gpio.pinmap = sysfs_pin;
    pininfo->gpio.mux_total = 0;
}

mraa_board_t*
mraa_radxa_cm3_sodimm_io()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_RADXA_CM3_SODIMM_IO_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_RADXA_CM3_SODIMM_IO)) {
            b->platform_name = PLATFORM_NAME_RADXA_CM3_SODIMM_IO;
            b->uart_dev[0].device_path = (char*) radxa_cm3_sodimm_io_serialdev[0];
            b->uart_dev[1].device_path = (char*) radxa_cm3_sodimm_io_serialdev[1];
            b->uart_dev[2].device_path = (char*) radxa_cm3_sodimm_io_serialdev[2];
        }
    }

    // UART*3
    b->uart_dev_count = MRAA_RADXA_CM3_SODIMM_IO_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;
    b->uart_dev[1].index = 5;
    b->uart_dev[2].index = 7;

    // I2C*4
    if (strncmp(b->platform_name, PLATFORM_NAME_RADXA_CM3_SODIMM_IO, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_RADXA_CM3_SODIMM_IO_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 0;
        b->i2c_bus[1].bus_id = 1;
        b->i2c_bus[2].bus_id = 2;
        b->i2c_bus[3].bus_id = 4;
    }

    // SPI*3
    b->spi_bus_count = MRAA_RADXA_CM3_SODIMM_IO_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;
    b->spi_bus[1].bus_id = 1;
    b->spi_bus[2].bus_id = 3;

    // PWM*6
    b->pwm_dev_count = MRAA_RADXA_CM3_SODIMM_IO_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[13].pwm.parent_id = 0;
    b->pins[13].pwm.mux_total = 0;
    b->pins[13].pwm.pinmap = 0;
    b->pins[16].pwm.parent_id = 0;
    b->pins[16].pwm.mux_total = 0;
    b->pins[16].pwm.pinmap = 0;
    b->pins[18].pwm.parent_id = 0;
    b->pins[18].pwm.mux_total = 0;
    b->pins[18].pwm.pinmap = 0;
    b->pins[21].pwm.parent_id = 0;
    b->pins[21].pwm.mux_total = 0;
    b->pins[21].pwm.pinmap = 0;
    b->pins[33].pwm.parent_id = 0;
    b->pins[33].pwm.mux_total = 0;
    b->pins[33].pwm.pinmap = 0;
    b->pins[37].pwm.parent_id = 0;
    b->pins[37].pwm.mux_total = 0;
    b->pins[37].pwm.pinmap = 0; 

    mraa_radxa_cm3_sodimm_io_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 3,   12, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C1_SDA");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 5,   11, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C1_SCL");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 7,  124, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D4");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 8,   25, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_TX");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 10,  24, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_RX");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 11, 131, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART7_RX_M2");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 12,  97, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1_CS0_M1");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 13,  13, (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "SPI0_CLK_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 15, 128, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_A0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 16,  20, (mraa_pincapabilities_t){1,1,1,0,1,0,0,0}, "SPI0_CS1_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 18,  22, (mraa_pincapabilities_t){1,1,1,0,1,0,0,0}, "SPI0_CS0_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 19, 138, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SPI3_MOSI_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 21, 136, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI3_MISO_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 22,  21, (mraa_pincapabilities_t){1,1,1,0,1,0,0,0}, "SPI0_MISO_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 23, 139, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SPI3_CLK_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 24, 134, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI3_CS0_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 26, 135, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI3_CS1_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 27, 140, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C2_SDA_M1");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 28, 141, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C2_SCL_M1");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 29, 143, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_B7");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 31, 125, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D5");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 32,  15, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PMW0_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 33, 144, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM11_IR_M1");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 35, 114, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1_MISO_M1");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 36, 130, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART7_TX_M2");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 37,  14, (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "SPI0_MOSI_M0");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 38, 113, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1_MOSI_M1");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_sodimm_io_pininfo(b, 40, 115, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1_CLK_M1");

    return b;
}
