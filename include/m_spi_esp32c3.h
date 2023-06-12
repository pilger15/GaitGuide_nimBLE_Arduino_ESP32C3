#pragma once

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "driver/spi_master.h"

spi_transaction_t spi_write_register = {
    .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x80,
    .length = 8,
    .rxlength = 8};
spi_transaction_t spi_read_register = {
    .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x00,
    .length = 8,
    .rxlength = 8};

uint8_t rx_buffer[512 * 7];
spi_transaction_t spi_read_registers = {
    .cmd = 0x00,
    .rx_buffer = rx_buffer};

// FUNCTIONALITY:
//   Write an 8-bit value to a specific SPI device's 8-bit register
//
// ARGUMENTS:
//   cs_pin: an enumerated M2 general-purpose I/O (GPIO) pin
//   reg: 8-bit SPI device register
//   val: 8-bit value to write to SPI device register
void m_spi_write_register(spi_device_handle_t handle, uint8_t reg, uint8_t val);

// FUNCTIONALITY:
//   Read an 8-bit value from a specific SPI device's 8-bit register
//
// ARGUMENTS:
//   cs_pin: an enumerated M2 general-purpose I/O (GPIO) pin
//   reg: 8-bit SPI device register
//
// RETURNS:
//   8-bit value from SPI device register
uint8_t m_spi_read_register(spi_device_handle_t handle, uint8_t reg);

// FUNCTIONALITY:
//   Serial read multiple SPI device bytes starting from an 8-bit register
//   Note: SPI device must support burst/serial register reads
//
// ARGUMENTS:
//   cs_pin: an enumerated M2 general-purpose I/O (GPIO) pin
//   start_reg: 8-bit SPI device register
//   num_bytes: number of bytes to retrieve from SPI device
//   dest: pointer to a byte/char buffer for storing received bytes
void m_spi_read_registers(spi_device_handle_t handle,
                          uint8_t start_reg,
                          uint8_t num_bytes,
                          uint8_t *dest);
