#include <m_spi_esp32c3.h>

void m_spi_write_register(spi_device_handle_t handle, uint8_t reg, uint8_t val)
{
    spi_write_register.tx_data[0] = val;
    spi_write_register.addr = reg;
    spi_device_polling_transmit(handle, &spi_write_register);
}

uint8_t m_spi_read_register(spi_device_handle_t handle, uint8_t reg)
{
    spi_read_register.addr = reg;
    spi_device_polling_transmit(handle, &spi_read_register);
    return spi_read_register.rx_data[0];
}

// FUNCTIONALITY:
//   Serial read multiple SPI device bytes starting from an 8-bit register
//   Note: SPI device must support burst/serial register reads
//
// ARGUMENTS:
//   cs_pin: an enumerated M2 general-purpose I/O (GPIO) pin
//   start_reg: 8-bit SPI device register
//   num_bytes: number of bytes to retrieve from SPI device
//   dest: pointer to a byte/char buffer for storing received bytes
void m_spi_read_registers(spi_device_handle_t handle, uint8_t start_reg, uint8_t num_bytes, uint8_t *dest)
{
    spi_read_registers.addr = start_reg;
    spi_read_registers.length = num_bytes;
    spi_read_registers.rxlength = num_bytes;
    dest = (uint8_t *)spi_read_registers.rx_buffer;
    spi_device_polling_transmit(handle, &spi_read_register);
}
