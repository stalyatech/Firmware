/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <lib/drivers/device/spi.h>


/* SPI protocol address bits */
#define DIR_READ(a)	((a) | 0x80)
#define DIR_WRITE(a)    ((a) & 0x7f)

class BMI088_SPI : public device::SPI
{
public:
	/**
	 * Constructor
	 *
	 * @param device_type	The device type (see drv_sensor.h)
	 * @param name		Driver name
	 * @param bus		SPI bus on which the device lives
	 * @param device	Device handle (used by SPI_SELECT)
	 * @param mode		SPI clock/data mode
	 * @param frequency	SPI clock frequency
	 */
	BMI088_SPI(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency);

	~BMI088_SPI() override = default;

	/**
	 * Read directly from the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param reg	The register address at which to start reading
	 * @param data	The buffer into which the read values should be placed.
	 * @param count	The number of items to read.
	 * @return	The number of items read on success, negative errno otherwise.
	 */
	int read(unsigned reg, void *data, unsigned count) override;

	/**
	 * Write directly to the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param reg	The register address at which to start writing.
	 * @param data	The buffer from which values should be read.
	 * @param count	The number of items to write.
	 * @return	The number of items written on success, negative errno otherwise.
	 */
	int write(unsigned reg, void *data, unsigned count) override;

	/**
	 * Read a register from the device.
	 *
	 * @param	The register to read.
	 * @return	The value that was read.
	 */
	uint8_t read_reg(unsigned reg) override;

	/**
	 * Write a register to the device.
	 *
	 * @param reg	The register to write.
	 * @param value	The new value to write.
	 * @return	OK on success, negative errno otherwise.
	 */
	int write_reg(unsigned reg, uint8_t value) override;
};

BMI088_SPI::BMI088_SPI(uint8_t device_type, const char *name, const int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	SPI(device_type, name, bus, device, mode, frequency)
{
}

/**
 * Read directly from the device.
 *
 * The actual size of each unit quantity is device-specific.
 *
 * @param reg	The register address at which to start reading
 * @param data	The buffer into which the read values should be placed.
 * @param count	The number of items to read.
 * @return	The number of items read on success, negative errno otherwise.
 */
int BMI088_SPI::read(unsigned reg, void *data, unsigned count)
{
	return PX4_ERROR;
}

/**
 * Write directly to the device.
 *
 * The actual size of each unit quantity is device-specific.
 *
 * @param reg	The register address at which to start writing.
 * @param data	The buffer from which values should be read.
 * @param count	The number of items to write.
 * @return	The number of items written on success, negative errno otherwise.
 */
int BMI088_SPI::write(unsigned reg, void *data, unsigned count)
{
	return PX4_ERROR;
}

/**
 * Read a register from the device.
 *
 * @param	The register to read.
 * @return	The value that was read.
 */
uint8_t BMI088_SPI::read_reg(unsigned reg)
{
	return PX4_ERROR;
}

/**
 * Write a register to the device.
 *
 * @param reg	The register to write.
 * @param value	The new value to write.
 * @return	OK on success, negative errno otherwise.
 */
int BMI088_SPI::write_reg(unsigned reg, uint8_t value)
{
	return PX4_ERROR;
}

device::Device *BMI088_SPI_interface(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency)
{
	return new BMI088_SPI(device_type, name, bus, device, mode, frequency);
}
