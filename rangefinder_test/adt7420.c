
#include <stdlib.h>
#include "adt7420.h"

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
uint8_t adt7420_get_register_value(uint8_t register_address)
{
	uint8_t register_value = 0;
	
	/*
	i2c_write(dev->i2c_desc, &register_address, 1, 0);
	i2c_read(dev->i2c_desc, &register_value, 1, 1);
	*/

	i2c_start( ADT7420_ADDRESS | I2C_WRITE );
	i2c_write( register_address );
	i2c_rep_start( ADT7420_ADDRESS | I2C_READ );
	register_value = i2c_readNak();
	i2c_stop();

	return register_value;
}

/***************************************************************************//**
 * @brief Sets the value of a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 * @param register_value   - Value of the register.
 *
 * @return None.
*******************************************************************************/
void adt7420_set_register_value(uint8_t register_address, uint8_t register_value) {
	
	/*
		uint8_t data_buffer[2] = {0, 0};
		data_buffer[0] = register_address;
		data_buffer[1] = register_value;
		i2c_write(dev->i2c_desc, data_buffer, 2, 1);
	*/
	
	i2c_start( ADT7420_ADDRESS | I2C_WRITE );
	i2c_write(register_address);
	i2c_write(register_value);
	i2c_stop();
	
}



/*****************************************************************************
 * @brief Initializes the communication peripheral and checks if the device is
 *        present.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return status - The result of the initialization procedure.
 *                  Example: -1 - I2C peripheral was not initialized or the
 *                                device is not present.
 *                            0 - I2C peripheral was initialized and the
 *                                device is present.
*******************************************************************************/

int32_t adt7420_init() {
	
	/*
	struct adt7420_dev *dev;
	int32_t status;
	uint8_t test = 0;

	dev = (struct adt7420_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

	//I2C
	status = i2c_init(&dev->i2c_desc, &init_param.i2c_init);

	//Device Settings
	dev->resolution_setting = init_param.resolution_setting;
*/
	
	int32_t status = 0;
	uint8_t test = 0;
	test = adt7420_get_register_value(ADT7420_REG_ID);
	if(test != ADT7420_DEFAULT_ID)
		status = -1;

	//*device = dev;

	return status;
}

/***************************************************************************//**
 * @brief Free the resources allocated by adt7420_init().
 *
 * @param dev - The device structure.
 *
 * @return ret - The result of the remove procedure.
*******************************************************************************/
/*
int32_t adt7420_remove(struct adt7420_dev *dev)
{
	int32_t ret;

	ret = i2c_remove(dev->i2c_desc);

	free(dev);

	return ret;
}
*/
/***************************************************************************//**
 * @brief Resets the ADT7420.
 *        The ADT7420 does not respond to I2C bus commands while the default
 *        values upload (approximately 200 us).
 *
 * @param dev - The device structure.
 *
 * @return None.
*******************************************************************************/
/*
void adt7420_reset(struct adt7420_dev *dev)
{
	uint8_t register_address = ADT7420_REG_RESET;

	i2c_write(dev->i2c_desc,
		  &register_address,
		  1,
		  1);
	dev->resolution_setting = 0;
}
*/
/***************************************************************************//**
 * @brief Sets the operational mode for ADT7420.
 *
 * @param dev  - The device structure.
 * @param mode - Operation mode.
 *               Example: ADT7420_OP_MODE_CONT_CONV - continuous conversion;
 *                        ADT7420_OP_MODE_ONE_SHOT  - one shot;
 *                        ADT7420_OP_MODE_1_SPS     - 1 SPS mode;
 *                        ADT7420_OP_MODE_SHUTDOWN  - shutdown.
 *
 * @return None.
*******************************************************************************/
/*
void adt7420_set_operation_mode(struct adt7420_dev *dev,
				uint8_t mode)
{
	uint8_t register_value = 0;

	register_value  = adt7420_get_register_value(dev, ADT7420_REG_CONFIG);
	register_value &= ~ADT7420_CONFIG_OP_MODE(ADT7420_OP_MODE_SHUTDOWN);
	register_value |= ADT7420_CONFIG_OP_MODE(mode);
	adt7420_set_register_value(dev, ADT7420_REG_CONFIG, register_value);
}
*/
/***************************************************************************//**
 * @brief Sets the resolution for ADT7420.
 *
 * @param dev        - The device structure.
 * @param resolution - Resolution.
 *                     Example: 0 - 13-bit resolution;
 *                              1 - 16-bit resolution.
 *
 * @return None.
*******************************************************************************/
/*
void adt7420_set_resolution(struct adt7420_dev *dev,
			    uint8_t resolution)
{
	uint8_t register_value = 0;

	register_value  = adt7420_get_register_value(dev, ADT7420_REG_CONFIG);
	register_value &= ~ADT7420_CONFIG_RESOLUTION;
	register_value |= (resolution * ADT7420_CONFIG_RESOLUTION);
	adt7420_set_register_value(dev, ADT7420_REG_CONFIG, register_value);
	dev->resolution_setting = resolution;
}
*/
/***************************************************************************//**
 * @brief Reads the temperature data and converts it to Celsius degrees.
 *
 * @param dev - The device structure.
 *
 * @return temperature - Temperature in degrees Celsius.
*******************************************************************************/
/*
float adt7420_get_temperature(struct adt7420_dev *dev)
{
	uint8_t msb_temp = 0;
	uint8_t lsb_temp = 0;
	uint16_t temp = 0;
	float temp_c = 0;

	msb_temp = adt7420_get_register_value(dev, ADT7420_REG_TEMP_MSB);
	lsb_temp = adt7420_get_register_value(dev, ADT7420_REG_TEMP_LSB);
	temp    = ((uint16_t)msb_temp << 8) + lsb_temp;
	if(dev->resolution_setting) {
		if(temp & 0x8000)
			//! Negative temperature
			temp_c = (float)((int32_t)temp - 65536) / 128;
		else
			//! Positive temperature
			temp_c = (float)temp / 128;
	} else {
		temp >>= 3;
		if(temp & 0x1000)
			//Negative temperature
			temp_c = (float)((int32_t)temp - 8192) / 16;
		else
			// Positive temperature
			temp_c = (float)temp / 16;
	}

	return temp_c;
}
*/