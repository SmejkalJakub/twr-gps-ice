#include <twr_pcal6416a.h>

#define _TWR_PCAL6416A_PORT_0_CONFIGURATION_REGISTER 0x06
#define _TWR_PCAL6416A_PORT_1_CONFIGURATION_REGISTER 0x07

#define _TWR_PCAL6416A_OUTPUT_PORT_0_REGISTER 0x02
#define _TWR_PCAL6416A_OUTPUT_PORT_1_REGISTER 0x03

bool twr_pcal6416a_init(twr_pcal6416a_t *self, twr_i2c_channel_t i2c_channel, uint8_t i2c_address)
{
    memset(self, 0, sizeof(*self));

    self->_i2c_channel = i2c_channel;
    self->_i2c_address = i2c_address;

    twr_i2c_init(self->_i2c_channel, TWR_I2C_SPEED_400_KHZ);

    if (!twr_i2c_memory_read_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_PORT_0_CONFIGURATION_REGISTER, &self->_direction_port_0))
    {
        return false;
    }

    if (!twr_i2c_memory_read_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_PORT_1_CONFIGURATION_REGISTER, &self->_direction_port_1))
	{
		return false;
	}

    if (!twr_i2c_memory_read_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_OUTPUT_PORT_1_REGISTER, &self->_output_port_1))
	{
		return false;
	}

    if (!twr_i2c_memory_read_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_OUTPUT_PORT_0_REGISTER, &self->_output_port_0))
    {
        return false;
    }

    return true;
}

bool twr_pcal6416a_set_pin_direction(twr_pcal6416a_t *self, twr_pcal6416a_pin_t pin, twr_pcal6416a_pin_direction_t direction)
{
    if(pin > 7)
    {
        pin = pin - 8;

        uint8_t port_direction = self->_direction_port_1;

        port_direction &= ~(1 << (uint8_t) pin);

        if (direction == TWR_PCAL6416A_PIN_DIRECTION_INPUT)
        {
            port_direction |= 1 << (uint8_t) pin;
        }

        if (!twr_i2c_memory_write_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_PORT_1_CONFIGURATION_REGISTER, port_direction))
        {
            return false;
        }

        self->_direction_port_1 = port_direction;
    }
    else
    {
        uint8_t port_direction = self->_direction_port_0;

        port_direction &= ~(1 << (uint8_t) pin);

        if (direction == TWR_PCAL6416A_PIN_DIRECTION_INPUT)
        {
            port_direction |= 1 << (uint8_t) pin;
        }

        if (!twr_i2c_memory_write_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_PORT_0_CONFIGURATION_REGISTER, port_direction))
        {
            return false;
        }

        self->_direction_port_0 = port_direction;
    }

    return true;
}

bool twr_pcal6416a_write_pin(twr_pcal6416a_t *self, twr_pcal6416a_pin_t pin, int value)
{
    if(pin > 7)
    {
        pin = pin - 8;

        uint8_t port = self->_output_port_1;

        port &= ~(1 << (uint8_t) pin);

        if (value != 0)
        {
            port |= 1 << (uint8_t) pin;
        }

        if (!twr_i2c_memory_write_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_OUTPUT_PORT_1_REGISTER, port))
        {
            return false;
        }

        self->_output_port_1 = port;
    }
    else
    {
        uint8_t port = self->_output_port_0;

        port &= ~(1 << (uint8_t) pin);

        if (value != 0)
        {
            port |= 1 << (uint8_t) pin;
        }

        if (!twr_i2c_memory_write_8b(self->_i2c_channel, self->_i2c_address, _TWR_PCAL6416A_OUTPUT_PORT_0_REGISTER, port))
        {
            return false;
        }

        self->_output_port_0 = port;
    }

    return true;
}
