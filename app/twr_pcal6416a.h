#ifndef _TWR_PCAL6416A_H
#define _TWR_PCAL6416A_H

#include <twr_i2c.h>

typedef enum
{
    TWR_PCAL6416A_PIN_P0_0 = 0,
    TWR_PCAL6416A_PIN_P0_1 = 1, 
    TWR_PCAL6416A_PIN_P0_2 = 2,
    TWR_PCAL6416A_PIN_P0_3 = 3,         
    TWR_PCAL6416A_PIN_P0_4 = 4,
    TWR_PCAL6416A_PIN_P0_5 = 5,
    TWR_PCAL6416A_PIN_P0_6 = 6,
    TWR_PCAL6416A_PIN_P0_7 = 7,
    TWR_PCAL6416A_PIN_P1_0 = 8,
    TWR_PCAL6416A_PIN_P1_1 = 9,
    TWR_PCAL6416A_PIN_P1_2 = 10,
    TWR_PCAL6416A_PIN_P1_3 = 11,
    TWR_PCAL6416A_PIN_P1_4 = 12,
    TWR_PCAL6416A_PIN_P1_5 = 13,
    TWR_PCAL6416A_PIN_P1_6 = 14,
    TWR_PCAL6416A_PIN_P1_7 = 15

} twr_pcal6416a_pin_t;

typedef struct
{
    twr_i2c_channel_t _i2c_channel;
    uint8_t _i2c_address;
    uint8_t _direction_port_0;
    uint8_t _direction_port_1;
    uint8_t _output_port_0;
    uint8_t _output_port_1;
} twr_pcal6416a_t;

typedef enum
{
    TWR_PCAL6416A_PIN_DIRECTION_OUTPUT = 0,
    TWR_PCAL6416A_PIN_DIRECTION_INPUT = 1

} twr_pcal6416a_pin_direction_t;

bool twr_pcal6416a_init(twr_pcal6416a_t *self, twr_i2c_channel_t i2c_channel, uint8_t i2c_address);
bool twr_pcal6416a_set_pin_direction(twr_pcal6416a_t *self, twr_pcal6416a_pin_t pin, twr_pcal6416a_pin_direction_t direction);
bool twr_pcal6416a_write_pin(twr_pcal6416a_t *self, twr_pcal6416a_pin_t pin, int value);

#endif // _TWR_pcal6416a_H
