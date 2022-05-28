// Tower Kit documentation https://tower.hardwario.com/
// SDK API description https://sdk.hardwario.com/
// Forum https://forum.hardwario.com/

#define VDDA_VOLTAGE 3.3f

#include <application.h>

// LED instance
twr_led_t led;

// Expander insance
twr_pcal6416a_t pcal6416a;

// Proggress of the whole test sequence
int test_progress = 0;

twr_button_t button_left;
twr_button_t button_right;

// Initial state for the tester function
test_state state = TEST_EXPANDER;

twr_scheduler_task_id_t lcd_print_task;
twr_scheduler_task_id_t tester_task;

twr_led_t lcdLedRed;
twr_led_t lcdLedGreen;
twr_led_t lcdLedBlue;

// Results of each test
bool test_results[5];

// Some flags for the tester
bool x0A_version = false;

// Result voltages of each GPIO sub test
float gpio_test_results_voltages[5];

void tester();
void lcd_print_results();

// Task to print the results onto LCD 
void lcd_print_results()
{
    if(!twr_module_lcd_is_ready())
    {
        twr_scheduler_plan_current_relative(20);
        return;
    }

    twr_system_pll_enable();

    twr_module_lcd_clear();

    twr_module_lcd_set_font(&twr_font_ubuntu_15);
    twr_module_lcd_draw_string(0, 5, "CHESTER TESTER X0", 1);
    twr_module_lcd_draw_string(0, 15, "--------------------------", 1);

    if(test_progress == 0)
    {
        twr_module_lcd_set_font(&twr_font_ubuntu_13);
        twr_module_lcd_draw_string(15, 43, "Insert chester X0", 1);
        twr_module_lcd_draw_string(40, 58, "and hold", 1);
        twr_module_lcd_draw_string(8, 73, "right button for X0A", 1);
        twr_module_lcd_draw_string(12, 88, "left button for X0B", 1);

    }

    twr_module_lcd_set_font(&twr_font_ubuntu_13);

    if(test_progress > 0)
    {
        if(test_results[0])
        {
            twr_module_lcd_draw_string(0, 25, "Expander test: OK", 1);
        }
        else
        {
            twr_module_lcd_draw_string(3, 25, "Expander test: FAILED", 1);
        }
    }

    if(test_progress > 1)
    {
        if(test_results[1])
        {
            twr_module_lcd_draw_string(0, 35, "GP0/A0 test: OK", 1);
        }
        else
        {
            twr_module_lcd_draw_string(0, 35, "GP0/A0 test: FAILED", 1);
        }
    }

    if(test_progress > 2)
    {
        if(test_results[2])
        {
            twr_module_lcd_draw_string(0, 45, "GP1/A1 test: OK", 1);
        }
        else
        {
            twr_module_lcd_draw_string(0, 45, "GP1/A1 test: FAILED", 1);
        }
    }

    if(test_progress > 3)
    {
        if(test_results[3])
        {
            twr_module_lcd_draw_string(0, 55, "GP2/A2 test: OK", 1);
        }
        else
        {
            twr_module_lcd_draw_string(0, 55, "GP2/A2 test: FAILED", 1);
        }
    }

    if(test_progress > 4)
    {
        if(test_results[4])
        {
            twr_module_lcd_draw_string(0, 65, "GP3/A3 test: OK", 1);
        }
        else
        {
            twr_module_lcd_draw_string(0, 65, "GP3/A3 test: FAILED", 1);
        }

        twr_module_lcd_draw_string(0, 75, "--------------------------", 1);

        twr_module_lcd_draw_string(0, 85, "Hold right button", 1);
        twr_module_lcd_draw_string(0, 95, "to test another", 1);
        twr_module_lcd_draw_string(0, 105, "chester X0", 1);

        int numberOfErrors = 0;
        for(int i = 0; i < 5; i++)
        {
            if(!test_results[i])
            {
                numberOfErrors++;
            }
        }

        twr_led_set_mode(&lcdLedBlue, TWR_LED_MODE_OFF);

        if(numberOfErrors == 0)
        {
            twr_led_set_mode(&lcdLedGreen, TWR_LED_MODE_ON);
        }
        else
        {
            twr_led_set_mode(&lcdLedRed, TWR_LED_MODE_ON);
        }

    }

    twr_module_lcd_update();

    twr_system_pll_disable();

    twr_scheduler_plan_current_relative(200);
}

// Reset all the pins into the default state (INPUT with 1 at the output register). Cleanup after each test
void reset_gpio(int test_index)
{
    for(int i = 0; i < 4; i++)
    {
        twr_pcal6416a_set_pin_direction(&pcal6416a, gpio_pins[test_index][i], TWR_PCAL6416A_PIN_DIRECTION_INPUT);
        twr_pcal6416a_write_pin(&pcal6416a, gpio_pins[test_index][i], 1);
    }
}

// Get the voltage for each GPIO sub test
void gpio_test_get_voltage(int sub_test_index, twr_adc_channel_t adc_channel, float *voltage)
{    
    float vdda_voltage = NAN;
    twr_adc_get_vdda_voltage(&vdda_voltage);
    
    uint16_t adc;
    twr_adc_get_value(adc_channel, &adc);

    *voltage = (adc * VDDA_VOLTAGE) / 65536.f;

    twr_log_debug("TEST %d: %.5f", sub_test_index + 1, *voltage);
}

// LCD button handler. Left hold starts the test for Chester X0B (GPIO sub test 3 is skiped), Right hold starts the test for Chester X0A
void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    (void) self;

    if(self == &button_right && event == TWR_BUTTON_EVENT_HOLD && test_progress == 0)
    {
        twr_led_set_mode(&lcdLedBlue, TWR_LED_MODE_ON);

        x0A_version = true;
        twr_scheduler_plan_now(tester_task);
    }
    if(self == &button_left && event == TWR_BUTTON_EVENT_HOLD && test_progress == 0)
    {
        twr_led_set_mode(&lcdLedBlue, TWR_LED_MODE_ON);

        x0A_version = false;
        twr_scheduler_plan_now(tester_task);
    }
    else if(self == &button_right && event == TWR_BUTTON_EVENT_HOLD && test_progress > 4)
    {        
        x0A_version = true;

        twr_led_set_mode(&lcdLedRed, TWR_LED_MODE_OFF);
        twr_led_set_mode(&lcdLedGreen, TWR_LED_MODE_OFF);
        twr_led_set_mode(&lcdLedBlue, TWR_LED_MODE_ON);

        test_progress = 0;
        state = TEST_EXPANDER;
        twr_scheduler_plan_now(tester_task);
    }
    else if(self == &button_left && event == TWR_BUTTON_EVENT_HOLD && test_progress > 4)
    {
        x0A_version = false;

        twr_led_set_mode(&lcdLedRed, TWR_LED_MODE_OFF);
        twr_led_set_mode(&lcdLedGreen, TWR_LED_MODE_OFF);
        twr_led_set_mode(&lcdLedBlue, TWR_LED_MODE_ON);

        test_progress = 0;
        state = TEST_EXPANDER;
        twr_scheduler_plan_now(tester_task);
    }
}

void gpio_set_test_pins(int test_index, int sub_test_index)
{
    for(int pin = 0; pin < 4; pin++)
    {
        twr_pcal6416a_set_pin_direction(&pcal6416a, gpio_pins[test_index][pin], gpio_subtest_pins_setup[sub_test_index][pin]);
        if(gpio_subtest_pins_setup[sub_test_index][pin] == TWR_PCAL6416A_PIN_DIRECTION_OUTPUT)
        {
            twr_pcal6416a_write_pin(&pcal6416a, gpio_pins[test_index][pin], gpio_subtest_pins_output[sub_test_index][pin]);
        }
    }
}

void delay()
{
    twr_delay_us(60000);
    twr_delay_us(60000);
}

// State machine of the whole tester
void tester()
{
    start:
    switch(state)
    {
        // Simple test if the expander is communicatig well
        case TEST_EXPANDER:
            if(!twr_pcal6416a_init(&pcal6416a, TWR_I2C_I2C0, 0x20))
            {
                test_results[test_progress] = false;

                state = GPIO_TEST;
                test_progress++;
                twr_delay_us(60000);
                goto start;
            }
            else
            {
                test_results[test_progress] = true;

                state = GPIO_TEST;
                test_progress++;
                twr_delay_us(60000);
                goto start;
            }
            break;
        case GPIO_TEST:
            for(int test_index = 0; test_index < NUMBER_OF_GPIO_TESTS; test_index++)
            {
                if(test_results[0] == false)
                {
                    twr_log_debug("jsem tu");
                    test_results[test_index + 1] = false;
                    test_progress++;
                    continue;
                }

                reset_gpio(test_index);
                delay();

                twr_log_debug("START OF GPIO TEST %d", test_index + 1);
                for(int sub_test_index = 0; sub_test_index < NUMBER_OF_GPIO_SUB_TESTS; sub_test_index++)
                {
                    if(!x0A_version && sub_test_index == 2)
                    {
                        continue;
                    }

                    gpio_set_test_pins(test_index, sub_test_index);
                    twr_adc_calibration();
                    delay();

                    float voltage = NAN;
                    gpio_test_get_voltage(sub_test_index, gpio_adc[test_index], &voltage);
                    gpio_test_results_voltages[sub_test_index] = voltage;

                    reset_gpio(test_index);
                    delay();
                }

                if(x0A_version)
                {
                    if(gpio_test_results_voltages[0] <= gpio_test_results_voltages[3] &&
                    gpio_test_results_voltages[0] < 0.02f &&
                    gpio_test_results_voltages[1] < 0.15f && gpio_test_results_voltages[1] > 0.08f &&
                    gpio_test_results_voltages[2] < 0.47f && gpio_test_results_voltages[2] > 0.42f &&
                    gpio_test_results_voltages[3] < 0.08f &&
                    gpio_test_results_voltages[4] > 0.007f && gpio_test_results_voltages[4] < 0.03f)
                    {
                        test_results[test_index + 1] = true;
                    }
                    else
                    {
                        test_results[test_index + 1] = false;
                    }
                }
                else
                {
                    if(gpio_test_results_voltages[0] <= gpio_test_results_voltages[3] &&
                    gpio_test_results_voltages[0] < 0.02f &&
                    gpio_test_results_voltages[1] < 0.15f && gpio_test_results_voltages[1] > 0.08f &&
                    gpio_test_results_voltages[3] < 0.04f &&
                    gpio_test_results_voltages[4] > 0.007f && gpio_test_results_voltages[4] < 0.03f)
                    {
                        test_results[test_index + 1] = true;
                    }
                    else
                    {
                        test_results[test_index + 1] = false;
                    }
                }
                test_progress++;
                twr_log_debug("END OF GPIO TEST %d", test_index + 1);
                twr_log_debug("---------------------------------------------");
            }
            break;

        default:
            return;
    }
}

// Application initialization function which is called once after boot
void application_init(void)
{
    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, 0);
    twr_led_pulse(&led, 2000);

    // Initialize ADC
    twr_adc_init();

    twr_adc_resolution_set(TWR_ADC_CHANNEL_A2, TWR_ADC_RESOLUTION_12_BIT);
    twr_adc_oversampling_set(TWR_ADC_CHANNEL_A2, TWR_ADC_OVERSAMPLING_16);

    twr_adc_resolution_set(TWR_ADC_CHANNEL_A3, TWR_ADC_RESOLUTION_12_BIT);
    twr_adc_oversampling_set(TWR_ADC_CHANNEL_A3, TWR_ADC_OVERSAMPLING_16);

    twr_adc_resolution_set(TWR_ADC_CHANNEL_A4, TWR_ADC_RESOLUTION_12_BIT);
    twr_adc_oversampling_set(TWR_ADC_CHANNEL_A4, TWR_ADC_OVERSAMPLING_16);

    twr_adc_resolution_set(TWR_ADC_CHANNEL_A5, TWR_ADC_RESOLUTION_12_BIT);
    twr_adc_oversampling_set(TWR_ADC_CHANNEL_A5, TWR_ADC_OVERSAMPLING_16);

    // Initialize LCD
    twr_module_lcd_init();
    twr_module_lcd_set_font(&twr_font_ubuntu_13);
    twr_module_lcd_update();

    const twr_button_driver_t* lcdButtonDriver =  twr_module_lcd_get_button_driver();
    twr_button_init_virtual(&button_left, 0, lcdButtonDriver, 0);
    twr_button_init_virtual(&button_right, 1, lcdButtonDriver, 0);
    twr_button_set_event_handler(&button_left, button_event_handler, (int*)0);
    twr_button_set_event_handler(&button_right, button_event_handler, (int*)1);

    const twr_led_driver_t* driver = twr_module_lcd_get_led_driver();
    twr_led_init_virtual(&lcdLedRed, TWR_MODULE_LCD_LED_RED, driver, 1);
    twr_led_init_virtual(&lcdLedGreen, TWR_MODULE_LCD_LED_GREEN, driver, 1);
    twr_led_init_virtual(&lcdLedBlue, TWR_MODULE_LCD_LED_BLUE, driver, 1);

    twr_button_set_hold_time(&button_left, 300);
    twr_button_set_hold_time(&button_right, 300);

    twr_button_set_debounce_time(&button_left, 30);
    twr_button_set_debounce_time(&button_right, 30);

    // Set up all the tasks
    lcd_print_task = twr_scheduler_register(lcd_print_results, NULL, 1000);
    tester_task = twr_scheduler_register(tester, NULL, TWR_TICK_INFINITY);
}
