#include <application.h>

#define UPDATE_INTERVAL (15 * 60 * 1000)

#define LIS2DH12_UPDATE_INTERVAL 8000

#define GPS_TIMEOUT_INTERVAL (5 * 60 * 1000)

#define GPS_SEND_INTERVAL (15 * 60 * 1000)

#define THRESHOLD 0.04f

#define HORIZONTAL_THRESHOLD 30
#define VERTICAL_THRESHOLD 30

twr_led_t led;

bool sending_gps = false;

twr_lis2dh12_t lis2dh12;
twr_lis2dh12_result_g_t lis2dh12_result;

float last_X = 0.0f;
float last_Y = 0.0f;
float last_Z = 0.0f;

twr_led_t gps_led_r;
twr_led_t gps_led_g;

// Core Module temperature sensor instance
twr_tmp112_t tmp112;

twr_scheduler_task_id_t send_gps_task;
twr_scheduler_task_id_t gps_timeout_task;

void send_gps_coordinates();
void gps_timeout();

void lis2_event_handler(twr_lis2dh12_t *self, twr_lis2dh12_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == TWR_LIS2DH12_EVENT_UPDATE) {
        twr_lis2dh12_get_result_g(&lis2dh12, &lis2dh12_result);

        if(lis2dh12_result.x_axis - last_X > THRESHOLD  ||
           lis2dh12_result.x_axis - last_X < -THRESHOLD ||
           lis2dh12_result.y_axis - last_Y > THRESHOLD  ||
           lis2dh12_result.y_axis - last_Y < -THRESHOLD ||
           lis2dh12_result.z_axis - last_Z > THRESHOLD  ||
           lis2dh12_result.z_axis - last_Z < -THRESHOLD)
        {
            twr_log_debug("ALARM");

            if(!sending_gps)
            {
                twr_log_debug("REPLANING");

                twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL);
            }
        }

        last_X = lis2dh12_result.x_axis;
        last_Y = lis2dh12_result.y_axis;
        last_Z = lis2dh12_result.z_axis;

    } else {
        twr_log_debug("error");
    }
}

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *event_param)
{
    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        float measured_temp;
        if (twr_tmp112_get_temperature_celsius(self, &measured_temp))
        {
            twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &measured_temp);
        }
    }
}

void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    float voltage;
    int percentage;

    if(event == TWR_MODULE_BATTERY_EVENT_UPDATE)
    {
        if (twr_module_battery_get_voltage(&voltage))
        {
            twr_radio_pub_battery(&voltage);
        }
    }
}

void gps_module_event_handler(twr_module_gps_event_t event, void *event_param)
{
    if (event == TWR_MODULE_GPS_EVENT_START)
    {
        twr_log_info("APP: Event TWR_MODULE_GPS_EVENT_START");

        twr_led_set_mode(&gps_led_g, TWR_LED_MODE_ON);
    }

    else if (event == TWR_MODULE_GPS_EVENT_STOP)
    {
        twr_log_info("APP: Event TWR_MODULE_GPS_EVENT_STOP");

        twr_led_set_mode(&gps_led_g, TWR_LED_MODE_OFF);
    }

    else if (event == TWR_MODULE_GPS_EVENT_UPDATE)
    {
        twr_led_pulse(&gps_led_r, 50);

        twr_module_gps_position_t position;

        if (twr_module_gps_get_position(&position))
        {
            twr_log_info("APP: Lat: %03.5f", position.latitude);
            twr_log_info("APP: Lon: %03.5f", position.longitude);
            twr_module_gps_altitude_t altitude;

            if (twr_module_gps_get_altitude(&altitude))
            {
                twr_log_info("APP: Altitude: %.1f %c", altitude.altitude, tolower(altitude.units));
            }

            twr_module_gps_accuracy_t accuracy;

            if(twr_module_gps_get_accuracy(&accuracy))
            {
                twr_log_debug("%.2f", accuracy.horizontal);
                twr_log_debug("%.2f", accuracy.vertical);
            }

            if(accuracy.horizontal < HORIZONTAL_THRESHOLD && accuracy.vertical < VERTICAL_THRESHOLD)
            {
                twr_scheduler_unregister(gps_timeout_task);
                gps_timeout_task = twr_scheduler_register(gps_timeout, NULL, TWR_TICK_INFINITY);

                sending_gps = false;

                twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL);

                twr_module_gps_stop();
                twr_radio_pub_float("latitude", &(position.latitude));
                twr_radio_pub_float("longitude", &(position.longitude));
                twr_radio_pub_float("altitude", &(altitude.altitude));
            }
        }

        twr_module_gps_quality_t quality;

        if (twr_module_gps_get_quality(&quality))
        {
            twr_log_info("APP: Fix quality: %d", quality.fix_quality);
            twr_log_info("APP: Satellites: %d", quality.satellites_tracked);
        }

        twr_module_gps_invalidate();
    }

    else if (event == TWR_MODULE_GPS_EVENT_ERROR)
    {
        twr_log_info("APP: Event TWR_MODULE_GPS_EVENT_ERROR");
    }
}

void gps_timeout()
{
    twr_log_debug("TIMEOUT");

    sending_gps = false;

    twr_module_gps_stop();
    twr_radio_pub_float("latitude", 0);
    twr_radio_pub_float("longitude", 0);
    twr_radio_pub_float("altitude", 0);

    twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL);
}

void send_gps_coordinates()
{
    twr_log_debug("SENDING");
    sending_gps = true;
    twr_module_gps_start();

    twr_scheduler_plan_relative(gps_timeout_task, GPS_TIMEOUT_INTERVAL);
}


void application_init(void)
{
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    twr_lis2dh12_init(&lis2dh12, TWR_I2C_I2C0, 0x19);
    twr_lis2dh12_set_event_handler(&lis2dh12, lis2_event_handler, NULL);
    twr_lis2dh12_set_update_interval(&lis2dh12, LIS2DH12_UPDATE_INTERVAL);

    if (!twr_module_gps_init())
    {
        twr_log_error("APP: GPS Module initialization failed");
    }

    else
    {
        twr_module_gps_set_event_handler(gps_module_event_handler, NULL);
    }

    twr_led_init_virtual(&gps_led_r, TWR_MODULE_GPS_LED_RED, twr_module_gps_get_led_driver(), 0);
    twr_led_init_virtual(&gps_led_g, TWR_MODULE_GPS_LED_GREEN, twr_module_gps_get_led_driver(), 0);

    twr_module_battery_init();
    twr_module_battery_set_event_handler(battery_event_handler, NULL);
    twr_module_battery_set_update_interval(UPDATE_INTERVAL);

    twr_tmp112_init(&tmp112, TWR_I2C_I2C0, 0x49);
    twr_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    twr_tmp112_set_update_interval(&tmp112, UPDATE_INTERVAL);

     // Initialize radio communication
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);
    twr_radio_pairing_request("ice-gps", VERSION);

    twr_led_init(&led, TWR_GPIO_LED, false, 0);
    twr_led_pulse(&led, 2000);

    send_gps_task = twr_scheduler_register(send_gps_coordinates, NULL, GPS_SEND_INTERVAL);
    gps_timeout_task = twr_scheduler_register(gps_timeout, NULL, TWR_TICK_INFINITY);
}


