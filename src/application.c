#include <application.h>

#define UPDATE_INTERVAL (5 * 60 * 1000)

#define TEMPERATURE_UPDATE_INTERVAL (10 * 1000)
#define TMP112_PUB_NO_CHANGE_INTERVAL (5 * 60 * 1000)
#define TMP112_PUB_VALUE_CHANGE 0.5f

#define LIS2DH12_UPDATE_INTERVAL 8000

#define RESET_SEND_SIGNAL_TIME 30 * 1000

//CORRECT VERSION
#define GPS_TIMEOUT_INTERVAL (5 * 60 * 1000)
//TEST VERSION
//#define GPS_TIMEOUT_INTERVAL (1 * 60 * 1000)

//CORRECT VERSION
#define GPS_SEND_INTERVAL_MOVED (15 * 60 * 1000)
//TEST VERSION
//#define GPS_SEND_INTERVAL_MOVED (1 * 60 * 1000)

//CORRECT VERSION
#define GPS_SEND_INTERVAL_IDLE (10080 * 60 * 1000)
//TEST VERSION
//#define GPS_SEND_INTERVAL_IDLE (5 * 60 * 1000)

#define THRESHOLD 0.04f

#define HORIZONTAL_THRESHOLD 30
#define VERTICAL_THRESHOLD 30

twr_led_t led;

bool sending_gps = false;
bool moved = false;

int special_alarm_count = 0;

bool not_sended = true;

twr_lis2dh12_t lis2dh12;
twr_lis2dh12_result_g_t lis2dh12_result;

// alarm settings
twr_lis2dh12_alarm_t alarm;

float last_X = 0.0f;
float last_Y = 0.0f;
float last_Z = 0.0f;

twr_led_t gps_led_r;
twr_led_t gps_led_g;

// Core Module temperature sensor instance
twr_tmp112_t tmp112;
float last_temperature;
twr_tick_t temperature_next_pub;

twr_scheduler_task_id_t send_gps_task;
twr_scheduler_task_id_t gps_timeout_task;

twr_scheduler_task_id_t reset_send_signal_task;

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

            if(sending_gps)
            {
                moved = true;
            }

            if(!sending_gps)
            {
                twr_log_debug("REPLANING");

                twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL_MOVED);
            }
        }

        last_X = lis2dh12_result.x_axis;
        last_Y = lis2dh12_result.y_axis;
        last_Z = lis2dh12_result.z_axis;

    }
    else if (event == TWR_LIS2DH12_EVENT_ALARM) {
        if(special_alarm_count > 6)
        {
            send_measurements();
        }
        else
        {
            special_alarm_count++;
        }
        twr_log_debug("SPECIAL ALARM");
    }
    else {
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
            if ((fabs(measured_temp - last_temperature) >= TMP112_PUB_VALUE_CHANGE) || (temperature_next_pub < twr_scheduler_get_spin_tick()))
            {
                twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &measured_temp);
                last_temperature = measured_temp;
                temperature_next_pub = twr_scheduler_get_spin_tick() + TMP112_PUB_NO_CHANGE_INTERVAL;
            }
        }
    }
}

void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    float voltage;
    //int percentage;

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

                if(moved)
                {
                    moved = false;
                    twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL_MOVED);
                }
                else
                {
                    moved = false;
                    twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL_IDLE);
                }

                twr_module_gps_stop();

                static char pos_buf[40];
                snprintf(pos_buf, sizeof(pos_buf), "[%03.5f, %03.5f, %03.5f]", position.latitude, position.longitude, altitude.altitude);
                static char acc_buf[15];
                snprintf(acc_buf, sizeof(acc_buf), "[%.2f, %.2f]", accuracy.horizontal, accuracy.vertical);

                twr_radio_pub_string("pos", pos_buf);
                twr_radio_pub_string("acc", acc_buf);
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


void send_measurements()
{
    if(not_sended)
    {
        not_sended = false;
        special_alarm_count = 0;
        last_temperature = 0;
        twr_tmp112_measure(&tmp112);
        twr_module_battery_measure();

        twr_scheduler_plan_relative(reset_send_signal_task, RESET_SEND_SIGNAL_TIME);
    }
}

void reset_send_signal()
{
    not_sended = true;
}

void gps_timeout()
{
    twr_log_debug("TIMEOUT");

    sending_gps = false;

    twr_module_gps_stop();
    static char pos_buf[15];
    snprintf(pos_buf, sizeof(pos_buf), "[0, 0, 0]");
    static char acc_buf[10];
    snprintf(acc_buf, sizeof(acc_buf), "[0, 0]");

    twr_radio_pub_string("pos", pos_buf);
    twr_radio_pub_string("acc", acc_buf);

    if(moved)
    {
        moved = false;
        twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL_MOVED);
    }
    else
    {
        moved = false;
        twr_scheduler_plan_relative(send_gps_task, GPS_SEND_INTERVAL_IDLE);
    }
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

    alarm.x_high = true;
    alarm.y_high = true;
    alarm.threshold = 800;

    twr_lis2dh12_init(&lis2dh12, TWR_I2C_I2C0, 0x19);
    twr_lis2dh12_set_event_handler(&lis2dh12, lis2_event_handler, NULL);
    twr_lis2dh12_set_alarm(&lis2dh12, &alarm);
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
    twr_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_INTERVAL);

     // Initialize radio communication
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);
    twr_radio_pairing_request("ice-gps", VERSION);

    twr_led_init(&led, TWR_GPIO_LED, false, 0);
    twr_led_pulse(&led, 2000);

    send_gps_task = twr_scheduler_register(send_gps_coordinates, NULL, GPS_SEND_INTERVAL_MOVED);
    gps_timeout_task = twr_scheduler_register(gps_timeout, NULL, TWR_TICK_INFINITY);
    reset_send_signal_task = twr_scheduler_register(reset_send_signal, NULL, TWR_TICK_INFINITY);
}

