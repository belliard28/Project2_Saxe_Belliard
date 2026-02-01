#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

// PIN DEFINITIONS

// LEDs
#define GREEN_LED GPIO_NUM_10          // Indicates system ready to start
#define YELLOW_LED GPIO_NUM_11         // Indicates engine running

// Buttons / switches
#define DRIVER_OCC GPIO_NUM_5          // Driver seat occupancy
#define DRIVER_BELT GPIO_NUM_7         // Driver seatbelt
#define PASS_OCC GPIO_NUM_4            // Passenger seat occupanc
#define PASS_BELT GPIO_NUM_6           // Passenger seatbelt
#define IGNITION GPIO_NUM_8             // Ignition button

// Buzzer
#define BUZZER GPIO_NUM_47              // Buzzer (car alarm)

// Loop timing
#define LOOP_DELAY_MS 25                // Main loop delay

// Potentiometer / ADC inputs
#define POT_ADC_CH ADC2_CHANNEL_4
#define POT_ADC_GPIO GPIO_NUM_15

// Headlights
#define HEADLIGHT_LED GPIO_NUM_21       // Headlight output LED
#define ADC_ATTEN       ADC_ATTEN_DB_12 // ADC attenuation (0–3.3V range)
#define BITWIDTH        ADC_BITWIDTH_12 // 12-bit ADC resolution
#define DELAY_MS        10     

#define DIAL_CHANNEL     ADC_CHANNEL_4 // Headlight control dial
#define SENSOR_CHANNEL   ADC_CHANNEL_2 // Light sensor input

// Light sensor digital input
#define LIGHT_SENSOR GPIO_NUM_13

// Millisecond delay wrapper for FreeRTOS
void delay_ms(int t) { 
    vTaskDelay(t / portTICK_PERIOD_MS); 
}

void app_main(void) {

    // ADC SETUP

    adc_oneshot_unit_handle_t adc2_handle;

    // Initialize ADC unit 2
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };
    adc_oneshot_new_unit(&init_config2, &adc2_handle);

    // Configure ADC channel parameters
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };

    // Configure both ADC channels
    adc_oneshot_config_channel(adc2_handle, DIAL_CHANNEL, &config);
    adc_oneshot_config_channel(adc2_handle, SENSOR_CHANNEL, &config);

    // ADC calibration configuration (curve fitting)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .chan = DIAL_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };

    // Calibration handle for dial channel
    adc_cali_handle_t adc1_cali_chan_handle = NULL;
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan_handle);

    // Calibration handle for sensor channel
    cali_config.chan = SENSOR_CHANNEL;
    adc_cali_handle_t adc2_cali_chan_handle = NULL;
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_chan_handle);

    // GPIO CONFIGURATION

    // Output LEDs
    gpio_reset_pin(GREEN_LED);
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);

    gpio_reset_pin(YELLOW_LED);
    gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);

    gpio_reset_pin(HEADLIGHT_LED);
    gpio_set_direction(HEADLIGHT_LED, GPIO_MODE_OUTPUT);

    // Input switches (active LOW, pull-ups enabled)
    gpio_reset_pin(DRIVER_OCC);
    gpio_set_direction(DRIVER_OCC, GPIO_MODE_INPUT);
    gpio_pullup_en(DRIVER_OCC);

    gpio_reset_pin(DRIVER_BELT);
    gpio_set_direction(DRIVER_BELT, GPIO_MODE_INPUT);
    gpio_pullup_en(DRIVER_BELT);

    gpio_reset_pin(PASS_OCC);
    gpio_set_direction(PASS_OCC, GPIO_MODE_INPUT);
    gpio_pullup_en(PASS_OCC);

    gpio_reset_pin(PASS_BELT);
    gpio_set_direction(PASS_BELT, GPIO_MODE_INPUT);
    gpio_pullup_en(PASS_BELT);

    gpio_reset_pin(IGNITION);
    gpio_set_direction(IGNITION, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION);

    // Buzzer output
    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);

    // Light sensor input
    gpio_reset_pin(LIGHT_SENSOR);
    gpio_set_direction(LIGHT_SENSOR, GPIO_MODE_INPUT);
    gpio_pullup_en(LIGHT_SENSOR);

    // STATE VARIABLES

    bool driver_occupied_prev = false;  // Previous driver occupancy state
    bool driver_occupied = false;
    bool driver_belt = false;
    bool pass_occupied = false;
    bool pass_belt = false;

    bool ignition = false;              // Current ignition button state
    bool ignition_prev = false;         // Previous ignition state

    bool can_start = false;             // All safety conditions satisfied
    bool engine_on = false;             // Engine running state

    int dial_bits;                      // Raw ADC reading (dial)
    int dial_adc_mV;                    // Dial voltage (mV)
    int sensor_bits;                    // Raw ADC reading (light sensor)
    int sensor_adc_mV;                  // Sensor voltage (mV)

    int headlights = 0;                 // Headlight output state

    // MAIN LOOP
    while (1) {

        // Save previous states for edge detection
        ignition_prev = ignition;
        driver_occupied_prev = driver_occupied;

        // Read all inputs (active LOW)
        driver_occupied = gpio_get_level(DRIVER_OCC) == 0;
        driver_belt     = gpio_get_level(DRIVER_BELT) == 0;
        pass_occupied   = gpio_get_level(PASS_OCC) == 0;
        pass_belt       = gpio_get_level(PASS_BELT) == 0;
        ignition        = gpio_get_level(IGNITION) == 0;

        // Determine if engine is allowed to start
        can_start = driver_occupied && driver_belt &&
                    pass_occupied && pass_belt;

        // Print welcome message when driver sits down
        if (driver_occupied && !driver_occupied_prev) {
            printf("Welcome to enhanced alarm system model 218-W25\n");
        }

        // Green LED indicates system ready (only when engine off)
        gpio_set_level(GREEN_LED, (!engine_on && can_start));

        // HEADLIGHT CONTROL
        if (engine_on) {

            // Read dial voltage
            adc_oneshot_read(adc2_handle, DIAL_CHANNEL, &dial_bits);
            adc_cali_raw_to_voltage(adc1_cali_chan_handle,
                                    dial_bits, &dial_adc_mV);

            // Read light sensor voltage
            adc_oneshot_read(adc2_handle, SENSOR_CHANNEL, &sensor_bits);
            adc_cali_raw_to_voltage(adc2_cali_chan_handle,
                                    sensor_bits, &sensor_adc_mV);

            // Dial OFF position
            if (dial_adc_mV == 0) {
                headlights = 0;
            }
            // Dial forced ON position
            else if (dial_adc_mV > 3000) {
                headlights = 1;
            }
            // Automatic mode using light sensor
            else {
                if (sensor_adc_mV > 1000) {
                    headlights = 1;
                    delay_ms(1000);
                } else {
                    headlights = 0;
                    delay_ms(2000);
                }
            }

            gpio_set_level(HEADLIGHT_LED, headlights);
        }

        // IGNITION CONTROL
        // Detect rising edge of ignition button
        if (ignition && !ignition_prev) {

            // Attempt to start engine
            if (!engine_on) {
                if (can_start) {
                    engine_on = true;
                    gpio_set_level(YELLOW_LED, 1);
                    gpio_set_level(BUZZER, 0);
                    printf("Engine Started\n");
                } else {
                    // Safety violation (inhibit ignition)
                    gpio_set_level(BUZZER, 1);
                    printf("Ignition Inhibited\n");

                    if (!driver_occupied) printf("Driver seat not occupied\n");
                    if (!driver_belt)     printf("Driver seatbelt not fastened\n");
                    if (!pass_occupied)   printf("Passenger seat not occupied\n");
                    if (!pass_belt)       printf("Passenger seatbelt not fastened\n");
                }
            }
            // Turn engine OFF
            else {
                headlights = 0;
                gpio_set_level(HEADLIGHT_LED, 0);
                engine_on = false;
                gpio_set_level(YELLOW_LED, 0);
                gpio_set_level(BUZZER, 0);
                printf("Engine Stopped\n");
            }
        }

        // Main loop pacing delay
        delay_ms(LOOP_DELAY_MS);
    }
}
