#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

//Defining Pins

//LEDS
#define GREEN_LED GPIO_NUM_10
#define YELLOW_LED GPIO_NUM_11

//BUTTONS
#define DRIVER_OCC GPIO_NUM_5
#define DRIVER_BELT GPIO_NUM_7
#define PASS_OCC GPIO_NUM_4
#define PASS_BELT GPIO_NUM_6
#define IGNITION GPIO_NUM_8

//BUZZER
#define BUZZER GPIO_NUM_47

//DELAY
#define LOOP_DELAY_MS 25

// POTENTIOMETER
#define POT_ADC_CH ADC2_CHANNEL_4
#define POT_ADC_GPIO GPIO_NUM_15

// HEADLIGHTS
#define HEADLIGHT_LED GPIO_NUM_21
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12
#define DELAY_MS        10     
#define DIAL_CHANNEL     ADC_CHANNEL_4
#define SENSOR_CHANNEL   ADC_CHANNEL_2

// LIGHT SENSOR
#define LIGHT_SENSOR GPIO_NUM_13


void delay_ms(int t) { vTaskDelay(t / portTICK_PERIOD_MS); }


void app_main(void) {

adc_oneshot_unit_handle_t adc2_handle;              // Unit handle
        
adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2, 
};
adc_oneshot_new_unit(&init_config2, &adc2_handle);  // Populate unit handle
    
adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN,
    .bitwidth = BITWIDTH
};                                                  // Channel config
adc_oneshot_config_channel(adc2_handle, DIAL_CHANNEL, &config);
adc_oneshot_config_channel(adc2_handle, SENSOR_CHANNEL, &config);

adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_2,
    .chan = DIAL_CHANNEL,
    .atten = ADC_ATTEN,
    .bitwidth = BITWIDTH
};

adc_cali_handle_t adc1_cali_chan_handle = NULL;            // Calibration handle
adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan_handle);
cali_config.chan = SENSOR_CHANNEL;

adc_cali_handle_t adc2_cali_chan_handle = NULL;            // Calibration handle
adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_chan_handle);

//Configuring LEDS
    
    //Configuring GREEN_LED
    gpio_reset_pin(GREEN_LED);
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_pullup_en(GREEN_LED);

    //Configuring YELLOW_LED
    gpio_reset_pin(YELLOW_LED);
    gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
    gpio_pullup_en(YELLOW_LED);  

    //Configure HEADLIGHT_LED
    gpio_reset_pin(HEADLIGHT_LED);
    gpio_set_direction(HEADLIGHT_LED, GPIO_MODE_OUTPUT);
    gpio_pullup_en(HEADLIGHT_LED);

//Configuring Buttons / switches

    //Configuring DRIVER_OCC
    gpio_reset_pin(DRIVER_OCC);
    gpio_set_direction(DRIVER_OCC, GPIO_MODE_INPUT);
    gpio_pullup_en(DRIVER_OCC);

    //Configuring DRIVER_BELT
    gpio_reset_pin(DRIVER_BELT);
    gpio_set_direction(DRIVER_BELT, GPIO_MODE_INPUT);
    gpio_pullup_en(DRIVER_BELT);

    //Configuring PASS_OCC
    gpio_reset_pin(PASS_OCC);
    gpio_set_direction(PASS_OCC, GPIO_MODE_INPUT);
    gpio_pullup_en(PASS_OCC);

    //Configuring PASS_BELT
    gpio_reset_pin(PASS_BELT);
    gpio_set_direction(PASS_BELT, GPIO_MODE_INPUT);
    gpio_pullup_en(PASS_BELT);

    //Configuring IGNITION
    gpio_reset_pin(IGNITION);
    gpio_set_direction(IGNITION, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION);

    //Configuring BUZZER
    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_pullup_en(BUZZER);

    //Configuring light sensor
    gpio_reset_pin(LIGHT_SENSOR);
    gpio_set_direction(LIGHT_SENSOR, GPIO_MODE_INPUT);
    gpio_pullup_en(LIGHT_SENSOR);

    //defining variables
    bool driver_occupied_prev = false;
    bool driver_occupied = false;
    bool driver_belt = false;
    bool pass_occupied = false;
    bool pass_belt = false;
    bool ignition = false;
    bool can_start = false;
    bool ignition_prev = false;
    bool engine_on = false;
    int dial_bits;
    int dial_adc_mV;
    int sensor_bits;
    int sensor_adc_mV;
    int headlights;


    //main loop
    while (1) {

    // Save previous ignition state
    ignition_prev = ignition;
    driver_occupied_prev = driver_occupied;
   
    // Read inputs (active low)
    driver_occupied = gpio_get_level(DRIVER_OCC) == 0;
    driver_belt     = gpio_get_level(DRIVER_BELT) == 0;
    pass_occupied   = gpio_get_level(PASS_OCC) == 0;
    pass_belt       = gpio_get_level(PASS_BELT) == 0;
    ignition        = gpio_get_level(IGNITION) == 0;

    // Can engine start?
    can_start = driver_occupied && driver_belt && pass_occupied && pass_belt;

    //welcome message
    if (driver_occupied && !driver_occupied_prev) { //if driver seat is occupied
        printf("Welcome to enhanced alarm system model 218-W25\n"); //print welcome message
    }

    // Green LED shows readiness (only when engine is off)
    if (!engine_on && can_start) {
        gpio_set_level(GREEN_LED, 1);
    } else {
        gpio_set_level(GREEN_LED, 0);
    }

    if (engine_on){

        adc_oneshot_read(adc2_handle, DIAL_CHANNEL, &dial_bits);
        adc_cali_raw_to_voltage(adc1_cali_chan_handle, dial_bits, &dial_adc_mV);

        adc_oneshot_read(adc2_handle, SENSOR_CHANNEL, &sensor_bits);
        adc_cali_raw_to_voltage(adc2_cali_chan_handle, sensor_bits, &sensor_adc_mV);
        if (dial_adc_mV == 0){
            headlights = 0;
        }
        else if (dial_adc_mV> 3000){
            headlights = 1;
        }
        else {

            if (sensor_adc_mV>1000){
                headlights = 1;
                delay_ms (1000);
            }
            else {
                headlights = 0;
                delay_ms (2000);
            }
        }
        gpio_set_level(HEADLIGHT_LED, headlights);
    }

    // Detect ignition button press (rising edge)
    if (ignition && !ignition_prev) {

        // If engine is OFF → try to start
        if (!engine_on) {
            if (can_start) {
                engine_on = true;
                gpio_set_level(YELLOW_LED, 1);
                gpio_set_level(BUZZER, 0);
                printf("Engine Started\n");
                
            } else {
                gpio_set_level(BUZZER, 1);
                printf("Ignition Inhibited\n");

                if (!driver_occupied) printf("Driver seat not occupied\n");
                if (!driver_belt)     printf("Driver seatbelt not fastened\n");
                if (!pass_occupied)   printf("Passenger seat not occupied\n");
                if (!pass_belt)       printf("Passenger seatbelt not fastened\n");
            }

        }
        // When engine shuts off, turn off headlights and engine running light (yellow)
        else {
            headlights = 0;
            gpio_set_level(HEADLIGHT_LED, headlights);
            engine_on = false;
            gpio_set_level(YELLOW_LED, 0);
            gpio_set_level(BUZZER, 0);
            printf("Engine Stopped\n");
        }
    }

    delay_ms (LOOP_DELAY_MS);


}

}