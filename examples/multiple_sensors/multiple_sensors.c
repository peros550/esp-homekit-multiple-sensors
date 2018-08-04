#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "wifi_config.h"

#include <dht/dht.h>

#include "button.h"

// The GPIO pin that is connected to the LED on the ESP Device. On Wemos D1 mini it is pin 2 (D4), on Sonoff it is pin 13. 
const int led_gpio = 2;
// The GPIO pin that is oconnected to the button on the ESP Device. On Wemos D1 mini I selected Pin 4 (D2) where I connected a button. On Sonoff the on board button is on pin 0. 
const int button_gpio = 4; 
//The GPIO pin that motion sensor is connected. 
const int motion_sensor_gpio = 14;


bool led_value = false; //This is to keep track of the LED status. 
homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
//Additional for Motion
homekit_characteristic_t motion_detected  = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, 0);
homekit_characteristic_t currentAmbientLightLevel = HOMEKIT_CHARACTERISTIC_(CURRENT_AMBIENT_LIGHT_LEVEL, 0,.min_value = (float[]) {0},);
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Sonoff Switch");


#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif

void identify_task(void *_args) {
    vTaskDelete(NULL);
}

void identify(homekit_value_t _value) {
    printf("identify\n");
    xTaskCreate(identify_task, "identify", 128, NULL, 2, NULL);
}

void button_callback(uint8_t gpio, button_event_t event);

void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
	led_value = on; //Keep track of the status
}


void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    printf("Resetting Wifi Config\n");
    
    wifi_config_reset();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("Resetting HomeKit Config\n");
    
    homekit_server_reset();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("Restarting\n");
    
    sdk_system_restart();
    
    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting Sonoff configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}







void motion_sensor_callback(uint8_t gpio) {

    if (gpio == motion_sensor_gpio){
		
		  uint16_t val = sdk_system_adc_read();
   			//float val2 = (3.2 / 1023.0);
			//val2 = val * val2;
				//printf ("ADC voltage is %f\n",  val2);
				printf ("ADC voltage is %d\n",  val);
		//printf ("ADC voltage is %.3f\n", sdk_system_adc_read());
		//currentAmbientLightLevel = val;
		
		//homekit_characteristic_notify(&currentAmbientLightLevel, HOMEKIT_FLOAT(val));
	
        int new = 0;
        new = gpio_read(motion_sensor_gpio);
        motion_detected.value = HOMEKIT_BOOL(new);

	led_write(new);


        homekit_characteristic_notify(&motion_detected, HOMEKIT_BOOL(new));
        printf("Motion Detected on %d\n", gpio);
    }
    else {
        printf("Interrupt on %d", gpio);

    }

}


void button_callback(uint8_t gpio, button_event_t event) {
    switch (event) {
        case button_event_single_press:
           printf("Toggling relay\n");
led_write(!led_value); //A Single press will just toggle the LED. This was left for testing purposes. 
            //The following code is left commented in case we want to implement a single button for something else in homekit. 
			//switch_on.value.bool_value = !switch_on.value.bool_value;
            //relay_write(switch_on.value.bool_value);
            //homekit_characteristic_notify(&switch_on, switch_on.value);
			
			  
		
			
            break;
        case button_event_long_press:
	printf("Resetting Wifi\n");
            reset_configuration();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}

void temperature_sensor_identify(homekit_value_t _value) {
    printf("Temperature sensor identify\n");
}

void light_sensor_identify(homekit_value_t _value) {
    printf("Light sensor identify\n");
}

void temperature_sensor_task(void *_args) {
    gpio_set_pullup(SENSOR_PIN, false, false);

    float humidity_value, temperature_value;
    while (1) {
        bool success = dht_read_float_data(
            DHT_TYPE_DHT22, SENSOR_PIN,
            &humidity_value, &temperature_value
        );
        if (success) {
            temperature.value.float_value = temperature_value;
            humidity.value.float_value = humidity_value;

            homekit_characteristic_notify(&temperature, HOMEKIT_FLOAT(temperature_value));
            homekit_characteristic_notify(&humidity, HOMEKIT_FLOAT(humidity_value));
        } else {
            printf("Couldnt read data from sensor\n");
        }

        vTaskDelay(6000 / portTICK_PERIOD_MS);
    }
}
void temperature_sensor_init() {
    xTaskCreate(temperature_sensor_task, "Temperature Sensor", 256, NULL, 2, NULL);
}

void light_sensor_task(void *_args) {
    uint16_t analog_light_value;
    while (1) {
         analog_light_value = sdk_system_adc_read();
		 //The below code does not produce accurate LUX readings which is what homekit expects. It only provides an indication of brightness on a scale between 0 to 1024
		//More work needs to be done so that accurate conversation to LUX scale can take place. However this is strongly dependent on the type of sensor used. 
		//In my case I used a Photodiode Light Sensor 
            currentAmbientLightLevel.value.float_value = (1024 - analog_light_value);
            homekit_characteristic_notify(&currentAmbientLightLevel, HOMEKIT_FLOAT((1024 - analog_light_value)));
            vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
void light_sensor_init() {
    xTaskCreate(light_sensor_task, "Light Sensor", 256, NULL, 2, NULL);
}



homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            &name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "YP"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "0012345"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MultipleSensors"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            &temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
            &humidity,
            NULL
        }),
        NULL
    }),
HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Motion Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "YP"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "0012346"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MotionSensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
            NULL
        }),
        HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Motion Sensor"),
            &motion_detected,
            NULL
        }),
        NULL
    }),
	HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Light Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "YP"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "0012347"),
            HOMEKIT_CHARACTERISTIC(MODEL, "LightSensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
			HOMEKIT_CHARACTERISTIC(IDENTIFY, light_sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LIGHT_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Light Sensor"),
            &currentAmbientLightLevel,
            NULL
        }),
        NULL
    }),
    NULL
};



void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);
    
    int name_len = snprintf(NULL, 0, "Homekit Sensor-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Homekit Sensor-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);
    
    name.value = HOMEKIT_STRING(name_value);
}

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}


void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(true);
//gpio_enable(button_gpio, GPIO_INPUT);

	gpio_enable(motion_sensor_gpio, GPIO_INPUT);
    gpio_set_pullup(motion_sensor_gpio, false, false);
    gpio_set_interrupt(motion_sensor_gpio, GPIO_INTTYPE_EDGE_ANY, motion_sensor_callback); 
    
}


void user_init(void) {
    uart_set_baud(0, 115200);
    wifi_config_init("Homekit-sensor", NULL, on_wifi_ready);
	gpio_init();
	create_accessory_name();
    temperature_sensor_init();
	light_sensor_init();

 if (button_create(button_gpio, 0, 4000, button_callback)) {
        printf("Failed to initialize button\n");
    }
  
}




