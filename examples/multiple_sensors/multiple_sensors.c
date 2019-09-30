#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "wifi_config.h"

#include <dht/dht.h>
#include <adv_button.h>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
const int led_gpio = 2; // Pin D4
const int button_gpio = 4; //Pin (D2) 
const int motion_sensor_gpio = 16; // Wemos D1 mini pin: D0 
const int SENSOR_PIN = 5; //DHT sensor on pin D1
const int IR_PIN = 14 ; // Wemos D1 mini pin: D5. 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool led_value = false; //This is to keep track of the LED status. 
volatile float old_humidity_value = 0.0, old_temperature_value = 0.0 , old_light_value =0.0, old_move_value = 0.0;
ETSTimer extra_func_timer ;

homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
//Additional for Motion
homekit_characteristic_t motion_detected  = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, 0);
homekit_characteristic_t currentAmbientLightLevel = HOMEKIT_CHARACTERISTIC_(CURRENT_AMBIENT_LIGHT_LEVEL, 0,.min_value = (float[]) {0},);
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Sonoff Switch");




void identify_task(void *_args) {
    vTaskDelete(NULL);
}

void identify(homekit_value_t _value) {
    printf("identify\n");
    xTaskCreate(identify_task, "identify", 128, NULL, 2, NULL);
}

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


void sensor_worker() {
//Temperature measurement
   	float humidity_value, temperature_value;
    bool get_temp = false;
     
        get_temp = dht_read_float_data(DHT_TYPE_DHT22, SENSOR_PIN, &humidity_value, &temperature_value);
        if (get_temp) {
       // printf("RC >>> Sensor: temperature %g, humidity %g\n", temperature_value, humidity_value);
        
			if (temperature_value != old_temperature_value) {
				old_temperature_value = temperature_value;
				
			//	current_temperature.value = HOMEKIT_FLOAT(temperature_value); //Update AC current temp
			//	homekit_characteristic_notify(&current_temperature, HOMEKIT_FLOAT(temperature_value));
				
				temperature.value.float_value = temperature_value;
				homekit_characteristic_notify(&temperature, HOMEKIT_FLOAT(temperature_value));
			}
				
	
			if (humidity_value != old_humidity_value) {
				old_humidity_value = humidity_value;
				humidity.value.float_value =humidity_value;
				//current_humidity.value = HOMEKIT_FLOAT(humidity_value);
				//homekit_characteristic_notify(&current_humidity, current_humidity.value);
				 homekit_characteristic_notify(&humidity, HOMEKIT_FLOAT(humidity_value));
			}
				
			
			
		} else 
		{
        printf("RC !!! ERROR Sensor\n");
        //led_code(LED_GPIO, SENSOR_ERROR);
        
		}

	//printf("Checking Light\n");
	//Light measurement
	float light_value;
	light_value = sdk_system_adc_read();
	 	 if (light_value != old_light_value) 
		 {
		 old_light_value = light_value;
		 currentAmbientLightLevel.value.float_value = (1024 - light_value);
		 homekit_characteristic_notify(&currentAmbientLightLevel, HOMEKIT_FLOAT((1024 - light_value))); 
		}
		
		uint32_t freeheap = xPortGetFreeHeapSize();
    //printf("xPortGetFreeHeapSize = %d bytes\n", freeheap);
	 

}



void movement_detected_fn(const uint8_t gpio) {
	printf("Movement detected\n");
	old_move_value = true;
	motion_detected.value = HOMEKIT_BOOL(old_move_value);
	led_write(old_move_value);
	homekit_characteristic_notify(&motion_detected, HOMEKIT_BOOL(old_move_value));
	
}

void movement_not_detected_fn(const uint8_t gpio) {
	printf("Movement not detected\n");

	old_move_value = false;
	motion_detected.value = HOMEKIT_BOOL(old_move_value);
	led_write(old_move_value);
	homekit_characteristic_notify(&motion_detected, HOMEKIT_BOOL(old_move_value));
}


void button_callback_single(const uint8_t gpio) {

	printf("Toggling relay\n");
	led_write(!led_value); //A Single press will just toggle the LED. This was left for testing purposes. 
	//The following code is left commented in case we want to implement a single button for something else in homekit. 
	//switch_on.value.bool_value = !switch_on.value.bool_value;
	//relay_write(switch_on.value.bool_value);
	//homekit_characteristic_notify(&switch_on, switch_on.value);

	//http_get_task(1);
	// xTaskCreate(http_get_task, "http", 256, NULL, 1, NULL);
	//vTaskDelay(100 / portTICK_PERIOD_MS);
			
}


void button_hold_callback(const uint8_t gpio) {
	printf("Resetting Wifi\n");
    reset_configuration();
           
}


void temperature_sensor_identify(homekit_value_t _value) {
    printf("Temperature sensor identify\n");
}

void light_sensor_identify(homekit_value_t _value) {
    printf("Light sensor identify\n");
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

// void on_wifi_ready() {
//     homekit_server_init(&config);

//     sdk_os_timer_setfn(&extra_func_timer, sensor_worker, NULL);
// 	sdk_os_timer_disarm(&extra_func_timer);
//     sdk_os_timer_arm(&extra_func_timer, 10000, 1);
// }

void on_wifi_event(wifi_config_event_t event) {
    if (event == WIFI_CONFIG_CONNECTED) {
        printf("Connected to WiFi\n");

    homekit_server_init(&config);

    sdk_os_timer_setfn(&extra_func_timer, sensor_worker, NULL);
	sdk_os_timer_disarm(&extra_func_timer);
    sdk_os_timer_arm(&extra_func_timer, 10000, 1);

    } else if (event == WIFI_CONFIG_DISCONNECTED) {
        printf("Disconnected from WiFi\n");
    }
}


void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(true);


    adv_toggle_create(motion_sensor_gpio, false);  // false -> without internal pull-up resistor
    adv_toggle_register_callback_fn(motion_sensor_gpio, movement_detected_fn, 1);    // High gpio state
    adv_toggle_register_callback_fn(motion_sensor_gpio, movement_not_detected_fn, 0);    // Low gpio state
    

    adv_button_create(button_gpio, true);
    adv_button_register_callback_fn(button_gpio, button_callback_single, 1);
    adv_button_register_callback_fn(button_gpio, button_hold_callback, 5);	

}


void user_init(void) {
    uart_set_baud(0, 115200);
   // wifi_config_init("Homekit-sensor", NULL, on_wifi_ready);
    wifi_config_init2("Homekit-sensor", NULL, on_wifi_event);
	gpio_init();
	create_accessory_name();
 
}




