//#include <stdio.h>
//#include <string.h>
#include <esp/uart.h>
//#include <rboot-api.h>
#include <sysparam.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>
#include <led_codes.h>
#include <adv_button.h>

#include <dht/dht.h>
#include <ds18b20/ds18b20.h>

//#include "../common/custom_characteristics.h"
#include <ir/ir.h>
#include <ir/raw.h>
//#include "http.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <unistd.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define	USE_THINGSPEAK 0 //Turn this into '1' if you want to use log temperature/humidity data at ThingsSpeak.com server
#define WEB_SERVER "api.thingspeak.com"
#define API_KEY  "xxxxxxxxxxxxx" //Private API key. This is personal and can been  obtained by ThingsSpeak.com
#define FIELD1 "field1=" //temp
#define FIELD2 "field2=" //hum
#define WEB_PORT "80"
#define WEB_PATH "/update?api_key="
#define THINGSPEAK_INTERVAL 600000 //600000 = 10min , 300000 = 5min 60000 = 1min

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
const int led_gpio = 2; // Pin D4
const int button_gpio = 4; //Pin (D2) 
const int motion_sensor_gpio = 16; // Wemos D1 mini pin: D0 
const int SENSOR_PIN = 5; //DHT sensor on pin D1
const int IR_PIN = 14 ; // Wemos D1 mini pin: D5. (Just for Reference)
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



volatile float old_humidity_value = 0.0, old_temperature_value = 0.0 , old_light_value =0.0, old_move_value = 0.0;
volatile int current_ac_state = 0;
volatile bool paired = false;
bool led_value = false; //This is to keep track of the LED status. 
volatile bool Wifi_Connected = false; 

ETSTimer extra_func_timer ;

void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_state();
}

void on_temp_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_temp();
}

void on_active_change(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_active();
}


homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
//homekit_characteristic_t target_temperature  = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE, 22, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_temp_update));
//Additional for Motion
homekit_characteristic_t motion_detected  = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, 0);
homekit_characteristic_t currentAmbientLightLevel = HOMEKIT_CHARACTERISTIC_(CURRENT_AMBIENT_LIGHT_LEVEL, 0,.min_value = (float[]) {0},);
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Sonoff Switch");

//AC Required Parameters
homekit_characteristic_t active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0,.callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_active_change));
homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t current_heater_cooler_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATER_COOLER_STATE, 0);
homekit_characteristic_t target_heater_cooler_state = HOMEKIT_CHARACTERISTIC_(TARGET_HEATER_COOLER_STATE, 0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));

//optionals
//homekit_characteristic_t units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);
homekit_characteristic_t cooling_threshold = HOMEKIT_CHARACTERISTIC_(COOLING_THRESHOLD_TEMPERATURE, 18,.callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_temp_update),.min_value = (float[]) {16},.max_value = (float[]) {30},.min_step = (float[]) {1} );
homekit_characteristic_t heating_threshold = HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 22,.callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_temp_update),.min_value = (float[]) {16},.max_value = (float[]) {30},.min_step = (float[]) {1});
//homekit_characteristic_t rotation_speed = HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void update_state() {
	//No logic yet. Just trying to make it work. 
	uint8_t state = target_heater_cooler_state.value.int_value;
	//printf("Running update_state() \n" );
	//printf("New Target Mode on %d\n", state );

	current_heater_cooler_state.value = HOMEKIT_UINT8(state+1);
	homekit_characteristic_notify(&current_heater_cooler_state, current_heater_cooler_state.value);
	
	uint8_t active_state = active.value.int_value;
	if (active_state == 0) {
	//	printf("OFF\n" );
	//	ac_button_off();
	}
	else
	{	
		//printf("AC is now ON, updating temp\n" );
		update_temp();
	} 

	               
}
void update_temp() {
		//printf("Running update_temp() \n" );
	
	uint8_t state = active.value.int_value;
	uint8_t target_state = target_heater_cooler_state.value.int_value;
	float target_temp1 = 0;
 
	if (target_state == 1) {
	 //Read the Heat target
		target_temp1= heating_threshold.value.float_value;
	}
	else
	{
	 //Else read the Cool target
		target_temp1= cooling_threshold.value.float_value; 
	}
  
	//printf("Target State: %d\n",target_state );
	//printf("Current State: %d\n",state );
	//printf("Target temp: %f\n",target_temp1 );
	
	target_temp1 = (int)target_temp1;

 
	if(state == 1 && target_state == 2){
	//Cool mode

		switch ((int)target_temp1)
		{
			
			case 16:
				//printf("New Cooling Temp: %d\n", 16 );
				ac_button_cool_16();
				break;
			case 17:
				//printf("New Cooling Temp: %d\n", 17 );
				ac_button_cool_17();
				break;
			case 18:
				//printf("New Cooling Temp: %d\n", 18 );
				ac_button_cool_18();
				break;
			case 19:
				//printf("New Cooling Temp: %d\n", 19 );
				ac_button_cool_19();
				break;
			case 20:
				//printf("New Cooling Temp: %d\n", 20 );
				ac_button_cool_20();
				break;
			case 21:
				//printf("New Cooling Temp: %d\n", 21 );
				ac_button_cool_21();
				break;
			case 22:
				//printf("New Cooling Temp: %d\n", 22 );
				ac_button_cool_22();
				break;
			case 23:
				//printf("New Cooling Temp: %d\n", 23 );
				ac_button_cool_23();
				break;
			case 24:
				//printf("New Cooling Temp: %d\n", 24 );
				ac_button_cool_24();
				break;
			case 25:
				//printf("New Cooling Temp: %d\n", 25 );
				ac_button_cool_25();
				break;
			case 26:
				//printf("New Cooling Temp: %d\n", 26 );
				ac_button_cool_26();
				break;
			case 27:
				//printf("New Cooling Temp: %d\n", 27 );
				ac_button_cool_27();
				break;
			case 28:
				//printf("New Cooling Temp: %d\n", 28 );
				ac_button_cool_28();
				break;
			case 29:
				//printf("New Cooling Temp: %d\n", 29 );
				ac_button_cool_29();
				break;
			case 30:
				//printf("New Cooling Temp: %d\n", 30 );
				ac_button_cool_30();
				break;	

		default:
				printf("No action \n" );
		} 
		
	}
 	if(state == 1 && target_state == 1){
		//Heat mode
		 switch ((int)target_temp1){
			case 16:
				//printf("New Heating Temp: %d\n", 16 );
				ac_button_heat_16();
				break;
			case 17:
				//printf("New Heating Temp: %d\n", 17 );
				ac_button_heat_17();
				break;
			case 18:
				//printf("New Heating Temp: %d\n", 18 );
				ac_button_heat_18();
				break;
			case 19:
				//printf("New Heating Temp: %d\n", 19 );
				ac_button_heat_19();
				break;
			case 20:
				//printf("New Heating Temp: %d\n", 20 );
				ac_button_heat_20();
				break;
			case 21:
				//printf("New Heating Temp: %d\n", 21 );
				ac_button_heat_21();
				break;
			case 22:
				//printf("New Heating Temp: %d\n", 22 );
				ac_button_heat_22();
				break;
			case 23:
				//printf("New Heating Temp: %d\n", 23 );
				ac_button_heat_23();
				break;
			case 24:
				//printf("New Heating Temp: %d\n", 24 );
				ac_button_heat_24();
				break;
			case 25:
				//printf("New Heating Temp: %d\n", 25 );
				ac_button_heat_25();
				break;
			case 26:
				//printf("New Heating Temp: %d\n", 26 );
				ac_button_heat_26();
				break;
			case 27:
				//printf("New Heating Temp: %d\n", 27 );
				ac_button_heat_27();
				break;
			case 28:
				//printf("New Heating Temp: %d\n", 28 );
				ac_button_heat_28();
				break;
			case 29:
				//printf("New Heating Temp: %d\n", 29 );
				ac_button_heat_29();
				break;
			case 30:
				//printf("New Heating Temp: %d\n", 30 );
				ac_button_heat_30();
				led_code(led_gpio, FUNCTION_C);
				break;	

		default:
				printf("No action \n" );
		}

	} 
	
	if(state == 1 && target_state == 0){
		//printf("Fan mode\n", 30 );
		ac_button_aut();
	}
}
void update_active() {
	//printf("Running update_active() \n" );
	uint8_t state = active.value.int_value;
	//printf("New Active status on %d\n", state );
	if (current_ac_state != state && state ==1)
	{
	update_temp();	
	}
	
	
	if (state == 0) {
	//	printf("OFF\n" );
	ac_button_off();
	led_code(led_gpio, FUNCTION_C);
	}
	else
	{	
		//printf("AC is now ON, updating temp\n" );
		//update_temp();
	} 
	current_ac_state = state;
	// printf("Changed active status\n");
            
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void call_things_process()
{
	int successes = 0, failures = 0;
    printf("HTTP get task starting...\r\n");
while(1) {
 
if (Wifi_Connected == true)
{

			const struct addrinfo hints = {
				.ai_family = AF_UNSPEC,
				.ai_socktype = SOCK_STREAM,
			};
			struct addrinfo *res;

			printf("Running DNS lookup for %s...\r\n", WEB_SERVER);
			int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

			if (err != 0 || res == NULL) {
				printf("DNS lookup failed err=%d res=%p\r\n", err, res);
				if(res)
				freeaddrinfo(res);
				
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				failures++;
				continue;
			}

	#if LWIP_IPV6
			{
				struct netif *netif = sdk_system_get_netif(0);
				int i;
				for (i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
					printf("  ip6 %d state %x\n", i, netif_ip6_addr_state(netif, i));
					if (!ip6_addr_isinvalid(netif_ip6_addr_state(netif, i)))
						printf("  ip6 addr %d = %s\n", i, ip6addr_ntoa(netif_ip6_addr(netif, i)));
				}
			}
	#endif

			struct sockaddr *sa = res->ai_addr;
			if (sa->sa_family == AF_INET) {
				printf("DNS lookup succeeded. IP=%s\r\n", inet_ntoa(((struct sockaddr_in *)sa)->sin_addr));
			}
	#if LWIP_IPV6
			if (sa->sa_family == AF_INET6) {
				printf("DNS lookup succeeded. IP=%s\r\n", inet6_ntoa(((struct sockaddr_in6 *)sa)->sin6_addr));
			}
	#endif

			int s = socket(res->ai_family, res->ai_socktype, 0);
			if(s < 0) {
				printf("... Failed to allocate socket.\r\n");
				freeaddrinfo(res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
				failures++;
				continue;
			}

			printf("... allocated socket\r\n");

			if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
				close(s);
				freeaddrinfo(res);
				printf("... socket connect failed.\r\n");
				vTaskDelay(4000 / portTICK_PERIOD_MS);
				failures++;
				continue;
			}

			printf("... connected\r\n");
			freeaddrinfo(res);
			
			char temp2[10];
			snprintf(temp2, sizeof(temp2),"%2.2f", old_temperature_value );
			printf("%s \n",temp2);
			printf(temp2);
			
			char hum1[6];
			snprintf(hum1,sizeof(hum1), "%2.2f", old_humidity_value );
			printf("%s \n",hum1 );
			

			char req[300]="GET " WEB_PATH API_KEY "&" FIELD1;
			strncat(req,temp2,10);
			strncat(req,"&",2);
			strncat(req,FIELD2,20);
			strncat(req,hum1,6);
			strncat(req," HTTP/1.1\r\nHost: "WEB_SERVER"\r\nUser-Agent: esp-open-rtos/0.1 esp8266\r\nConnection: close\r\n\r\n",250);
			
			
			printf("%s \n",req );
			

			
			if (old_temperature_value ==0) 
			{
				vTaskDelay(4000 / portTICK_PERIOD_MS);
				failures++;
				continue;
			}

			
			// printf("%s \n",req );
			
			if (write(s, req, strlen(req)) < 0) {
				printf("... socket send failed\r\n");
				close(s);
				vTaskDelay(4000 / portTICK_PERIOD_MS);
				failures++;
				continue;
			}
			printf("... socket send success\r\n");

			static char recv_buf[128];
			int r;
			do {
				bzero(recv_buf, 128);
				r = read(s, recv_buf, 127);
				if(r > 0) {
					printf("%s", recv_buf);
				}
			} while(r > 0);

			printf("... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
		
		
			if(r != 0)
				failures++;
			else
				successes++;
			close(s);
			printf("successes = %d failures = %d\r\n", successes, failures);
			printf("\r\nEnding!\r\n");
			vTaskDelay(THINGSPEAK_INTERVAL / portTICK_PERIOD_MS);
	}
	else
	{
		vTaskDelay(60000 / portTICK_PERIOD_MS);
	}

}	    
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void identify_task(void *_args) {
    vTaskDelete(NULL);
}

void identify(homekit_value_t _value) {
    printf("identify\n");
    xTaskCreate(identify_task, "identify", 128, NULL, 2, NULL);
	vTaskDelay(100 / portTICK_PERIOD_MS);
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
	vTaskDelay(100 / portTICK_PERIOD_MS);
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
				
				current_temperature.value = HOMEKIT_FLOAT(temperature_value); //Update AC current temp
				homekit_characteristic_notify(&current_temperature, HOMEKIT_FLOAT(temperature_value));
				
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
	 HOMEKIT_ACCESSORY(.id=4, .category=homekit_accessory_category_air_conditioner , .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "AIR CONDITIONER"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "001"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MyThermostat"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
            NULL
        }),
        HOMEKIT_SERVICE(HEATER_COOLER, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "AirConditioner"),
            &active,
			&current_temperature,
			//&target_temperature,
            &current_heater_cooler_state,
            &target_heater_cooler_state,
           &cooling_threshold,
            &heating_threshold,
//            &units,
//            &rotation_speed,
            NULL
        }),
        NULL
    }),
    NULL
};


void on_event(homekit_event_t event) {
    if (event == HOMEKIT_EVENT_SERVER_INITIALIZED) {
        //led_status_set(led_status, paired ? &normal_mode : &unpaired);
		 printf("SERVER JUST INITIALIZED\n");
		 
		 if (homekit_is_paired()){
			printf("Found pairing, starting timers\n");
			sdk_os_timer_arm(&extra_func_timer, 10000, 1);

			if (USE_THINGSPEAK == 1){
			//Start the ThingsSpeak process only if relevant flag is set
				xTaskCreate(call_things_process, "http", 1024, NULL, 1, NULL);
			}
	

			adv_toggle_create(motion_sensor_gpio, false);  // false -> without internal pull-up resistor
			adv_toggle_register_callback_fn(motion_sensor_gpio, movement_detected_fn, 1);    // High gpio state
			adv_toggle_register_callback_fn(motion_sensor_gpio, movement_not_detected_fn, 0);    // Low gpio state
			
			
			adv_button_create(button_gpio, true);
			adv_button_register_callback_fn(button_gpio, button_callback_single, 1);
			adv_button_register_callback_fn(button_gpio, button_hold_callback, 5);		
		 }
		 
    }
    else if (event == HOMEKIT_EVENT_CLIENT_CONNECTED) {
        if (!paired)
           // led_status_set(led_status, &pairing);
	   printf("CLIENT JUST CONNECTED\n");
	   
	   
    }
    else if (event == HOMEKIT_EVENT_CLIENT_DISCONNECTED) {
        if (!paired)
            //led_status_set(led_status, &unpaired);
		printf("CLIENT JUST DISCONNECTED\n");
    }
    else if (event == HOMEKIT_EVENT_PAIRING_ADDED || event == HOMEKIT_EVENT_PAIRING_REMOVED) {
		paired = homekit_is_paired();
		// led_status_set(led_status, paired ? &normal_mode : &unpaired);
		printf("CLIENT JUST PAIRED\n");
		
	
		sdk_os_timer_arm(&extra_func_timer, 10000, 1);
			
		adv_toggle_create(motion_sensor_gpio, false);  // false -> without internal pull-up resistor
		adv_toggle_register_callback_fn(motion_sensor_gpio, movement_detected_fn, 1);    // High gpio state
		adv_toggle_register_callback_fn(motion_sensor_gpio, movement_not_detected_fn, 0);    // Low gpio state
		
		
		adv_button_create(button_gpio, true);
		adv_button_register_callback_fn(button_gpio, button_callback_single, 1);
		adv_button_register_callback_fn(button_gpio, button_hold_callback, 5);
    }
}




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
    .password = "111-11-111",
	.on_event = on_event,
};


void on_wifi_event(wifi_config_event_t event) {
    if (event == WIFI_CONFIG_CONNECTED) {
        printf("CONNECTED TO >>> WIFI <<<\n");
	
	Wifi_Connected=true;

	create_accessory_name();
    homekit_server_init(&config);
	
	sdk_os_timer_setfn(&extra_func_timer, sensor_worker, NULL);
	sdk_os_timer_disarm(&extra_func_timer);


    } else if (event == WIFI_CONFIG_DISCONNECTED) {
		Wifi_Connected = false;
        printf("DISCONNECTED FROM >>> WIFI <<<\n");
    }
}



void hardware_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(true);

	gpio_set_pullup(SENSOR_PIN, false, false);

	// IR Common initialization (can be done only once)
	ir_tx_init();
}


void user_init(void) {
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
	hardware_init();
	//wifi_config_init("Homekit-sensor", NULL, on_wifi_ready);
	wifi_config_init2("Homekit-sensor", NULL, on_wifi_event);
  
}