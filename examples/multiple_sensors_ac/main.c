//#include <stdio.h>
//#include <string.h>
#include <esp/uart.h>
#include <rboot-api.h>
#include <sysparam.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>
#include <led_codes.h>
#include <adv_button.h>
#include <led_status.h>

#include <dht/dht.h>
#include <ds18b20/ds18b20.h>

//#include "../common/custom_characteristics.h"
#include <ir/ir.h>
#include <ir/raw.h>
//#include "http.h"
 #include <udplogger.h>

#include <lwip/dhcp.h>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <unistd.h>
#include <string.h>

#include <espressif/esp_common.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/api.h"

#include "ac_commands.h"

#define UDPLOG_PRINTF_TO_UDP
#define UDPLOG_PRINTF_ALSO_SERIAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#define DEVICE_MANUFACTURER "Unknown"
#define DEVICE_NAME "MultiSensor"
#define DEVICE_MODEL "esp8266"
char serial_value[13];  //Device Serial is set upon boot based on MAC address
#define FW_VERSION "0.0.1"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//#define	USE_THINGSPEAK 0 //Turn this into '1' if you want to use log temperature/humidity data at ThingsSpeak.com server
bool USE_THINGSPEAK =false ;
#define WEB_SERVER "api.thingspeak.com"
char *API_KEY = NULL; //Private Key
char *FIELD1 = NULL; //temp
char *FIELD2 = NULL; //hum
#define WEB_PORT "80"
#define WEB_PATH "/update?api_key="
#define THINGSPEAK_INTERVAL 600000 //600000 = 10min , 300000 = 5min 60000 = 1min
#define NUMBER_OF_RETRIES 5
bool Http_TaskStarted  =false;

TaskHandle_t callThingsProcess_handle = NULL;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// add this section to make your device OTA capable
// create the extra characteristic &ota_trigger, at the end of the primary service (before the NULL)
// it can be used in Eve, which will show it, where Home does not
// and apply the four other parameters in the accessories_information section

#include "ota-api.h"
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const int led_gpio = 2; // Pin D4
const int button_gpio = 4; //Pin (D2) 
const int motion_sensor_gpio = 16; // Wemos D1 mini pin: D0 
const int SENSOR_PIN = 5; //DHT sensor on pin D1
const int IR_PIN = 14 ; // Wemos D1 mini pin: D5. (Just for Reference)
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#define FREEHEAP()                          printf("Free Heap: %d\n", xPortGetFreeHeapSize())

volatile float old_humidity_value = 0.0, old_temperature_value = 0.0 , old_light_value =0.0, old_move_value = 0.0;
//AC related variables
volatile int stored_active_state = 0;
volatile int current_ac_mode = 0; //0=Automatic , 1=Heater , 2=Cool
volatile int current_ac_temp = 0;

volatile bool paired = false;
bool led_value = false; //This is to keep track of the LED status. 
volatile bool Wifi_Connected = false; 


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Light Sensor Parameters
#define LIGHT_SENSOR_TYPE               (0)
#define LUX_CALC_SCALAR                 (14357564.33)
#define LIGHT_SENSOR_OFFSET             (0)
#define LIGHT_SENSOR_RESISTOR           (8500)
#define LUX_CALC_EXPONENT               (1.396066767)
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool extra_function_TaskStarted = false;
bool extra_http_TaskStarted = false;
bool param_started = false;
ETSTimer extra_func_timer ;

void httpd_task(void *pvParameters)
{
	if (!FIELD1)
	FIELD1= malloc(20);

	if (!API_KEY)
	API_KEY= malloc(20);

	if (!FIELD2)
	FIELD2= malloc(20);

    struct netconn *client = NULL;
    struct netconn *nc = netconn_new(NETCONN_TCP);
    if (nc == NULL) {
        printf("Failed to allocate socket.\n");
        vTaskDelete(NULL);
    }
    netconn_bind(nc, IP_ADDR_ANY, 81);
    netconn_listen(nc);
    char buf[920];
    const char *webpage = {
        "HTTP/1.1 200 OK\r\n"
        "Content-type: text/html\r\n\r\n"
        "<html><head><title>HTTP Server</title>"
        "<style> div.main {"
        "font-family: Arial;"
        "padding: 0.01em 16px;"
        "box-shadow: 2px 2px 1px 1px #d2d2d2;"
        "background-color: #f1f1f1;}"
        "</style></head>"
        "<body><div class='main'>"
        "<h3>Homekit AC</h3>"
        "<p>URL: %s</p>"
        "<p>Uptime: %d seconds</p>"
        "<p>Free heap: %d bytes</p>"
		"<p>Thingspeak: %d</p>"
		"<p>API: %s</p>"
		"<p>Field1: %s</p>"
		"<p>Field2: %s</p>"
        "<button onclick=\"location.href='/REBOOT'\" type='button'>"
        "Reboot</button></p>"
        "<button onclick=\"location.href='/UPDATE'\" type='button'>"
        "Update</button></p>"
		"<button onclick=\"location.href='/OFF'\" type='button'>"
        "AC</button></p>"
		"<button onclick=\"location.href='/COOL'\" type='button'>"
        "COOL</button></p>"
		"<button onclick=\"location.href='/HEAT'\" type='button'>"
        "HEAT</button></p>"
        "</div></body></html>"
    };
    /* disable LED */

    while (1) {
        err_t err = netconn_accept(nc, &client);
        if (err == ERR_OK) {
            struct netbuf *nb;
            if ((err = netconn_recv(client, &nb)) == ERR_OK) {
                void *data;
                u16_t len;
                netbuf_data(nb, &data, &len);
                /* check for a GET request */
                if (!strncmp(data, "GET ", 4)) {
                    char uri[25];
                    const int max_uri_len = 25;
                    char *sp1, *sp2;
                    /* extract URI */
                    sp1 = data + 4;
                    sp2 = memchr(sp1, ' ', max_uri_len);
                    int len = sp2 - sp1;
                    memcpy(uri, sp1, len);
                    uri[len] = '\0';
                    printf("uri: %s\n", uri);
                    if (!strncmp(uri, "/REBOOT", max_uri_len))
                       sdk_system_restart();
				    else if (!strncmp(uri, "/UPDATE", max_uri_len))
					{
						printf("Updating firmware...\n");
						rboot_set_temp_rom(1);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						sdk_system_restart();
					}
                    else if (!strncmp(uri, "/OFF", max_uri_len))
					{
						taskENTER_CRITICAL();
						ac_button_off();
						taskEXIT_CRITICAL();
					}
					else if (!strncmp(uri, "/COOL", max_uri_len))
					{
						ac_command(2,22);
					}
					else if (!strncmp(uri, "/HEAT", max_uri_len))
					{
						ac_command(1,27);
					}
					else if (!strncmp(uri, "/thingspeak", max_uri_len))
					{

						USE_THINGSPEAK = !USE_THINGSPEAK ;
						sysparam_set_int8("USE_THINGSPEAK",(int) USE_THINGSPEAK);
						if (USE_THINGSPEAK == false) {
							//If the flag just turned false, you must delete the already current task. 
							taskHttp_delete();
						}

					}
					else if (!strncmp(uri, "/reset-homekit", max_uri_len))
					{
						printf("Resetting HomeKit Config\n");
						homekit_server_reset();
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						printf("Restarting\n");
						sdk_system_restart();
					}
					else if (!strncmp(uri, "/api", 4))
					{
						if (!API_KEY)
						API_KEY= malloc(20);
						int length1 = strlen(uri);
						strncpy(API_KEY,uri+5,length1-4);
						sysparam_set_string("API_KEY",API_KEY);
					}
					else if (!strncmp(uri, "/field1", 7))
					{
						int length1 = strlen(uri);
						if((length1-7)<10 && length1>7)
						{
							if (!FIELD1)
							FIELD1= malloc(20);
							strncpy(FIELD1,uri+8,length1-6);
							printf("%s\n",FIELD1);
							sysparam_set_string("FIELD1",FIELD1);
						}
					}
					else if (!strncmp(uri, "/field2", 7))
										{
						int length1 = strlen(uri);
						if((length1-7)<10 && length1>7)
						{
							if (!FIELD2)
							FIELD2= malloc(20);
							strncpy(FIELD2,uri+8,length1-6);
							printf("%s\n",FIELD2);
							sysparam_set_string("FIELD2",FIELD2);
						}
					}
					

                    //    gpio_write(2, true);
					printf("TEST\n");
                    snprintf(buf, sizeof(buf), webpage,
                            uri,
                            xTaskGetTickCount() * portTICK_PERIOD_MS / 1000,
                            (int) xPortGetFreeHeapSize(),(int) USE_THINGSPEAK,API_KEY,FIELD1,FIELD2);
                    netconn_write(client, buf, strlen(buf), NETCONN_COPY);
               
			    }
            }
            netbuf_delete(nb);
			
        }
        printf("Closing connection\n");
        netconn_close(client);
        netconn_delete(client);
    }
}

inline double fast_precise_pow(double a, double b) {
    int e = abs((int)b);

    union {
        double d;
        int x[2];
    } u = { a };

    u.x[1] = (int) ((b - e) * (u.x[1] - 1072632447) + 1072632447);
    u.x[0] = 0;

    double r = 1.0;

    while (e) {
        if (e & 1) {
            r *= a;
        }
        a *= a;
        e >>= 1;
    }

    return r * u.d;
}

float light_sensor_task() {
	// https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/
	//Thanks to: https://github.com/RavenSystem/esp-homekit-devices/blob/2fd952227bedebc6694236fe64129b6861791dac/devices/HAA/main.c#L3258
      
	float luxes = 0.0001f;

	const float adc_raw_read = sdk_system_adc_read();
 	float ldr_resistor;
	if (LIGHT_SENSOR_TYPE < 2) {

		if (LIGHT_SENSOR_TYPE == 0) {
					ldr_resistor = LIGHT_SENSOR_RESISTOR * ((1023.1 - adc_raw_read) / adc_raw_read);
		} 
		else if (LIGHT_SENSOR_TYPE == 1)
		{
					ldr_resistor = (LIGHT_SENSOR_RESISTOR  * adc_raw_read) / (1023.1 - adc_raw_read);
		}
		luxes = 1 / fast_precise_pow(ldr_resistor, LUX_CALC_EXPONENT);
		luxes = (luxes * LUX_CALC_SCALAR) + LIGHT_SENSOR_OFFSET;
		}
	else if (LIGHT_SENSOR_TYPE == 2)
	{
		luxes = adc_raw_read;

	}
	else if (LIGHT_SENSOR_TYPE == 3)
	{
		luxes = 1023 - adc_raw_read;

	}
	   

	printf("Luxes: %g\n", luxes);
	printf("ADC: %g\n",(float) sdk_system_adc_read());

    if (luxes < 0.0001f) {
        luxes = 0.0001f;
    } else if (luxes > 100000.f) {
        luxes = 100000.f;
    }

 
   return luxes;
}



//####################################################################################
//LED codes
// 1000ms ON, 1000ms OFF
led_status_pattern_t waiting_wifi = LED_STATUS_PATTERN({1500, -500});

// one short blink every 3 seconds
led_status_pattern_t normal_mode = LED_STATUS_PATTERN({100, -2900});

// one short blink every 3 seconds
led_status_pattern_t unpaired  = LED_STATUS_PATTERN({100, -100, 100, -100, 100, -700});

led_status_pattern_t pairing_started  = LED_STATUS_PATTERN({400, -200,400, -200, 400, -200, 100, -700});


// three short blinks
led_status_pattern_t three_short_blinks = LED_STATUS_PATTERN({100, -100, 100, -100, 100, -700});

#define STATUS_LED_PIN 2

static led_status_t led_status ;
//####################################################################################


void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_state();
}

void on_temp_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_temp();
}


void on_active_change(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_active();
}

////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



//following code thanks to @maccoylton at https://github.com/maccoylton/esp-homekit-common-functions/blob/master/shared_functions/shared_functions.c
//* configUSE_TRACE_FACILITY must be defined as 1 in FreeRTOSConfig.h , or in makefile: EXTRA_CFLAGS += -DconfigUSE_TRACE_FACILITY
// * uxTaskGetSystemState() to be available.
TaskHandle_t task_stats_task_handle = NULL;
void task_stats_task ( void *args)
{
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;
    unsigned long ulStatsAsPercentage;
    
    
    printf ("%s", __func__);
    
    while (1) {
        /* Take a snapshot of the number of tasks in case it changes while this
         function is executing. */
        uxArraySize = uxTaskGetNumberOfTasks();
        
        printf (", uxTaskGetNumberOfTasks %ld", uxArraySize);
        /* Allocate a TaskStatus_t structure for each task.  An array could be
         allocated statically at compile time. */
        pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
        
        printf (", pvPortMalloc");
        
        if( pxTaskStatusArray != NULL )
        {
            /* Generate raw status information about each task. */
            uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                               uxArraySize,
                                               &ulTotalRunTime );
            
            printf (", uxTaskGetSystemState, ulTotalRunTime %d, array size %ld\n", ulTotalRunTime, uxArraySize);
            
            /* Avoid divide by zero errors. */
            /*        if( ulTotalRunTime > 0 )
             {*/
            /* For each populated position in the pxTaskStatusArray array,
             format the raw data as human readable ASCII data. */
            for( x = 0; x < uxArraySize; x++ )
            {
                /* What percentage of the total run time has the task used?
                 This will always be rounded down to the nearest integer.
                 ulTotalRunTimeDiv100 has already been divided by 100. */
                ulStatsAsPercentage =
                pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime / 100;
                
                
                printf ( "Name:%-20s,  Runtime Counter:%-3d, Current State:%-3d, Current Priority:%-5ld, Base Priority:%-5ld, High Water Mark (bytes) %-5d\n",
                        pxTaskStatusArray[ x ].pcTaskName,
                        pxTaskStatusArray[ x ].ulRunTimeCounter,
                        pxTaskStatusArray[ x ].eCurrentState,
                        pxTaskStatusArray[ x ].uxCurrentPriority ,
                        pxTaskStatusArray[ x ].uxBasePriority,
                        pxTaskStatusArray[x].usStackHighWaterMark);
                
                /*                printf ( " Runtime Percentage:%lu,",ulStatsAsPercentage);*/
                
            }
            /*        }*/
            


            /* The array is no longer needed, free the memory it consumes. */
            vPortFree( pxTaskStatusArray );
            printf ("%s, vPortFree\n", __func__);
        }

		FREEHEAP();  

        vTaskDelay((30000) / portTICK_PERIOD_MS);
    }
    
}

void switch_on_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
homekit_characteristic_t switch_on = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_callback)
);


void switch_on_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    printf("Task Stats\n");
	FREEHEAP();
	    if (switch_on.value.bool_value)
        {
            if (task_stats_task_handle == NULL){
                xTaskCreate(task_stats_task, "task_stats_task", 256 , NULL, tskIDLE_PRIORITY+1, &task_stats_task_handle);
            } else {
                printf ("%s task_Status_set TRUE, but task pointer not NULL\n", __func__);
            }
        }
    else
    {
        if (task_stats_task_handle != NULL){
            vTaskDelete (task_stats_task_handle);
            task_stats_task_handle = NULL;
        } else {
            printf ("%s task_Status_set FALSE, but task pointer is NULL\n", __func__);
        }
	FREEHEAP();
        
    }
    
}
// //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Temperature, Humidity sensors
homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
//Additional for Motion Sensor
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
homekit_characteristic_t cooling_threshold = HOMEKIT_CHARACTERISTIC_(COOLING_THRESHOLD_TEMPERATURE, 18,.callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_temp_update),.min_value = (float[]) {18},.max_value = (float[]) {30},.min_step = (float[]) {1} );
homekit_characteristic_t heating_threshold = HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 25,.callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_temp_update),.min_value = (float[]) {18},.max_value = (float[]) {30},.min_step = (float[]) {1});
//homekit_characteristic_t rotation_speed = HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));

//Device Information
homekit_characteristic_t manufacturer     = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  DEVICE_MANUFACTURER);
homekit_characteristic_t serial           = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, serial_value);
homekit_characteristic_t model            = HOMEKIT_CHARACTERISTIC_(MODEL,         DEVICE_MODEL);
homekit_characteristic_t revision         = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  FW_VERSION);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//AC related code
void update_state() {
	uint8_t target_state = target_heater_cooler_state.value.int_value;
	uint8_t active_status = active.value.int_value;
	uint8_t current_state = current_heater_cooler_state.value.int_value;
	//printf("Running update_state() \n" );
	printf("UPDATE STATE: Active State: %d, Current State: %d, Target State: %d \n",active_status, current_state, target_state );
	
	
	//current_heater_cooler_state.value = HOMEKIT_UINT8(target_state+1);
	//homekit_characteristic_notify(&current_heater_cooler_state, current_heater_cooler_state.value);
	sysparam_set_int8("ac_mode",target_state);
	current_ac_mode = target_state;
	stored_active_state = active_status;
	
	if (active_status == 0) {
	//	printf("OFF\n" );
	//	ac_button_off();
	}
	else
	{	
		if(target_state == 0){
		taskENTER_CRITICAL();
		ac_button_aut();
		taskEXIT_CRITICAL();
		led_status_signal(led_status, &three_short_blinks);
		}
		else if ((target_state + 1 )!=current_state){
		//printf("AC is now ON, updating temp\n" );
		update_temp();
		}
	} 
	               
}
void update_temp() {
	//TARGET: 
	//0 = AUTOMATIC
	//1 = HEAT
	//2 = COOL

	//CURRENT STATE:
	//0 = inactive
	//1 = idle
	//2 = heating
	//3 = cooling

	//printf("Running update_temp() \n" );
	uint8_t target_state = target_heater_cooler_state.value.int_value;
	uint8_t active_status = active.value.int_value;
	uint8_t current_state = current_heater_cooler_state.value.int_value;
	printf("UPDATE TEMP: Active Status: %d, Current State: %d, Target State: %d \n",active_status, current_state, target_state );


	float target_temp1 = 0;
	target_state = (int)target_state;
 
	if (target_state == 0)//AUTOMATIC
	{
		if (active_status ==1)
		{
			printf("This should normally sent AUTO command\n" );
			//Auto mode
			current_heater_cooler_state.value = HOMEKIT_UINT8(target_state);
			homekit_characteristic_notify(&current_heater_cooler_state, current_heater_cooler_state.value);
		}

		/* code */
	}
	else if (target_state == 1) { //HEAT
	 //Read the Heat target
		target_temp1= heating_threshold.value.float_value;
		current_heater_cooler_state.value = HOMEKIT_UINT8(target_state+1);
		homekit_characteristic_notify(&current_heater_cooler_state, current_heater_cooler_state.value);
	}
	else if (target_state == 2) //COOLING
	{
	 //Else read the Cool target
		target_temp1= cooling_threshold.value.float_value; 
		current_heater_cooler_state.value = HOMEKIT_UINT8(target_state+1);
		homekit_characteristic_notify(&current_heater_cooler_state, current_heater_cooler_state.value);
	}
  	current_ac_temp = (int)target_temp1;
	printf("Target temp: %.1f\n",target_temp1 );

	sysparam_set_int8("ac_mode",target_state);
	sysparam_set_int8("ac_temp",(int)target_temp1);
	
	taskENTER_CRITICAL();
 	ac_command(target_state,target_temp1);
	taskEXIT_CRITICAL();
	
	led_status_signal(led_status, &three_short_blinks);

}

void update_active() {

	//printf("\nRunning update_active() \n" );
	uint8_t target_state = target_heater_cooler_state.value.int_value;
	uint8_t active_status = active.value.int_value;
	uint8_t current_state = current_heater_cooler_state.value.int_value;
	printf("\nUPDATE ACTIVE: Active Status: %d, Current State: %d, Target State: %d \n",active_status, current_state, target_state );
	
	//If its being requested to turn ON and saved status is different, then send IR command   
	
	if (active_status == 1  )
	{
		if (target_state == 0)
		{
			taskENTER_CRITICAL();
			ac_button_aut();
			taskEXIT_CRITICAL();
			led_status_signal(led_status, &three_short_blinks);
		}
		else
		{
			update_temp();
		}
	}
	
	//If it is requested to turn off  
	if (active_status == 0  ) {
	//	printf("OFF\n" );
	taskENTER_CRITICAL();
	ac_button_off();
	taskEXIT_CRITICAL();
	led_status_signal(led_status, &three_short_blinks);
	}

	
	stored_active_state = active_status;

	//save ON/OFF status to sysparameters
	sysparam_set_int8("ac_active",stored_active_state);

             
}

void int_ac_state(){
//At reboot, set initial AC state from saved parameters  
sysparam_status_t status;

	printf("Initializing AC parameters\n");

	status = sysparam_get_int8("ac_mode", &current_ac_mode);
	if (status == SYSPARAM_OK){

		int new_current_status = 1;
		if (current_ac_mode == 1){
			//Cool
			new_current_status =2;
			
			status = sysparam_get_int8("ac_temp", &current_ac_temp);
			if (status == SYSPARAM_OK){
			heating_threshold.value = HOMEKIT_FLOAT((float)current_ac_temp);
			}

		}
		if (current_ac_mode == 2)
		{
			//Heating
			new_current_status =3;
			status = sysparam_get_int8("ac_temp", &current_ac_temp);
			if (status == SYSPARAM_OK){
				printf("Temp:%d \n",current_ac_temp);
			cooling_threshold.value = HOMEKIT_FLOAT((float)current_ac_temp);
			
			}
		}
	
		current_heater_cooler_state.value = HOMEKIT_UINT8(new_current_status);
		target_heater_cooler_state.value = HOMEKIT_UINT8(current_ac_mode);
		
	 }


	status = sysparam_get_int8("ac_active", &stored_active_state);
	 if (status == SYSPARAM_OK){
	active.value = HOMEKIT_UINT8(stored_active_state);
	 }

	//Thingspeak parameters initialiazation

	if (!FIELD1)
	FIELD1= malloc(20);

	if (!API_KEY)
	API_KEY= malloc(20);

	if (!FIELD2)
	FIELD2= malloc(20);

	status = sysparam_get_int8("USE_THINGSPEAK", &USE_THINGSPEAK);
	status = sysparam_get_string("API_KEY",&API_KEY);
	status = sysparam_get_string("FIELD1",&FIELD1);
	status = sysparam_get_string("FIELD2",&FIELD2);


}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void call_things_process()
{
	int successes = 0, failures = 0;
    printf("HTTP get task starting...\r\n");
	
	while(1) {
	FREEHEAP();
	if (xPortGetFreeHeapSize()<8000){
	printf("Low free heap\n");
	printf("Temporarily killing http task\n\n");
	taskHttp_delete();
	}


if (Wifi_Connected == true)
{
	printf("Running HTTP get request...\r\n");

			if (old_humidity_value == 0.0) 
			{
				printf("Temporarily killing http task, sensor values are not ready \r\n");
				taskHttp_delete();
								
				//vTaskDelay(120000 / portTICK_PERIOD_MS);
				//continue;
			}
	
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
				if (failures>NUMBER_OF_RETRIES)
				{
				printf("Too many http failures. Temporarily killing http task\r\n");
				taskHttp_delete();
				}
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
				if (failures>NUMBER_OF_RETRIES)
				{
				printf("Too many http failures. Temporarily killing http task\r\n");
				taskHttp_delete();
				}
				continue;
			}

			printf("... allocated socket\r\n");

			if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
				close(s);
				freeaddrinfo(res);
				printf("... socket connect failed.\r\n");
				vTaskDelay(4000 / portTICK_PERIOD_MS);
				failures++;
				// if (failures>NUMBER_OF_RETRIES)
				// {
				// 	printf("Temporarily killing http task");
				// 	vTaskDelete(callThingsProcess_handle);
				// 	callThingsProcess_handle = NULL;
				// }
				if (failures>NUMBER_OF_RETRIES)
				{
				printf("Too many http failures. Temporarily killing http task\r\n");
				taskHttp_delete();
				}
				
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
			

			//char req[300]="GET " WEB_PATH API_KEY "&" FIELD1;
			char req[300]="GET " WEB_PATH;
			strncat(req,API_KEY,16);
			strncat(req,"&",2);
			strncat(req,FIELD1,20);
			strncat(req,temp2,10);
			strncat(req,"&",2);
			strncat(req,FIELD2,20);
			strncat(req,hum1,6);
			strncat(req," HTTP/1.1\r\nHost: "WEB_SERVER"\r\nUser-Agent: esp-open-rtos/0.1 esp8266\r\nConnection: close\r\n\r\n",250);
			
			
			printf("%s \n",req );
		
			
			// printf("%s \n",req );
			
			if (write(s, req, strlen(req)) < 0) {
				printf("... socket send failed\r\n");
				close(s);
				vTaskDelay(4000 / portTICK_PERIOD_MS);
				failures++;
				if (failures>NUMBER_OF_RETRIES)
				{
				printf("Temporarily killing http task\n");
				taskHttp_delete();
				}
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

			successes=0;
			failures =0;

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
	sensor_worker();
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
	//light_value = sdk_system_adc_read();

		//New version thanks to @Ravensystem and HAA 3.7.0, 5.0.4
		light_value = light_sensor_task();
	
	 	 if (light_value != old_light_value) 
		 {
		 old_light_value = light_value;
		 //currentAmbientLightLevel.value.float_value = (1024 - light_value);
		 //homekit_characteristic_notify(&currentAmbientLightLevel, HOMEKIT_FLOAT((1024 - light_value))); 
		currentAmbientLightLevel.value.float_value = ( light_value);
		homekit_characteristic_notify(&currentAmbientLightLevel, HOMEKIT_FLOAT(( light_value))); 
		}




	 

}
	
	
void movement_detected_fn(const uint8_t gpio) {
	printf("Movement detected\n");
	old_move_value = true;
	motion_detected.value = HOMEKIT_BOOL(old_move_value);
	//led_write(old_move_value);
	led_status_signal(led_status, &three_short_blinks);
	homekit_characteristic_notify(&motion_detected, HOMEKIT_BOOL(old_move_value));
	
}

void movement_not_detected_fn(const uint8_t gpio) {
	printf("Movement not detected\n");

	old_move_value = false;
	motion_detected.value = HOMEKIT_BOOL(old_move_value);

	homekit_characteristic_notify(&motion_detected, HOMEKIT_BOOL(old_move_value));
}




void button_callback_single(const uint8_t gpio) {

	printf("Toggling relay\n");
	//led_write(!led_value); //A Single press will just toggle the LED. This was left for testing purposes. 
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
			&manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
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
			//          &units,
//          &rotation_speed,
			&ota_trigger,
            NULL
        }),
        NULL
    }),
 HOMEKIT_ACCESSORY( .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
           	&manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
            NULL
        }),
		HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Motion Sensor"),
            &motion_detected,
            NULL
        }), 

        		HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
			&temperature,
            NULL
        }),
		 HOMEKIT_SERVICE(LIGHT_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Light Sensor"),
            &currentAmbientLightLevel,
            NULL
        }),
		  
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
			&humidity,
            NULL
        }),
    NULL
 }),
 NULL
    
};

int event_counter =0;
void on_event(homekit_event_t event) {
	paired = homekit_is_paired();
	event_counter+=1;

			if ((!paired && event_counter>15) || paired){ 
			if (extra_http_TaskStarted == false){
				extra_http_TaskStarted = true;
				xTaskCreate(&httpd_task, "http_server", 1024, NULL, tskIDLE_PRIORITY+2, NULL);
				printf("\n\nSTARTING HTTP SEVER . . . \n\n");	 	
			 }
			 }


    if (event == HOMEKIT_EVENT_SERVER_INITIALIZED) {
        led_status_set(led_status, paired ? NULL : &unpaired);
		 printf("SERVER JUST INITIALIZED\n");
		
		 if (paired == true){
			 //led_status_set(led_status, NULL);
			 
			if (USE_THINGSPEAK == true && !Http_TaskStarted){
				//Start the ThingsSpeak process
				FREEHEAP();
				if (xPortGetFreeHeapSize()>8000){
				printf("http task started\n");
				xTaskCreate(call_things_process, "http", 1024, NULL, 1, &callThingsProcess_handle);
				Http_TaskStarted=true;
				}
			}
		 
			if (extra_function_TaskStarted ==false)
			{
				 if (sdk_wifi_station_get_connect_status() == STATION_GOT_IP) {
				UDPLOG_PRINTF_TO_UDP;
				//UDPLOG_PRINTF_ALSO_SERIAL;
    			//udplog_init(3);
				udplog_init(tskIDLE_PRIORITY+1);
				printf("Started UDP logging1\n");

				printf("Found pairing, starting timers\n");
				sdk_os_timer_setfn(&extra_func_timer, sensor_worker, NULL);
				//sdk_os_timer_disarm(&extra_func_timer);
				sdk_os_timer_arm(&extra_func_timer, 10000, 1);
				extra_function_TaskStarted=true;
			
				adv_toggle_create(motion_sensor_gpio, false);  // false -> without internal pull-up resistor
				adv_toggle_register_callback_fn(motion_sensor_gpio, movement_detected_fn, 1);    // High gpio state
				adv_toggle_register_callback_fn(motion_sensor_gpio, movement_not_detected_fn, 0);    // Low gpio state
						
				 }
			}

			if (param_started == false){
			int_ac_state();
			param_started=true;
			}	
		}
	
		 
    }
    else if (event == HOMEKIT_EVENT_CLIENT_CONNECTED) {
        if (!paired){
            led_status_set(led_status, &pairing_started);
		}
		else
		{
			 led_status_set(led_status, NULL);
		}
		
	   		printf("CLIENT JUST CONNECTED\n");
	   
	   
    }
    else if (event == HOMEKIT_EVENT_CLIENT_DISCONNECTED) {
        if (!paired){
 			led_status_set(led_status, &unpaired);

		}else
		{
			//Τhis is where I start secondart tasks. The pairing has already been completed and a first client disconnection has happened. 
			//if (extra_http_TaskStarted == false){
			//	extra_http_TaskStarted = true;
			//	xTaskCreate(&httpd_task, "http_server", 1024, NULL, tskIDLE_PRIORITY+2, NULL);
			// }
		}
		
        printf("CLIENT JUST DISCONNECTED\n");

    }
	else if (event == HOMEKIT_EVENT_CLIENT_VERIFIED) {
		printf("CLIENT JUST VERIFIED\n");
		if (paired == true)
		{
			if (extra_http_TaskStarted == false){
				extra_http_TaskStarted = true;
				xTaskCreate(&httpd_task, "http_server", 1024, NULL, tskIDLE_PRIORITY+2, NULL);
			 }
			
			led_status_set(led_status, NULL);
			if (callThingsProcess_handle == NULL)
			{
				//printf("The CALL PROCESS IS NOT NULL\n");
				printf("\n\n\n\n\n\n\nHTTP FLAG IS %d\n\n\n\n\n\n\n\n",(int)Http_TaskStarted);
				if (USE_THINGSPEAK == true && !Http_TaskStarted){
					//Start the ThingsSpeak process
					FREEHEAP();
					if (xPortGetFreeHeapSize()>8000){
					printf("http task started\n");
					xTaskCreate(call_things_process, "http", 1024, NULL, 1, &callThingsProcess_handle);
					Http_TaskStarted=true;
					}
				}
			}
				if (extra_function_TaskStarted ==false)
				{
					 if (sdk_wifi_station_get_connect_status() == STATION_GOT_IP) {
					UDPLOG_PRINTF_TO_UDP;
					//UDPLOG_PRINTF_ALSO_SERIAL;
					//udplog_init(3);
					udplog_init(tskIDLE_PRIORITY+1);
					printf("Started UDP logging2\n");

					printf("Found pairing, starting timers\n");
					sdk_os_timer_setfn(&extra_func_timer, sensor_worker, NULL);
					//sdk_os_timer_disarm(&extra_func_timer);
					sdk_os_timer_arm(&extra_func_timer, 10000, 1);
					extra_function_TaskStarted=true;
					
					adv_toggle_create(motion_sensor_gpio, false);  // false -> without internal pull-up resistor
					adv_toggle_register_callback_fn(motion_sensor_gpio, movement_detected_fn, 1);    // High gpio state
					adv_toggle_register_callback_fn(motion_sensor_gpio, movement_not_detected_fn, 0);    // Low gpio state
					 }
				}	

				if (param_started == false){
					int_ac_state();
					param_started=true;
				}

		 
		}


	}
    else if (event == HOMEKIT_EVENT_PAIRING_ADDED || event == HOMEKIT_EVENT_PAIRING_REMOVED) {
		paired = homekit_is_paired();
		led_status_set(led_status, paired ? NULL : &unpaired);
		printf("CLIENT JUST PAIRED\n");

		if (!paired){
			printf("CLIENT LOST PAIRING - REBOOT\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			sdk_system_restart();
		}
			// if (USE_THINGSPEAK == 1){
			// 	if (TaskStarted == false){
			// 	//Start the ThingsSpeak process only if relevant flag is set
			// 	printf("http task started\n");
			// 	xTaskCreate(call_things_process, "http", 1024, NULL, 1, NULL);
			// 	TaskStarted=true;
			// 	}
			// }
	
		// sdk_os_timer_arm(&extra_func_timer, 10000, 1);
			
		// adv_toggle_create(motion_sensor_gpio, false);  // false -> without internal pull-up resistor
		// adv_toggle_register_callback_fn(motion_sensor_gpio, movement_detected_fn, 1);    // High gpio state
		// adv_toggle_register_callback_fn(motion_sensor_gpio, movement_not_detected_fn, 0);    // Low gpio state
		
		
		// adv_button_create(button_gpio, true);
		// adv_button_register_callback_fn(button_gpio, button_callback_single, 1);
		// adv_button_register_callback_fn(button_gpio, button_hold_callback, 5);
    }
}

void taskHttp_delete()
{
TaskHandle_t xTask = callThingsProcess_handle;
Http_TaskStarted = false;
//vTaskSuspendAll();
if( callThingsProcess_handle != NULL )
{
    /* The task is going to be deleted.
    Set the handle to NULL. */
    callThingsProcess_handle = NULL;
    /* Delete using the copy of the handle. */
    vTaskDelete( xTask );
}
//xTaskResumeAll();
}


void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);
    
    int name_len = snprintf(NULL, 0, "HomekitSensor-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "HomekitSensor-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);
    
    name.value = HOMEKIT_STRING(name_value);


	snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);

}

homekit_server_config_t config = {
	.accessories = accessories,
    .password = "222-22-222",
	.on_event = on_event,
};

void on_wifi_event(wifi_config_event_t event) {


    if (event == WIFI_CONFIG_CONNECTED) 
	{
		printf("CONNECTED TO >>> WIFI <<<\n");
		//led_status_signal(led_status, &three_short_blinks); //This seemed like needed, otherwise the library does not work well.
		led_status_set(led_status, NULL);
		
		//led_status_set(led_status,NULL);
		Wifi_Connected=true;

		create_accessory_name();
			// 	if (homekit_is_paired()){
			// 	char *custom_hostname = name.value.string_value;
			// 	struct netif *netif = sdk_system_get_netif(STATION_IF);
			// 		if (netif) {
			// 			printf("HOSTNAME set>>>>>:%s\n", custom_hostname);
			// 			LOCK_TCPIP_CORE();
			// 			dhcp_release_and_stop(netif);
			// 			netif->hostname = "foobar";
			// 			netif->hostname =custom_hostname;
			// 			dhcp_start(netif);
			// 			UNLOCK_TCPIP_CORE();
			// 		 }
			// }
			//OTA
			int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
											&model.value.string_value,&revision.value.string_value);
			

			c_hash=1; revision.value.string_value="0.0.1"; //cheat line
			config.accessories[0]->config_number=c_hash;
    


			homekit_server_init(&config);
			FREEHEAP();
    } 
	else if (event == WIFI_CONFIG_DISCONNECTED)
	 {
		Wifi_Connected = false;
        printf("DISCONNECTED FROM >>> WIFI <<<\n");
		led_status_set(led_status,&waiting_wifi);
    }
}



void hardware_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
  //  led_write(true);

	gpio_set_pullup(SENSOR_PIN, false, false);
	gpio_set_pullup(IR_PIN, false, false);

	// IR Common initialization (can be done only once)
	ir_tx_init();

	adv_button_create(button_gpio, true);
	adv_button_register_callback_fn(button_gpio, button_callback_single, 1);
	adv_button_register_callback_fn(button_gpio, button_hold_callback, 2);	

	 led_status = led_status_init(STATUS_LED_PIN, 0);
	 
	  
	 
}


void user_init(void) {
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
	hardware_init();



	//wifi_config_init("Homekit-sensor", NULL, on_wifi_ready);
	wifi_config_init2("Homekit-Sensor", NULL, on_wifi_event);
  
}
