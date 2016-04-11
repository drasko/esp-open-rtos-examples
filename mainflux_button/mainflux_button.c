/* Respond to a button press.
 *
 * This code combines two ways of checking for a button press -
 * busy polling (the bad way) and button interrupt (the good way).
 *
 * This sample code is in the public domain.
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "esp8266.h"

#include <string.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>

#include "ssid_config.h"

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

/* You can use http://test.mosquitto.org/ to test mqtt_client instead
 * of setting up your own MQTT server */
#define MQTT_HOST ("test.mosquitto.org")
#define MQTT_PORT 1883

#define MQTT_USER NULL
#define MQTT_PASS NULL

#define PUB_MSG_LEN 16

/* pin config */
const int gpio = 0;   /* gpio 0 usually has "PROGRAM" button attached */
const int active = 0; /* active == 0 for active low */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;
#define GPIO_HANDLER gpio00_interrupt_handler

/**
 * MQTT Publish
 */
static void  mqtt_publish(void)
{
	int ret	= 0;
    uint8_t status	= 0;
	struct Network network;
	MQTTClient client = DefaultClient;
	char mqtt_client_id[20];
	uint8_t mqtt_buf[100];
	uint8_t mqtt_readbuf[100];
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    char msg[PUB_MSG_LEN - 1] = "\0";
    MQTTMessage message;

    status = sdk_wifi_station_get_connect_status();
	printf("%s: status = %d\n\r", __func__, status );

	NewNetwork( &network );
	memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
	strcpy(mqtt_client_id, "ESP-MAINFLUX");

	printf("%s: started\n", __func__);
	printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
	    MQTT_HOST);

	ret = ConnectNetwork(&network, MQTT_HOST, MQTT_PORT);
	if( ret ){
		printf("error: %d\n\r", ret);
		return;
	}
	printf("done\n\r");

	NewMQTTClient(&client, &network, 5000, mqtt_buf, 100, 
	    mqtt_readbuf, 100);

	data.willFlag		= 0;
	data.MQTTVersion	= 3;
	data.clientID.cstring	= mqtt_client_id;
	data.username.cstring	= MQTT_USER;
	data.password.cstring	= MQTT_PASS;
	data.keepAliveInterval	= 10;
	data.cleansession	= 0;
	printf("Send MQTT connect ... ");
	ret = MQTTConnect(&client, &data);
	if(ret){
		printf("error: %d\n", ret);
		DisconnectNetwork(&network);
        return;
	}
	printf("done\n");

	strcpy(msg, "HELLO-MAINFLUX");
    snprintf(msg, PUB_MSG_LEN, "HELLO-MAINFLUX\n");
    message.payload	= msg;
    message.payloadlen = PUB_MSG_LEN;
    message.dup = 0;
    message.qos = QOS1;
    message.retained = 0;
    ret = MQTTPublish(&client, "/mainflux", &message);
    if (ret != SUCCESS ){
   	    printf("error while publishing message: %d\n", ret );
    }

    ret = MQTTYield(&client, 1000);

    //printf("Connection dropped, request restart\n\r");
}


/* This task configures the GPIO interrupt and uses it to tell
   when the button is pressed.

   The interrupt handler communicates the exact button press time to
   the task via a queue.

   This is a better example of how to wait for button input!
*/
void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\n", gpio);
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    gpio_set_interrupt(gpio, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_RATE_MS;
        if(last < button_ts-200) {
            printf("Button interrupt fired at %dms\n", button_ts);
            last = button_ts;
            mqtt_publish();
        }
    }
}

static xQueueHandle tsqueue;

void GPIO_HANDLER(void)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    gpio_enable(gpio, GPIO_INPUT);

    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, (signed char *)"buttonIntTask", 2048, &tsqueue, 2, NULL);
}
