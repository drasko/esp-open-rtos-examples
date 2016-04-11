
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>


/* You can use http://test.mosquitto.org/ to test mqtt_client instead
 * of setting up your own MQTT server */
#define MQTT_HOST "mainflux.io"
#define MQTT_PORT 1883

#define MQTT_USER NULL
#define MQTT_PASS NULL


#define MAINFLUX_DEV_ID "FluxButton"
#define MAINFLUX_MQTT_TOPIC "/1234/"MAINFLUX_DEV_ID"/attributes/message"
#define MAINFLUX_MQTT_MESSAGE "ORBIT"

xSemaphoreHandle wifi_alive;
xSemaphoreHandle button_push;

xQueueHandle tsqueue;

#define PUB_MSG_LEN 16

/* pin config */
const int gpio = 0;   /* gpio 0 usually has "PROGRAM" button attached */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;
#define GPIO_HANDLER gpio00_interrupt_handler

static void  topic_received(MessageData *md)
{
	int i;
	MQTTMessage *message = md->message;
	printf("Received: ");
	for( i = 0; i < md->topic->lenstring.len; ++i)
		printf("%c", md->topic->lenstring.data[ i ]);

	printf(" = ");
	for( i = 0; i < (int)message->payloadlen; ++i)
		printf("%c", ((char *)(message->payload))[i]);

	printf("\r\n");
}

static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static void  mqttTask(void *pvParameters)
{
	int ret			= 0;
	struct Network network;
	MQTTClient client	= DefaultClient;
	char mqtt_client_id[20];
	uint8_t mqtt_buf[100];
	uint8_t mqtt_readbuf[100];
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

	NewNetwork( &network );
	memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
	strcpy(mqtt_client_id, "ESP-");
	strcat(mqtt_client_id, get_my_id());

	while(1)
    {
		xSemaphoreTake(wifi_alive, portMAX_DELAY);
		printf("%s: started\n", __func__);
		printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
			       	MQTT_HOST);
		ret = ConnectNetwork(&network, MQTT_HOST, MQTT_PORT);
		if (ret)
        {
			printf("error: %d\n", ret);
			taskYIELD();
			continue;
		}
		printf("done\n");
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
		if(ret)
        {
			printf("error: %d\n", ret);
			DisconnectNetwork(&network);
			taskYIELD();
			continue;
		}
		printf("done\n");
		
        /** Subscribe to the topic */
        MQTTSubscribe(&client, "/FluxButtonRx", QOS1, topic_received);

		while(1)
        {

			char msg[PUB_MSG_LEN - 1] = MAINFLUX_MQTT_MESSAGE;

		    xSemaphoreTake(button_push, portMAX_DELAY);

			printf("Got message to publish\n");
			MQTTMessage message;
			message.payload	= msg;
			message.payloadlen = PUB_MSG_LEN;
			message.dup = 0;
			message.qos = QOS1;
			message.retained = 0;
			ret = MQTTPublish(&client, MAINFLUX_MQTT_TOPIC, &message);
			if (ret != SUCCESS)
            {
				printf("error while publishing message: %d\n", ret );
			}

			ret = MQTTYield(&client, 1000);
			if (ret == DISCONNECTED)
				break;
		}
		printf("Connection dropped, request restart\n\r");
		taskYIELD();
	}
}

static void  wifiTask(void *pvParameters)
{
	uint8_t status	= 0;
	uint8_t retries = 30;
	struct sdk_station_config config = {
		.ssid = WIFI_SSID,
		.password = WIFI_PASS,
	};

	printf("WiFi: connecting to WiFi\n\r");
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);

	while(1)
	{
		while ((status != STATION_GOT_IP) && (retries))
        {
			status = sdk_wifi_station_get_connect_status();
			printf("%s: status = %d\n", __func__, status );
			if ( status == STATION_WRONG_PASSWORD )
            {
				printf("WiFi: wrong password\n");
				break;
			} else if (status == STATION_NO_AP_FOUND)
            {
				printf("WiFi: AP not found\n");
				break;
			} else if (status == STATION_CONNECT_FAIL) 
            {
				printf("WiFi: connection failed\n");
				break;
			}
			vTaskDelay( 1000 / portTICK_RATE_MS );
			--retries;
		}
		if (status == STATION_GOT_IP)
        {
			printf("WiFi: Connected\n");
			xSemaphoreGive(wifi_alive);
			taskYIELD();
		}

		while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP)
        {
			xSemaphoreGive(wifi_alive);
			taskYIELD();
		}
		printf("WiFi: disconnected\n");
		sdk_wifi_station_disconnect();
		vTaskDelay( 1000 / portTICK_RATE_MS );
	}
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
    gpio_set_interrupt(gpio, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_RATE_MS;
        if (last < button_ts-200)
        {
            printf("Button interrupt fired at %dms\n", button_ts);
            last = button_ts;

            xSemaphoreGive(button_push);
            taskYIELD();
        }
    }
}

void GPIO_HANDLER(void)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    vSemaphoreCreateBinary(wifi_alive);
    vSemaphoreCreateBinary(button_push);

    /** Take both semaphores (to "arm" them), as they are "given" by default */
	xSemaphoreTake(wifi_alive, portMAX_DELAY);
	xSemaphoreTake(button_push, portMAX_DELAY);

    tsqueue = xQueueCreate(2, sizeof(uint32_t));

    xTaskCreate(&wifiTask, (int8_t *)"wifiTask",  256, NULL, 2, NULL);
    xTaskCreate(&buttonIntTask, (int8_t *)"buttonIntTask", 256, NULL, 3, NULL); 
    xTaskCreate(&mqttTask, (int8_t *)"mqttTask", 1024, NULL, 4, NULL);
}
