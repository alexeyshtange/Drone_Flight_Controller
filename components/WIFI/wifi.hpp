#ifndef MAIN_WIFI_H_
#define MAIN_WIFI_H_
//-------------------------------------------------------------
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
//-------------------------------------------------------------
#define SERVER_IP "192.168.209.176"//"192.168.209.176"//
#define CLIENT_IP "192.168.209.100"
#define SERVER_PORT 4444
#define CLIENT_PORT 3333
#define ESP_WIFI_SSID "shtrundel"
#define ESP_WIFI_PASSWORD "12345678a"
#define ESP_MAXIMUM_RETRY 5

static const char *TAG = "wifi";

static int s_retry_num = 0;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
//-------------------------------------------------------------
#define ESP_MAXIMUM_RETRY 5
void wifi_init_sta(void*);
void wifi_check_task(void*);
void wifi_init_udp(int &sockfd, struct sockaddr_in &servaddr, struct sockaddr_in &cliaddr);
//-------------------------------------------------------------
#endif /* MAIN_WIFI_H_ */
