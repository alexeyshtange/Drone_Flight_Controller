#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "DRONE.hpp"

extern "C" {
	void app_main(void);
}

volatile uint8_t div_by_3 = 0;

SemaphoreHandle_t pidSemaphore;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
	if (div_by_3 == 2) {
		div_by_3 = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(pidSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
		div_by_3++;
}

void app_main(void)
{
	pidSemaphore = xSemaphoreCreateBinary();

	DRONE drone;
	xTaskCreate(&UdpTask, "UdpTask", 2048, &drone, 2, NULL);
	xTaskCreate(&ControlTask, "ControlTask", 15000, &drone, 1, NULL);

    gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        io_conf.pin_bit_mask = (1ULL << INTERRUPT_PIN);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
        gpio_isr_handler_add((gpio_num_t)INTERRUPT_PIN, gpio_isr_handler, NULL);
        wifi_check_task(NULL);
}
