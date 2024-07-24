#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#define LEDC_IO_3 27
#define LEDC_IO_2 13
#define LEDC_IO_1 12
#define LEDC_IO_0 14

#define PWM_RESOLUTION LEDC_TIMER_16_BIT
#define PWM_MAX 7536
#define PWM_MIN 2621
#define PWM_3RPM 2760

int16_t float_to_esc(float value);
float scale(float value, float min, float max, float new_min, float new_max);
void InitPWM(ledc_channel_config_t *ledc_channel);
void SetChannelPWM(ledc_channel_config_t *ledc_channel, uint16_t width);
void SetDronePWM(ledc_channel_config_t *ledc_channel, uint16_t *width);
void InitESC(ledc_channel_config_t *ledc_channel);
