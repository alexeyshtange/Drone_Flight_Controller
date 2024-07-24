/*
 * ESC.cpp
 *
 *  Created on: Jul 13, 2024
 *      Author: AliakseiShtanhel
 */


#include "ESC.hpp"

static const char *ESC = "ESC";

int16_t float_to_esc(float value){
	int16_t val =  ((value + 1) * (PWM_MAX - PWM_3RPM) / 2) + PWM_3RPM;
	if(val>PWM_MAX)
		return PWM_MAX;
	else if (val<PWM_3RPM)
		return PWM_3RPM;
	else
		return val;
}
float scale(float value, float min, float max, float new_min, float new_max){
	return ((value + max) * (new_max - new_min) / (max - min)) + new_min;
}

void InitPWM(ledc_channel_config_t *ledc_channel){
	ledc_timer_config_t ledc_timer;
	ledc_timer.duty_resolution = PWM_RESOLUTION;
	ledc_timer.freq_hz = 50;
	ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_timer.timer_num = LEDC_TIMER_0;
	ledc_timer.clk_cfg = LEDC_AUTO_CLK;

  	  ledc_timer_config(&ledc_timer);

  	      ledc_channel[0].gpio_num = LEDC_IO_0, ledc_channel[0].channel = LEDC_CHANNEL_0, ledc_channel[0].duty = 0, ledc_channel[0].intr_type = LEDC_INTR_DISABLE, ledc_channel[0].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[0].timer_sel = LEDC_TIMER_0, ledc_channel[0].hpoint =0;
  	      ledc_channel[1].gpio_num = LEDC_IO_1, ledc_channel[1].channel = LEDC_CHANNEL_1, ledc_channel[1].duty = 0, ledc_channel[1].intr_type = LEDC_INTR_DISABLE, ledc_channel[1].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[1].timer_sel = LEDC_TIMER_0, ledc_channel[1].hpoint =0;
  	      ledc_channel[2].gpio_num = LEDC_IO_2, ledc_channel[2].channel = LEDC_CHANNEL_2, ledc_channel[2].duty = 0, ledc_channel[2].intr_type = LEDC_INTR_DISABLE, ledc_channel[2].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[2].timer_sel = LEDC_TIMER_0, ledc_channel[2].hpoint =0;
  	      ledc_channel[3].gpio_num = LEDC_IO_3, ledc_channel[3].channel = LEDC_CHANNEL_3, ledc_channel[3].duty = 0, ledc_channel[3].intr_type = LEDC_INTR_DISABLE, ledc_channel[3].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[3].timer_sel = LEDC_TIMER_0, ledc_channel[3].hpoint =0;

  	  for (int i = 0; i < 4; i++) {
  	      ledc_channel_config(&ledc_channel[i]);
  	  }
    }
void SetChannelPWM(ledc_channel_config_t *ledc_channel, uint16_t width)
{
    ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, (width));
    ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel);
}
void SetDronePWM(ledc_channel_config_t *ledc_channel, uint16_t *width){
	for(uint8_t i = 0; i<4; i++) {
		SetChannelPWM(&ledc_channel[i], width[i]);
	}
}
void InitESC(ledc_channel_config_t *ledc_channel){
	InitPWM(ledc_channel);
    uint16_t width[4] = {PWM_MAX, PWM_MAX, PWM_MAX, PWM_MAX};
    SetDronePWM(ledc_channel, width);
	ESP_LOGI(ESC, "ESC_INIT_PWM_MAX\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    for(uint8_t i = 0; i<4; i++)
    	width[i]=PWM_MIN;
    SetDronePWM(ledc_channel, width);
    ESP_LOGI(ESC, "ESC_INIT_PWM_MIN\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    ESP_LOGI(ESC, "ESC_INITIALIZED !\n");
}
