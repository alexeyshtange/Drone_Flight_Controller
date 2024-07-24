/*
 * DRONE.cpp
 *
 *  Created on: Jul 13, 2024
 *      Author: AliakseiShtanhel
 */
#include "DRONE.hpp"

static const char *TAGG = "DRONE";

DRONE::DRONE(){
	this->SendDataMutex = xSemaphoreCreateMutex();
	this->ReceiveDataMutex = xSemaphoreCreateMutex();
	InitESC(this->ledc_channel);
	I2Cdev::initialize(CLK_SPEED, PIN_SDA, PIN_CLK);
	this->mpu = MPU6050();
	this->mpu.initialize();
	this->mpu.dmpInitialize();
	this->mpu.CalibrateAccel(6);
	this->mpu.CalibrateGyro(6);
	this->mpu.setDMPEnabled(true);
	this->mpu.setIntDataReadyEnabled(true);
	//this->mpu.setDLPFMode(1);
	//this->mpu.setRate(39);

	wifi_init_sta(NULL);

	wifi_init_udp(this->sockfd, this->servaddr, this->cliaddr);
}


void UdpTask(void*pvParameters) {
	DRONE* drone = (DRONE*)(pvParameters);
	    char buff[60] = {};
	    for (;;) {

    		xSemaphoreTake(drone->SendDataMutex, portMAX_DELAY);
    	    memcpy(&buff[0], &(drone->ActualAngles.x), sizeof(float));
    	    memcpy(&buff[4], &(drone->ActualAngles.y), sizeof(float));
    	    memcpy(&buff[8], &(drone->ActualAngles.z), sizeof(float));
        	xSemaphoreGive(drone->SendDataMutex);
        	sendto(drone->sockfd, buff, 12, 0, (struct sockaddr*) &(drone->servaddr), sizeof(drone->servaddr));

		    if(recv(drone->sockfd, buff, 60, 0)){
		    	xSemaphoreTake(drone->ReceiveDataMutex, portMAX_DELAY);
		        memcpy(&(drone->TargetAngles.x), &buff[0], sizeof(float));
		        memcpy(&(drone->TargetAngles.y), &buff[4], sizeof(float));
		        memcpy(&(drone->TargetAngles.z), &buff[8], sizeof(float));
		        memcpy(&(drone->Gas), &buff[12], sizeof(float));
		        drone->PidX.SetCoefficients((float*)&buff[16], (float*)&buff[20], (float*)&buff[24]);
		        drone->PidY.SetCoefficients((float*)&buff[28], (float*)&buff[32], (float*)&buff[36]);
		        drone->PidZ.SetCoefficients((float*)&buff[40], (float*)&buff[44], (float*)&buff[48]);
		    	xSemaphoreGive(drone->ReceiveDataMutex);
	    }
	}

}

void ControlTask(void*pvParameters) {
	DRONE* drone = (DRONE*)(pvParameters);
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	uint8_t mpuIntStatus;
	uint16_t width[4];
	float x_out, y_out, z_out, h_out;
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    for (;;) {
  xSemaphoreTake(pidSemaphore, portMAX_DELAY);
    	fifoCount = drone->mpu.getFIFOCount();
    	if(fifoCount>packetSize)
    		drone->mpu.resetFIFO();
    	else{
    		while (fifoCount < packetSize) fifoCount = drone->mpu.getFIFOCount();
    		drone->mpu.getFIFOBytes(fifoBuffer, packetSize);
    		drone->mpu.dmpGetQuaternion(&q, fifoBuffer);
    		drone->mpu.dmpGetGravity(&gravity, &q);
    		drone->mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  xSemaphoreTake(drone->SendDataMutex, portMAX_DELAY);
  drone->ActualAngles.x = -ypr[1]* 180/M_PI;
  drone->ActualAngles.y = -ypr[2]* 180/M_PI;
  drone->ActualAngles.z = ypr[0]* 180/M_PI;
  xSemaphoreTake(drone->ReceiveDataMutex, portMAX_DELAY);
    	x_out = drone->PidX.Calculate(drone->TargetAngles.x, drone->ActualAngles.x);
    	y_out = drone->PidY.Calculate(drone->TargetAngles.y, drone->ActualAngles.y);
    	z_out = drone->PidZ.Calculate(drone->TargetAngles.z, drone->ActualAngles.z);
  xSemaphoreGive(drone->SendDataMutex);
  xSemaphoreGive(drone->ReceiveDataMutex);
        	width[0] = float_to_esc(drone->Gas + x_out - y_out - z_out);
        	width[1] = float_to_esc(drone->Gas + x_out + y_out + z_out); //PWM_MIN
        	width[2] = float_to_esc(drone->Gas - x_out + y_out - z_out); //PWM_MIN
        	width[3] = float_to_esc(drone->Gas - x_out - y_out + z_out);
       SetDronePWM(drone->ledc_channel, width);
     //  ESP_LOGI(TAGG, "drone->ActualAngles.y = %f; TIME = %lld", drone->ActualAngles.y, esp_timer_get_time());
    }
}
}

