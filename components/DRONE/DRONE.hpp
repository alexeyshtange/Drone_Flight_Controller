#ifndef _DRONE_H_
#define _DRONE_H_
#include "wifi.hpp"
#include "PID.hpp"
#include "MPU6050_6Axis_MotionApps20.hpp"
#include "ESC.hpp"

#define CLK_SPEED 400000
#define PIN_SDA 21
#define PIN_CLK 22
#define INTERRUPT_PIN 4

extern SemaphoreHandle_t pidSemaphore;

void UdpTask(void*pvParameters);
void ControlTask(void*pvParameters);

class DRONE {
	private:
	  int sockfd;
	  struct sockaddr_in servaddr;
	  struct sockaddr_in cliaddr;
	  PID PidX, PidY, PidZ;
	  VectorFloat TargetAngles, ActualAngles;
	  float Gas;
	  MPU6050 mpu;
	  ledc_channel_config_t ledc_channel[4];
	  SemaphoreHandle_t SendDataMutex;
	  SemaphoreHandle_t ReceiveDataMutex;
public:
	DRONE();
    friend void UdpTask(void* pvParameters);
    friend void ControlTask(void* pvParameters);
};

#endif /* _DRONE_H_ */
