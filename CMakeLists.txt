cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Drone_Flight_Controller)

# Укажите папки с компонентами
set(EXTRA_COMPONENT_DIRS components/DRONE components/ESC components/I2C components/MPU6050 components/PID components/WIFI)