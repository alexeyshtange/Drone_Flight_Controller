idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES DRONE ESC I2C MPU6050 PID WIFI)  # Укажите зависимости от всех компонентов