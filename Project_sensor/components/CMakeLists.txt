set(COMPONENT_SRCS          "./MAX30100/MAX30100.c"
                            "./MAX30100/MAX30100_main.c"
                            "./MAX30100/MAX30100_filter.c"
                            "./User_I2C/i2cmain.c"
                            "./ADXL345/ADXL345.c"
                            "./User_HTTP/HTTP_main.c"
                            "./ADXL345/SimpleKalmanFilter.c"
)

set(COMPONENT_INCLUDEDIRS   
                            "./MAX30100"
                            "./User_HTTP"
                            "./User_BLE"
                            "./User_Timer"
                            "./User_I2C"
                            "./ADXL345"
)

set(COMPONENT_REQUIRES      "driver" "esp_timer" "lwip" "freertos" "nvs_flash")


idf_component_register(     SRCS                "${COMPONENT_SRCS}"
                            INCLUDE_DIRS        "${COMPONENT_INCLUDEDIRS}"
                            REQUIRES            "${COMPONENT_REQUIRES}"
                            )