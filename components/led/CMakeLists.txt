idf_component_register(
    SRCS "led.c" "ws2811.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_rmt mqtt driver)
