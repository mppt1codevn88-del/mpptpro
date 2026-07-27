idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif nvs_flash
             esp_http_client esp-tls esp_https_ota app_update
             esp_adc esp_driver_ledc esp_driver_gpio esp_timer
             led_strip
    PRIV_REQUIRES esp_http_server
)