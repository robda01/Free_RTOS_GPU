# Edit following two lines to set component requirements (see docs)

idf_component_register(SRCS "ltc2309.c"
                       INCLUDE_DIRS "."
                       REQUIRES FreeRTOS task queue esp_log esp_system esp_err i2c)