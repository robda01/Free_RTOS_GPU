# Edit following two lines to set component requirements (see docs)

idf_component_register(SRCS  "hd44780.c"
                       INCLUDE_DIRS "."
                       REQUIRES i2c pcf8574 freertos)
                       
