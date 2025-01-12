# for xiao rp2040
BOARD = GENERIC_RP_RP2040

# for mcp23017
I2C_DRIVER_REQUIRED = yes
CUSTOM_MATRIX = lite

SRC += matrix.c
# SRC += adns5050.cpp
# SRC += trackball.cpp
# SRC += scroll_sensor.cpp
