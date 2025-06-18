#
# Hexagon v66 Make file include
#

HEXAGON_SDK_DIR = /opt/hexagon-sdk/4.1.0.4-lite
INCLUDE_DIRS += $(HEXAGON_SDK_DIR)/rtos/qurt/computev66/include/qurt \
                $(HEXAGON_SDK_DIR)/rtos/qurt/computev66/include/posix

$(info In Hexagon makefile)

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
				   $(TARGET_PLATFORM_DIR)/include

MCU_FLASH_SIZE	:= 8192

MCU_COMMON_SRC = \
             HEXAGON/bus_i2c_hexagon.c \
             HEXAGON/bus_i2c_hexagon_init.c \
             HEXAGON/audio_hexagon.c \
             HEXAGON/bus_spi_hexagon.c \
             HEXAGON/serial_uart_hexagon.c \
             HEXAGON/motor_hexagon.c \
             drivers/bus_spi_config.c \
             HEXAGON/unresolved.c \
             HEXAGON/exti.c \
             HEXAGON/target/HEXAGONV66/hexagon_main.c
