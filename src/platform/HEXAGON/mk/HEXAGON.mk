#
# Hexagon v66 Make file include
#

# ifeq ($(DEBUG_HARDFAULTS),F7)
# CFLAGS               += -DDEBUG_HARDFAULTS
# endif

HEXAGON_SDK_DIR = /opt/hexagon-sdk/4.1.0.4-lite
INCLUDE_DIRS += $(HEXAGON_SDK_DIR)/rtos/qurt/computev66/include/qurt \
                $(HEXAGON_SDK_DIR)/rtos/qurt/computev66/include/posix

$(info In Hexagon makefile)

#CMSIS
# CMSIS_DIR      := $(LIB_MAIN_DIR)/CMSIS
# 
# #STDPERIPH
# STDPERIPH_DIR   = $(LIB_MAIN_DIR)/HEXAGONV66/Drivers/HEXAGONV66_HAL_Driver
# STDPERIPH_SRC   = \
#             hexagonv66_hal_adc.c \
#             hexagonv66_hal_adc_ex.c \
#             hexagonv66_hal.c \
#             hexagonv66_hal_cortex.c \
#             hexagonv66_hal_dac.c \
#             hexagonv66_hal_dac_ex.c \
#             hexagonv66_hal_dma.c \
#             hexagonv66_hal_dma_ex.c \
#             hexagonv66_hal_exti.c \
#             hexagonv66_hal_flash.c \
#             hexagonv66_hal_flash_ex.c \
#             hexagonv66_hal_gpio.c \
#             hexagonv66_hal_i2c.c \
#             hexagonv66_hal_i2c_ex.c \
#             hexagonv66_hal_pcd.c \
#             hexagonv66_hal_pcd_ex.c \
#             hexagonv66_hal_pwr.c \
#             hexagonv66_hal_pwr_ex.c \
#             hexagonv66_hal_rcc.c \
#             hexagonv66_hal_rcc_ex.c \
#             hexagonv66_hal_rtc.c \
#             hexagonv66_hal_rtc_ex.c \
#             hexagonv66_hal_spi.c \
#             hexagonv66_hal_spi_ex.c \
#             hexagonv66_hal_tim.c \
#             hexagonv66_hal_tim_ex.c \
#             hexagonv66_hal_uart.c \
#             hexagonv66_hal_uart_ex.c \
#             hexagonv66_hal_usart.c \
#             hexagonv66_ll_dma2d.c \
#             hexagonv66_ll_dma.c \
#             hexagonv66_ll_gpio.c \
#             hexagonv66_ll_rcc.c \
#             hexagonv66_ll_spi.c \
#             hexagonv66_ll_tim.c \
#             hexagonv66_ll_usb.c \
#             hexagonv66_ll_utils.c
# 
# #USB
# USBCORE_DIR = HEXAGONV66/Middlewares/ST/HEXAGON_USB_Device_Library/Core
# USBCORE_SRC = \
#             $(USBCORE_DIR)/Src/usbd_core.c \
#             $(USBCORE_DIR)/Src/usbd_ctlreq.c \
#             $(USBCORE_DIR)/Src/usbd_ioreq.c
# 
# USBCDC_DIR = HEXAGONV66/Middlewares/ST/HEXAGON_USB_Device_Library/Class/CDC
# USBCDC_SRC = \
#             $(USBCDC_DIR)/Src/usbd_cdc.c
# 
# USBHID_DIR = HEXAGONV66/Middlewares/ST/HEXAGON_USB_Device_Library/Class/HID
# USBHID_SRC = \
#             $(USBHID_DIR)/Src/usbd_hid.c
# 
# USBMSC_DIR = HEXAGONV66/Middlewares/ST/HEXAGON_USB_Device_Library/Class/MSC
# USBMSC_SRC = \
#             $(USBMSC_DIR)/Src/usbd_msc_bot.c \
#             $(USBMSC_DIR)/Src/usbd_msc.c \
#             $(USBMSC_DIR)/Src/usbd_msc_data.c \
#             $(USBMSC_DIR)/Src/usbd_msc_scsi.c
# 
# DEVICE_STDPERIPH_SRC := \
#             $(STDPERIPH_SRC) \
#             $(USBCORE_SRC) \
#             $(USBCDC_SRC) \
#             $(USBHID_SRC) \
#             $(USBMSC_SRC)
# 
# #CMSIS
# VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/HEXAGONV66xx:$(STDPERIPH_DIR)/Src
# 
# CMSIS_SRC       :=
# INCLUDE_DIRS    := $(INCLUDE_DIRS) \
#                    $(TARGET_PLATFORM_DIR) \
#                    $(TARGET_PLATFORM_DIR)/startup \
#                    $(STDPERIPH_DIR)/Inc \
#                    $(LIB_MAIN_DIR)/$(USBCORE_DIR)/Inc \
#                    $(LIB_MAIN_DIR)/$(USBCDC_DIR)/Inc \
#                    $(LIB_MAIN_DIR)/$(USBHID_DIR)/Inc \
#                    $(LIB_MAIN_DIR)/$(USBMSC_DIR)/Inc \
#                    $(CMSIS_DIR)/Core/Include \
#                    $(LIB_MAIN_DIR)/HEXAGONV66/Drivers/CMSIS/Device/ST/HEXAGONV66xx/Include \
#                    $(TARGET_PLATFORM_DIR)/vcp_hal
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR)
# 
# #Flags
# # ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant
# ARCH_FLAGS      = 
# 
# # Flags that are used in the HEXAGON libraries
# DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER
# 
# DEVICE_FLAGS   += -DHEXAGONV66
# LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f765.ld
# STARTUP_SRC     = HEXAGON/startup/startup_hexagonv66.s
MCU_FLASH_SIZE	:= 8192
# 
# # Override the OPTIMISE_SPEED compiler setting to save flash space on these 512KB targets.
# # Performance is only slightly affected but around 50 kB of flash are saved.
# OPTIMISE_SPEED = -O2
# 
# DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DHEXAGON
# 
# VCP_SRC = \
#             HEXAGON/vcp_hal/usbd_desc.c \
#             HEXAGON/vcp_hal/usbd_conf_hexagonv66.c \
#             HEXAGON/vcp_hal/usbd_cdc_hid.c \
#             HEXAGON/vcp_hal/usbd_cdc_interface.c \
#             HEXAGON/serial_usb_vcp.c \
#             drivers/usb_io.c
# 
MCU_COMMON_SRC = \
             drivers/bus_i2c_config.c \
             HEXAGON/bus_i2c_hexagon.c \
             HEXAGON/bus_i2c_hexagon_init.c \
             HEXAGON/audio_hexagon.c \
             HEXAGON/bus_spi_hexagon.c \
             HEXAGON/serial_uart_hexagon.c \
             HEXAGON/motor_hexagon.c \
             drivers/accgyro/accgyro_mpu.c \
             drivers/bus_spi_config.c \
             HEXAGON/unresolved.c \
             HEXAGON/exti.c \
             HEXAGON/target/HEXAGONV66/hexagon_main.c

#             HEXAGON/serial_uart_hw.c \
#             drivers/serial_pinconfig.c \
#             drivers/serial_uart_pinconfig.c \

#             drivers/bus_i2c_timing.c \
#             drivers/dshot_bitbang_decode.c \
#             HEXAGON/adc_hexagonv66.c \
#             HEXAGON/audio_hexagonv66.c \
#             HEXAGON/bus_i2c_hal_init.c \
#             HEXAGON/bus_i2c_hal.c \
#             HEXAGON/bus_spi_ll.c \
#             HEXAGON/debug.c \
#             HEXAGON/dma_reqmap_mcu.c \
#             HEXAGON/dma_hexagonv66.c \
#             HEXAGON/dshot_bitbang_ll.c \
#             HEXAGON/dshot_bitbang.c \
#             HEXAGON/exti.c \
#             HEXAGON/io_stm32.c \
#             HEXAGON/light_ws2811strip_hal.c \
#             HEXAGON/persistent.c \
#             HEXAGON/pwm_output_dshot_hal.c \
#             HEXAGON/rcc_stm32.c \
#             HEXAGON/sdio_f7xx.c \
#             HEXAGON/serial_uart_hal.c \
#             HEXAGON/serial_uart_hexagonv66.c \
#             HEXAGON/system_hexagonv66.c \
#             HEXAGON/timer_hal.c \
#             HEXAGON/timer_hexagonv66.c \
#             HEXAGON/transponder_ir_io_hal.c \
#             HEXAGON/camera_control_stm32.c \
#             drivers/adc.c \
#             drivers/serial_escserial.c \
#             HEXAGON/startup/system_hexagonv66.c
# 
# MSC_SRC = \
#             drivers/usb_msc_common.c \
#             HEXAGON/usb_msc_hal.c \
#             msc/usbd_storage.c \
#             msc/usbd_storage_emfat.c \
#             msc/emfat.c \
#             msc/emfat_file.c \
#             msc/usbd_storage_sdio.c \
#             msc/usbd_storage_sd_spi.c
# 
# SPEED_OPTIMISED_SRC += \
#             HEXAGON/bus_i2c_hal.c \
#             HEXAGON/bus_spi_ll.c \
#             drivers/max7456.c \
#             HEXAGON/pwm_output_dshot_hal.c \
#             HEXAGON/exti.c
# 
# SIZE_OPTIMISED_SRC += \
#             drivers/bus_i2c_timing.c \
#             HEXAGON/bus_i2c_hal_init.c \
#             HEXAGON/serial_usb_vcp.c \
#             drivers/serial_escserial.c \
#             drivers/serial_pinconfig.c \
#             drivers/serial_uart_pinconfig.c
# 
# DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
# DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7

include $(TARGET_PLATFORM_DIR)/mk/HEXAGON_COMMON.mk
