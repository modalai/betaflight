#
# Hexagon v66 Make file include
#

DEFAULT_OUTPUT := elf

# The Hexagon SDK and toolchain are supplied by the platform build container.
CROSS_CC  := $(CCACHE) hexagon-clang
CROSS_CXX := $(CCACHE) hexagon-clang++
SIZE      := hexagon-size

# Keep the upstream Betaflight version unchanged while publishing the HEXAGON
# fork version through Betaflight's existing BUILD_KEY and RELEASE_NAME fields.
# A Docker/CI build without .git can supply FORK_VERSION explicitly.
HEXAGON_FC_VERSION := $(call pp_def_value_str,src/main/build/version.h,FC_VERSION_STRING)
ifndef FORK_VERSION
FORK_VERSION := $(shell git describe --tags --match '$(HEXAGON_FC_VERSION)-[0-9]*' --abbrev=9 --dirty 2>/dev/null)
ifeq ($(strip $(FORK_VERSION)),)
FORK_VERSION := $(HEXAGON_FC_VERSION)-unknown
endif
ifneq ($(strip $(shell git status --porcelain --untracked-files=normal 2>/dev/null)),)
FORK_VERSION := $(patsubst %-dirty,%,$(FORK_VERSION))-dirty
endif
endif
ifeq ($(strip $(FORK_VERSION)),)
FORK_VERSION := $(HEXAGON_FC_VERSION)-unknown
endif
TARGET_FLAGS += -DBUILD_KEY=HEXAGON -DRELEASE_NAME=$(FORK_VERSION)

# Hexagon clang does not support the GCC-only loop warning or GCC LTO flags.
LTO := no
CFLAGS_DISABLED += -Wunsafe-loop-optimizations

# HEXAGON selects its supported features explicitly in target.h rather than
# inheriting the classic-build feature and driver defaults.
TARGET_FLAGS += -DCLOUD_BUILD

# The DSP image is a shared ELF linked directly with the Hexagon linker.
ELF_LINK_CMD = hexagon-link $(LD_FLAGS) -o $@ $(TARGET_OBJS)

HEXAGON_SDK_DIR = /home/4.1.0.4
INCLUDE_DIRS += $(HEXAGON_SDK_DIR)/rtos/qurt/computev66/include/qurt \
                $(HEXAGON_SDK_DIR)/rtos/qurt/computev66/include/posix

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
				   $(TARGET_PLATFORM_DIR)/include \
				   /home/4.1.0.4/rtos/qurt/computev66/include

MCU_FLASH_SIZE	:= 8192

LD_FLAGS := -march=hexagon -mcpu=hexagonv66 -shared -call_shared -G0
LD_FLAGS += $(TOOLS_DIR)/../target/hexagon/lib/v66/G0/pic/initS.o
LD_FLAGS += -L$(TOOLS_DIR)/../target/hexagon/lib/v66/G0/pic
LD_FLAGS += -Bsymbolic
LD_FLAGS += $(TOOLS_DIR)/../target/hexagon/lib/v66/G0/pic/libgcc.a
LD_FLAGS += --wrap=malloc --wrap=calloc --wrap=free --wrap=realloc --wrap=printf
LD_FLAGS += --wrap=strdup --wrap=__stack_chk_fail -lc
LD_FLAGS += -T$(LINKER_DIR)/hexagon.ld

MCU_COMMON_SRC = \
             HEXAGON/bus_i2c_hexagon.c \
             HEXAGON/audio_hexagon.c \
             HEXAGON/bus_spi_hexagon.c \
             HEXAGON/serial_uart_hexagon.c \
             HEXAGON/motor_hexagon.c \
             drivers/bus_spi_config.c \
             HEXAGON/unresolved.c \
             HEXAGON/exti.c \
             HEXAGON/target/HEXAGONV66/hexagon_main.c
