TARGET_MCU        := HEXAGONV66
TARGET_MCU_FAMILY := HEXAGON

OPTIMISATION_BASE := -ffast-math -fmerge-all-constants

CROSS_CC    := ccache hexagon-clang
CROSS_CXX   := ccache hexagon-clang++

TARGET_FLAGS := -fblocks -fPIC -G0 -mv66 -fPIC -mcpu=hexagonv66 \
              -fomit-frame-pointer -fmerge-all-constants -fno-signed-zeros -fno-trapping-math \
              -freciprocal-math -fno-math-errno -fno-strict-aliasing -fvisibility=hidden -fno-rtti -fmath-errno

LD_FLAGS := -march=hexagon -mcpu=hexagonv66 -shared -call_shared -G0
LD_FLAGS += $(TOOLS_DIR)/../target/hexagon/lib/v66/G0/pic/initS.o
LD_FLAGS += -L$(TOOLS_DIR)/../target/hexagon/lib/v66/G0/pic
LD_FLAGS += -Bsymbolic
LD_FLAGS += $(TOOLS_DIR)/../target/hexagon/lib/v66/G0/pic/libgcc.a
LD_FLAGS += --wrap=malloc --wrap=calloc --wrap=free --wrap=realloc --wrap=printf
LD_FLAGS += --wrap=strdup --wrap=__stack_chk_fail -lc
LD_FLAGS += -T$(LINKER_DIR)/hexagon.ld
