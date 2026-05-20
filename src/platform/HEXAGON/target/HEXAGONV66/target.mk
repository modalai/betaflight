TARGET_MCU        := HEXAGONV66
TARGET_MCU_FAMILY := HEXAGON

TARGET_FLAGS := -fblocks -fPIC -G0 -mv66 -fPIC -mcpu=hexagonv66 \
              -fomit-frame-pointer -fmerge-all-constants -fno-signed-zeros -fno-trapping-math \
              -freciprocal-math -fno-math-errno -fno-strict-aliasing -fvisibility=hidden -fno-rtti -fmath-errno
