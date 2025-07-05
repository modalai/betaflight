mkdir -p /home/4.1.0.4/tools/HEXAGON_Tools/8.4.05/Tools/bin/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi
cp src/platform/HEXAGON/scripts/fake-arm-gcc.bash /home/4.1.0.4/tools/HEXAGON_Tools/8.4.05/Tools/bin/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/arm-none-eabi-gcc
chmod a+x /home/4.1.0.4/tools/HEXAGON_Tools/8.4.05/Tools/bin/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/arm-none-eabi-gcc
export PATH=/home/4.1.0.4/tools/HEXAGON_Tools/8.4.05/Tools/bin:${PATH}
V=1 make TOOLS_DIR=/home/4.1.0.4/tools/HEXAGON_Tools/8.4.05/Tools/bin TARGET=HEXAGONV66
