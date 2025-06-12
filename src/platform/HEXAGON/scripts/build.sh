rm -f mk/local.mk
echo "TOOLS_DIR := /opt/hexagon-sdk/4.1.0.4-lite/tools/HEXAGON_Tools/8.4.05/Tools/bin" > mk/local.mk
echo "TARGET := HEXAGONV66" >> mk/local.mk
V=1 make