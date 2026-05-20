#!/usr/bin/env bash
set -euo pipefail

readonly tools_dir=/home/4.1.0.4/tools/HEXAGON_Tools/8.4.05/Tools/bin

PATH="${tools_dir}:${PATH}" \
    make V=1 TOOLS_DIR="${tools_dir}" TARGET=HEXAGONV66
