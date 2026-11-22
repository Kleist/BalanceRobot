#!/bin/sh
# Render the fit-test plate (step G1) and each of its pieces to STL.
# Usage: ./render.sh [OUT_DIR]   (default: ./out, which git ignores)
# Needs OpenSCAD 2021.01 or newer and the DejaVu fonts (labels).
# The preview picture in README.md (needs a display, or xvfb-run):
#   openscad -o fit_test.png --imgsize=1600,1000 --colorscheme=Tomorrow \
#       --camera=108,50,0,45,0,15,420 fit_test.scad
set -eu
cd "$(dirname "$0")"
out="${1:-out}"
mkdir -p "$out"
openscad -o "$out/fit_test.stl" fit_test.scad
for p in cradle hubs inserts board ports cell comb; do
    openscad -D "part=\"$p\"" -o "$out/fit_test_$p.stl" fit_test.scad
done
