#!/bin/bash

IMAGE_DIR="$(rospack find fast_livo)/Log/Colmap/images"
SPARSE_DIR="$(rospack find fast_livo)/Log/Colmap/sparse/0"

# Preserve timestamped image archive folders under Log/Colmap/images/.
# Only remove loose image files from the image root before a new run.
if [ -d "$IMAGE_DIR" ]; then
    find "$IMAGE_DIR" -maxdepth 1 -type f \( -iname '*.png' -o -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.bmp' -o -iname '*.tif' -o -iname '*.tiff' \) -delete
    echo "Cleaned image files in: $IMAGE_DIR"
else
    mkdir -p "$IMAGE_DIR"
    echo "Created: $IMAGE_DIR"
fi

if [ -d "$SPARSE_DIR" ]; then
    rm -rf "$SPARSE_DIR"
    echo "Removed: $SPARSE_DIR"
fi
mkdir -p "$SPARSE_DIR"
echo "Created: $SPARSE_DIR"
