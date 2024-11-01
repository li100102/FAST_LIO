#!/bin/bash
DATA_ROOT="/home/li/data/mars_dataset"
OUTPUT_ROOT="/home/li/data/mm_balm_benchmark/data/mars_lvig"
SOURCE_DIR="/home/li/program/mm-balm-ws/src/FAST_LIO"

mkdir -p $OUTPUT_ROOT

for file in "$DATA_ROOT"/*.bag; do
    if [[ -f "$file" ]]; then
        # Process the file here
        FILE_NAME=$(basename "$file" .bag)
        echo "Processing file: $file"
        # Add your code to process the file
        
        mkdir -p "$OUTPUT_ROOT/$FILE_NAME"
        roslaunch fast_lio benchmark.launch bag:=$file
        cp $SOURCE_DIR/PCD/* "$OUTPUT_ROOT/$FILE_NAME"
        rm -r $SOURCE_DIR/PCD/*
        cp $SOURCE_DIR/Log/pose.txt "$OUTPUT_ROOT/$FILE_NAME"
    fi
done