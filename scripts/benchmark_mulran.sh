#!/bin/bash
DATA_ROOT="/home/li/data/hilti-2022"
OUTPUT_ROOT="/home/li/data/mm_balm_benchmark/data/hilti2022"
SOURCE_DIR="/home/li/program/mm-balm-ws/src/FAST_LIO"

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

# # Town 01 Ouster
# cd /home/weihairuo/Workspaces/FastLio_ec_ws
# source ./devel/setup.bash
# roslaunch fast_lio mapping_helipr_ouster.launch dataset:=Town seq:=Ouster01 & 
# sleep 5s
# cd /home/weihairuo/Workspaces/helipr_player
# source ./devel/setup.bash
# roslaunch file_player file_player.launch autoplay:=true file:=/media/weihairuo/T9/Town/01-O24140-V23935-A24141 speed:=2.0
# sleep 5s
# rosnode kill /laserMapping