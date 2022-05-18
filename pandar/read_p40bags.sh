#!/bin/bash
#转原始pandarpackets 数据为 Points 数据 
# ./pandar_converter input.bag out.bag 160.csv "/pandarp40_0/pandar_packets"
#$1 is the origin pandar_packetsbag path
#$2 is the new file for save pandar_points p40_0 bag
#$3 is the cali file.eg: 160.csv
#$4 is the new file for save pandar_points p40_2 bag 
#$5 is the cali file.eg: 162.csv

for ff in $(ls $1)
do 
echo $ff
#name=${ff%%*.}

./pandar_converter $1/$ff $2/$ff  $3 "/pandar40p_0/pandar_packets" &&\
./pandar_converter $1/$ff $4/$ff  $5 "/pandar40p_2/pandar_packets" 
done

