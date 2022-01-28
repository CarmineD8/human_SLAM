#!/bin/bash
i=0
for filename in /home/mike/catkin_ws/src/recognition/src/Recognition_from_Ontology/Data/*.txt; 
do 
   filename=$(basename $filename)
   rosparam set /current_filename "$filename"
   for j in {1..10}
   do
      now=$(date +"%T")
      logpath=/home/mike/Output/Corrected/$filename/
      mkdir -p $logpath/logs

      roslaunch nav2d_tutorials from_vicoli_bag_automated.launch filename:=$filename Onto:=true >> $logpath/logs/$now
   
   done

i=$i+1

done

