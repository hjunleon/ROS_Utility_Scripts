#!/usr/bin/env sh
input=$1
i=0 
bagFile=""
destDir=""
topicName=""
while IFS= read -r line;
do
    echo "$line"
    echo i: $i
    if  [ $i -eq 0 ];
    then 
        bagFile=$line

    elif [ $i -eq 1 ];
    then
        destDir=$line
        
    else
        topicName=$line
        rosbag info -y  -k topics "$bagFile" | cat
        roslaunch export.launch bagFile:="$bagFile" destDir:="$destDir" topicName:="$topicName"
        mv "~/.ros/frame*.jpg" destDir
        i=-1
    fi
    i=$(($i+1))
done < "$input"
