#!/bin/bash
input=$1
IFS=''
SAVEIFS=$IFS
echo $HOME
while read -r line;
do
    #echo $line
    #
    #echo ls $line/*.bag
    #bagFiles=$(ls $line/*.bag)
    #echo $bagFiles
    #IFS='\n'
    #while  read -r bagFile;
    #for (( j=0; i<${#bagFiles[@]}; j++))
    for bagFile in $line/*.bag;
    do
        # echo "j: $j"
        # bagFile=${bagFiles[$j]}
        # echo $bagFile
        # echo $(basename $bagFile )
        baseName=$(basename $bagFile )
        outputDir="${bagFile%.*}"
        # echo $outputDir

        mkdir "$outputDir"

        topic=$(rosbag info -y  -k topics $bagFile | grep topic: | cut -d' ' -f3)
        msgType=$(rosbag info -y  -k topics $bagFile | grep type: | cut -d' ' -f4)
        
        IFS=$'\n'
        echo "$msgType"
        msgType=($msgType)
        topic=($topic)
        for (( i=0; i<${#msgType[@]}; i++))
        do
            echo "$i: ${msgType[$i]}"
            echo "$i: ${topic[$i]}"
            subDir=${topic[$i]}
            subDir=${subDir//\//_}
            mkdir "$outputDir/$subDir"
            destDir="$outputDir/$subDir"

            echo "\"$bagFile\""
            
            
            if  [ "${msgType[$i]}" = "sensor_msgs/CompressedImage" ];
            then 
                echo "exporting compressed image"
                lastPart=$(basename ${topic[$i]})
                lastPart=${lastPart,,}
                echo "cur_topic suffix is: $lastPart"
                cur_topic=${topic[$i]}
                if [ $lastPart = "compressed" ];
                then
                    echo "compresse in name"
                    cur_topic=$(dirname ${topic[$i]})
                fi
                echo "cur_topic name is: $cur_topic"
                roslaunch export.launch bagFile:="\"$bagFile\"" destDir:="$destDir" topicName:="$cur_topic"
            elif [ "${msgType[$i]}" = "sensor_msgs/Image" ];
            then 
                echo "exporting raw"
                cur_topic=${topic[$i]}
                roslaunch exportRaw.launch bagFile:="\"$bagFile\"" destDir:="$destDir" topicName:="$cur_topic"
            fi
            mv $HOME/.ros/frame*.jpg $outputDir/$subDir
        done
        #echo ${msgType[0]}
        

    done

    IFS=$SAVEIFS

done < "$input"