#!/bin/bash

config_pkg="single_room_objects"
num_locations=2
num_objects=2
echo "Config_pkg: $config_pkg"
echo "Number of locations: $num_locations"
echo "Number of objects: $num_objects"

# set path for results
path=$(rospack find experiments_evaluation)/eval
echo "PATH = $path"

for l in `seq 1 $num_locations`;
do
	for o in `seq 1 $num_objects`;
	do
		# skip already executed experiments
		loc="loc_$l"
		obj="_obj_$o.eval"
		fileName=$loc$obj
		file="$path/$fileName"

		#echo $file
		if [ -f $file ]; then
			echo "Skip computation for $fileName"
			continue
		fi
		
		# experiment not executed yet
		# start computation
		sh ./experiment.sh $config_pkg $l $o

		echo "wait 120 seconds for system to recover"
		seconds=120; date1=$((`date +%s` + $seconds)); 
		while [ "$date1" -ne `date +%s` ]; do 
		  echo -ne "$(date -u --date @$(($date1 - `date +%s` )) +%H:%M:%S)\r"; 
		done

	done
done

