#!/bin/bash
# toolkit2docker.bash

# Checks if the temp folder exits
if [ ! -d "$HOME/.toolkit_tmp/" ]
then
	echo "Toolkit tmp folder does not exists."
	mkdir $HOME/.toolkit_tmp
else
	echo "Toolkit tmp folder exists"
fi

# Checks if a toolkit tar.gz exits
if [ -f "$HOME/.toolkit_tmp/toolkit_ws.tar.gz" ]
then
	# Removes file if exits
	echo "File exits"
	rm -rf .toolkit_tmp/toolkit_ws.tar.gz
else
	echo "File does not exits"
fi

tar -czf $HOME/.toolkit_tmp/toolkit.tar.gz toolkit_ws/
docker cp $HOME/.toolkit_tmp/toolkit.tar.gz 37722207bd14:/home/nao/toolkit_ws.tar.gz
