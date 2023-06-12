#!/bin/bash

echo "Run sample project for controlling payload"

# rm /home/gremsy/run_sample.log
# rm /home/gremsy/run_sample.err

sample_dir_path=/home/gremsy/khoa-dev/PayloadSdk/build/sample_project/demo_project
sample_file_path=./demo_project

###
device_name=/dev/ttyUSB0
baudrate=115200

if [ -e ${device_name} ]; then	
	if [[ ! -x ${device_name} ]]; then
		echo "${device_name} is exist but can not use";
		sudo chmod u+x ${device_name} 
	fi
else
	echo "${device_name} is not exist"
	exit
fi


if [ -d ${sample_dir_path} ]; then
	# exec_file_path -d /dev/ttyUSB0 -b 115200
	echo "Open sample project directory";
	cd  ${sample_dir_path}
	echo "Check permission of file";
	if [ ! -x  ${sample_file_path} ]; then
		#statements
		echo "demo_project is not executable";
		chmod u+x ${sample_file_path}
	else
		echo "demo_project is executable";
	fi

	echo "Run file"
	${sample_file_path} --device ${device_name} --baud ${baudrate}
else
	echo "Not found sample project directory";
fi


echo "Exit program"
exit 

