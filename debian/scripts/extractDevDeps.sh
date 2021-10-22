#!/bin/bash

std_IFS=$IFS
for f in $(cd debian && ls libsdurw*-dev.install && cd ..); do
	lib=$(echo $f | awk -F '.' '{print $1}');
	file="debian/$lib.substvars"
        strline=$(grep INTERFACE_LINK_LIBRARIES debian/$lib/usr/share/robwork*/cmake/targets/*.cmake | awk -F '"' '{print $2}')
	IFS=';'
#	echo "$file"
	depsline="cmakedep:Depends="
	found=0
	for l in $strline; do
		rwsimstr="$(echo $l | cut -c1-7)"
		rwsstr="$(echo $l | cut -c1-5)"
		rwstr="$(echo $l | cut -c1-4)"
		libname=""
		if [[ "$rwsimstr" == "RWSIM::" ]]; then
			libname="$(echo $l | cut -c8-)"
		elif [[ "$rwsstr" == "RWS::" ]]; then
			libname="$(echo $l | cut -c6-)"
		elif [[ "$rwstr" == "RW::" ]]; then
			libname="$(echo $l | cut -c5-)"
			if [[ "$libname" == "pqp" ]]; then
				libname=""
			fi
		fi
		if [ ! -z "$libname" ]; then
			found=1
			libname="$(echo $libname | sed -r 's/_/-/')"
			depsline="${depsline}lib$libname-dev (=\${binary:Version}), "
		fi
	done
	IFS=$std_IFS
	if [[ found == 1 ]]; then
	        depsline=$(echo $depsline | rev | cut -c2- | rev)
	fi
	echo $depsline >> $file
#	echo $file
#	echo $depsline
#	echo ""
done
