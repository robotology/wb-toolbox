#!/bin/bash
# Script for reporting template issues of the WBI-Toolbox
# Author: Jorhabib  Eljaik
# iCub Facility, Istituto Italiano di Tecnologia - 2015

set -e
WBI_TOOLBOX_ISSUE_URL=${WBI_TOOLBOX_ISSUE_URL:-"https://github.com/robotology/WB-Toolbox/issues/new"}
WBI_TOOLBOX_ISSUE_NAME_DEFAULT_TITLE=${WBI_TOOLBOX_ISSUE_NAME_DEFAULT_TITLE:-"[branch_name]: Title of your issue"}

# pulled from https://gist.github.com/cdown/1163649
function urlencode() {
	# urlencode <string>

	local length="${#1}"
	for (( i = 0; i < length; i++ )); do
			local c="${1:i:1}"
			case $c in
					[a-zA-Z0-9.~_-]) printf "$c" ;;
					*) printf '%%%02X' "'$c"
			esac
	done
}

function template() {
	cat<<-	EOM
	**Description:**
	Please describe your problem here...

	_____

	**Further Information**:
	- *System Information:* \``uname -a`\`
	- *Working branch:* \``git rev-parse --abbrev-ref HEAD`\`
	- *Current commit:* \``git rev-parse HEAD`\`
	- *YARP ROOT:* \``echo $YARP_ROOT`\`
	- *ICUB_ROOT:* \``echo $ICUB_ROOT`\`
	- *YARP_DATA_DIRS:* \``echo $YARP_DATA_DIRS`\`
	- *YARP_ROBOT_NAME:* \``echo $YARP_ROBOT_NAME`\`
	- *Yarp Installation:* \``which yarpserver`\`
	- *LD_LIBRARY_PATH:*\``echo $LD_LIBRARY_PATH`\`
	- *LD_PRELOAD:*\``echo $LD_PRELOAD`\`
	- *DYLD_INSERT_LIBRARIES:*\``echo $DYLD_INSERT_LIBRARIES`\`
EOM
}

function format_issue_url() {
	if [ ${#@} -ne 2 ] ; then
		return 1
	fi
	local issue_name=$(urlencode "${1}")
	local issue_body=$(urlencode "${2}")
	echo "${WBI_TOOLBOX_ISSUE_URL}?title=${issue_name}&body=${issue_body}"
}

echo -ne "Title of new issue: "
read -r issue_title
echo ""

issue_url=$(format_issue_url "${issue_title}" "$(template)")

if [[ $OSTYPE  == "linux-gnu" ]]; then
	if which xdg-open 2>/dev/null >/dev/null ; then
		read -p "To finish filing this issue it will be launched on your preferred browser. Do you want to proceed? [Y|n]: `echo $'\n> '`" reply
		if [ "${launch_now}" != "n" -a "${launch_now}" != "N" ]; then
			xdg-open "${issue_url}"
		fi
	fi
elif [[ $OSTYPE == "darwin"* ]]; then
	if which open 2>/dev/null >/dev/null ; then
		read -p "To finish filing this issue it will be launched on your preferred browser. Do you want to proceed? [Y|n]: `echo $'\n> '`" reply
		if [ "${launch_now}" != "n" -a "${launch_now}" != "N" ]; then
			open "${issue_url}"
		fi
	fi
fi
