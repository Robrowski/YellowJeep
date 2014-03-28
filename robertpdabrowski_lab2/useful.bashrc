# .bashrc

# Source global definitions
if [ -f /etc/bashrc ]; then
	. /etc/bashrc
fi

# User specific aliases and functions

#################
## RBE 3002
# Utility script for ros to configure ros master and workspace
# Settings are percisted through two settings files configured below
# Author: Mitchell Wills
 
 
# Configuration
declare -A robots=(
    ["hulk"]="192.168.1.2"
    ["ironman"]="192.168.1.3"
    ["captainamerica"]="192.168.1.4"
    ["thor"]="192.168.1.5"
    ["hawkeye"]="192.168.1.6"
    ["blackwidow"]="192.168.1.7"
    )   # List of robot names and their ip addresses
settingsFile="$HOME/.rossetup-settings"  # File name to store env vars
rosVersion="hydro" #the ros version used in hack workspaces
 
 
 
function is_ip(){
    local input=$1
    if [[ $input =~ ([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3}) ]]; then
	local ip=(${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]})
	for i in "${ip[@]}"
	do
	    if [[ $i -lt 0 || $i -gt 255 ]]; then
		return 1;
	    fi
	done
	
	return 0;
    else
	return 1;
    fi
}
 
 
function __rossetup-set-robot-ip(){
    local ip=$1
    export ROS_MASTER_URI=http://${ip}:11311
    export rossetup_robot_ip_config=$ip
    export rossetup_robot_ip_command="__rossetup-set-robot-ip $ip"
}
 
function __rossetup-set-local-hostname(){
    local hostname=$1
    export ROS_HOSTNAME=$hostname
    export rossetup_local_ip_config="hostname=$hostname"
    export rossetup_local_ip_command="__rossetup-set-local-hostname $hostname"
}
function __rossetup-set-local-interface(){
    local interface=$1
    
    # check if the interface exists
    local found=`ifconfig | egrep "^$interface\s"`
 
    # if the interface was found
    if [[ ! -z $found ]]; then
        # extract the ip address of an interface and cut addr:from the begining
	local my_ip=`ifconfig $interface | egrep -o 'addr:[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' | cut -c6- `
	if [ -z "$my_ip" ]; then
	    echo -ne "\E[33mWARNING:"; tput sgr0
	    echo " Unable to find an ip address for the interface: $interface"
	    echo "         Make sure that you are connected and have an ip address"
	    echo "         Defaulting hostname to localhost"
	    export ROS_HOSTNAME="localhost"
	else
	    export ROS_HOSTNAME=$my_ip
	fi
    else
	echo -ne "\E[33mWARNING:"; tput sgr0
	echo " Unable to find the interface: $interface"
	echo "         Defaulting hostname to localhost"
	export ROS_HOSTNAME="localhost"
    fi
    export rossetup_local_ip_config="iface=$interface"
    export rossetup_local_ip_command="__rossetup-set-local-interface $interface"
}
function __rossetup-set-local-auto(){
 
    if [ "$rossetup_robot_ip_config" = "localhost" ]; then
	__rossetup-set-local-hostname "localhost"
    else
        # make : seperated list of active interfaces excluding lo
	local connected_ifs=`ifconfig | egrep -B1 'addr:[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' | egrep -o '^\S+' | sed '/--/d' | sed '/lo/d' | tr '\n' ':' | sed -e 's/:$//'`
        # if the active connections list is not empty
	if [[ ! -z $connected_ifs ]]; then
	    local connected_count=`echo $connected_ifs | tr -cd ':' | wc -c`
	    local interfaces=()
	    eval `echo $connected_ifs | awk 'BEGIN{FS=":"}{for (i=1; i<=NF; i++) print "interfaces["i-1"]=\""$i"\""}'`
 
	    declare -A if_ips
	    local subnet_ifs=()
	    
	    for interface in "${interfaces[@]}"
	    do
		local if_ip_str=`ifconfig $interface | egrep -o 'addr:[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' | cut -c6- `
		local if_mask_str=`ifconfig $interface | egrep -o 'Mask:[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' | cut -c6- `
		local robot_ip_str;
		if ! is_ip $rossetup_robot_ip_config; then
		    local tmp_robot_ip=`host $rossetup_robot_ip_config | grep -m1 "has address" | awk '{print $NF}'`
		    if is_ip $tmp_robot_ip; then
			robot_ip_str=$tmp_robot_ip
		    else
			echo -ne "\E[33mWARNING:"; tput sgr0
			echo " Could not determine the robot ip address from the hostname"
		    fi
		else
		    robot_ip_str=$rossetup_robot_ip_config
		fi
 
		if_ips[$interface]=$if_ip_str
		
	    # turn interface ip address and mask as well as the robot's ip into arrays of the octets
		[[ $if_ip_str =~ ([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3}) ]];
		local if_ip=(${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]})
		[[ $if_mask_str =~ ([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3}) ]];
		local if_mask=(${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]})
		[[ $robot_ip_str =~ ([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3}) ]];
		local robot_ip=(${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]})
 
	    #determine if subnets match
		local is_same=1
		local i=0
		while [  $i -lt 4 ]; do
		    local if_subnet=$(( ${if_ip[$i]} & ${if_mask[$i]} ));
		    local robot_subnet=$(( ${robot_ip[$i]} & ${if_mask[$i]} ));
		    if [[ $if_subnet != $robot_subnet ]]; then
			is_same=0
		    fi
		    let i=i+1
		done
		if [[ $is_same != 0 ]]; then
		    subnet_ifs+=($interface)
		fi
 
	    done
 
	    
	    if [[ ${#subnet_ifs[@]} > 0 ]]; then
		if [[ ${#subnet_ifs[@]} > 1 ]]; then
		    echo -ne "\E[33mWARNING:"; tput sgr0
		    echo " More than one active interface is on the robots subnet"
		    echo "         Active connections on the robot's subnet are: (${subnet_ifs[*]})"
		    echo "         Choosing the first active interface (${subnet_ifs[0]})"
		fi
		export ROS_HOSTNAME=${if_ips[${subnet_ifs[0]}]}
		export rossetup_local_ip_config="auto,iface=${subnet_ifs[0]}"
	    else
		echo -ne "\E[33mWARNING:"; tput sgr0
		echo " No active interfaces found on the robots subnet"
		if [[ ${#interfaces[@]} > 1 ]]; then
		    echo -ne "\E[33mWARNING:"; tput sgr0
		    echo " More than one active interface found"
		    echo "         Active connections are: (${interfaces[*]})"
		    echo "         Choosing the first active interface (${interfaces[0]})"
		fi
		export ROS_HOSTNAME=${if_ips[${interfaces[0]}]}
		export rossetup_local_ip_config="auto,iface=${interfaces[0]}"
	    fi
 
 
    # no active connections
		else
		    echo -ne "\E[33mWARNING:"; tput sgr0	echo " No active interfaces found, defaulting to localhost"
		    export ROS_HOSTNAME="localhost"
		    export rossetup_local_ip_config="auto,local"
		fi
		export rossetup_local_ip_command="__rossetup-set-local-auto"
	fi
}
function __rossetup-set-local(){
    __rossetup-set-robot-ip "localhost"
    __rossetup-set-local-hostname "localhost"
}
 
function __rossetup-set-workspace(){
    local workspace=$1
	# if setup file exists (is a ros overlay) then run it's setup.bash
    if [ -f "$workspace/setup.bash" ]; then
	source "$workspace/setup.bash";
	# catkin workspace
    elif [ -f "$workspace/devel/setup.bash" ]; then
	source "$workspace/devel/setup.bash";
	export ROS_WORKSPACE="$workspace/src"
    else
	if [ -d "$workspace" ];then
	    echo "Did not detect ROS workspace, adding directory to the package path"
	else
	    echo -ne "\E[33mWARNING:"; tput sgr0
	    echo " the workspace directory does not exist"
	fi
	source "/opt/ros/$rosVersion/setup.bash"
	export ROS_PACKAGE_PATH=$workspace:$ROS_PACKAGE_PATH
	export ROS_WORKSPACE=$workspace
    fi
 
    export rossetup_workspace_config="$workspace"
    export rossetup_workspace_command="__rossetup-set-workspace $workspace"
}
function __rossetup-clear-workspace(){
    source "/opt/ros/$rosVersion/setup.bash"
    export ROS_WORKSPACE=
    export rossetup_workspace_config="none"
    export rossetup_workspace_command="__rossetup-clear-workspace"
}
 
function __rossetup-save-config(){
    echo -e "$rossetup_robot_ip_command\n$rossetup_local_ip_command\n$rossetup_workspace_command" > "$settingsFile"
    echo "Saved config"
}
 
function rossetup-print-help(){
    echo
    echo -e "\033[1mSYNOPSIS\033[0m"
    echo -e "\t\033[1mrobot\033[0m"
    echo -e "\t\033[1mrobot\033[0m \033[1m-h\033[0m"
    echo -e "\t\033[1mrobot\033[0m \033[4mmaster\033[0m [\033[4minterface\033[0m]"
    echo -e "\t\033[1mrobot\033[0m \033[4mmaster\033[0m [\033[1m--host\033[0m \033[4mhostname\033[0m]"
    echo
    echo -e "\033[1mDESCRIPTION\033[0m"
    echo -e "\t\033[1mrobot\033[0m is a tool for automatically configuring ros in a bash terminal."
    echo -e "\tCalling \033[1mrobot\033[0m without any arguments can be used to reload the configuration. This can be used if the network configuration has changed."
    echo
    echo -e "\033[1mOPTIONS\033[0m"
    echo -e "\t\033[1m-h\033[0m"
    echo -e "\t\tShow this help message"
    echo -e "\t\033[4mmaster\033[0m"
    echo -e "\t\tSpecify the master as either an ip address, hostname or one of the following named robot addresses (${!robots[@]})"
    echo -e "\t\tThis address will be expanded to be the ROS_MASTER_URI on port 11311"
 
    echo -e "\t\033[4minterface\033[0m"
    echo -e "\t\tSpecify the interface through which communication can be achieved to this computer from the robot/other machines"
    echo -e "\t\tWill try to automatically detect the ip address of this interface and use that as ROS_HOSTNAME"
    echo -e "\t\tIf the interface is set to auto then the interface to use is determined by looking at the ip and subnet of each interface"
    echo -e "\t\tIf no interface is specified then it will default to auto"
 
    echo -e "\t\033[1m--host\033[0m \033[4mhostname\033[0m"
    echo -e "\t\tSpecify the ip address or hostname through which this computer can be reached from the robot/other machines"
    echo -e "\t\tThis value will be used exactly as ROS_HOSTNAME"
}
 
 
 
 
# This function sets env vars in the current terminal and updates the config file for conecting to the robots
function robot() {
    # if there was no argument
    if [[ -z "$1" ]]; then
	#if there is a settings file
	if [ -f "$settingsFile" ]; then 
	    source "$settingsFile"
	    ros-quick-status
	else
            # setup for local operation if no config file
	    robot local;
	fi
    # if the argument was -h then show help
    elif [[ "$1" = "-h" ]]; then
	rossetup-print-help
    # otherwise search for other argument types
    else
	
       	# check if the arg was a robot name
	if [[ ${robots[$1]} ]]; then
	    __rossetup-set-robot-ip ${robots[$1]}
	#Setup for local use
	elif [ "$1" = "local" ]; then
	    __rossetup-set-local
	# otherwise the arg was an ip address
	else
	    __rossetup-set-robot-ip $1
	fi
 
	# calculate current computer's ip address and export master and hostname
	# if there is a second argument
	if [ ! -z "$2" ]; then
	    # if the second argument is "--host"
	    if [ "$2" = "--host" ]; then
		local hostname="$3"
		__rossetup-set-local-hostname $hostname
	    elif [ "$2" = "auto" ]; then
		__rossetup-set-local-auto
	    else
		local interface="$2"
		__rossetup-set-local-interface $interface
	    fi
	else
	    __rossetup-set-local-auto
	fi
 
	__rossetup-save-config
	ros-quick-status
    fi
	 
}
 
 
# This function sets env vars in the current terminal and updates the config file for the ros workspace
function ros-setup-workspace(){
    if [ ! -z "$1" ]; then
        #make sure to expand a reletive path
	__rossetup-set-workspace `readlink -f $1`
	__rossetup-save-config
	ros-quick-status
    else
	__rossetup-clear-workspace
	__rossetup-save-config
	ros-quick-status
    fi
}
 
function ros-build-workspace(){
    (roscd && cd .. && catkin_make)
}
 
# This function prints out the status of important ros variables and full configuration details
function ros-status(){
    local robot_ip=$rossetup_robot_ip_config
    local robot_host=""
    # if the robot hostname is not an ip address then try to resolve it to an ip
    if ! is_ip $rossetup_robot_ip_config ; then
	local tmp_robot_ip=`host $rossetup_robot_ip_config | grep -m1 "has address" | awk '{print $NF}'`
	if is_ip $tmp_robot_ip; then
	    robot_ip=$tmp_robot_ip
	fi
    else
	local host_out=`host $rossetup_robot_ip_config`;
	#if host was able to do a reverse lookup on the ip
	if [[ $host_out != *"not found"* ]]; then
	    #extract the hostname and chop off the last byte which is a period
	    robot_host=`host $rossetup_robot_ip_config | awk '{print $NF}' | sed 's/.\{1\}$//'`
	fi
    fi
 
 
    local host_ip=$ROS_HOSTNAME
    local host=""
    local host_if=""
    # if the hostname is not an ip address then try to resolve it to an ip
    if ! is_ip $ROS_HOSTNAME ; then
	local tmp_host_ip=`host $ROS_HOSTNAME | grep -m1 "has address" | awk '{print $NF}'`
	if is_ip $tmp_host_ip; then
	    host_ip=$tmp_host_ip
	fi
    else
	local host_out=`host $ROS_HOSTNAME`;
	#if host was able to do a reverse lookup on the ip
	if [[ $host_out != *"not found"* ]]; then
	    #extract the hostname and chop off the last byte which is a period
	    host=`host $ROS_HOSTNAME | awk '{print $NF}' | sed 's/.\{1\}$//'`
	fi
    fi
    if is_ip $host_ip; then
	host_if=`ifconfig | grep -B1 "$host_ip" | head -n1 | awk '{print $1}'`
    fi
 
    echo -ne "ROS_MASTER_URI="
    echo -ne "\E[1;34m$ROS_MASTER_URI"; tput sgr0
    if [ ! -z $robot_ip ] && ! is_ip $rossetup_robot_ip_config; then
	echo -ne " (ip=$robot_ip)"
    fi
    if [ ! -z $robot_host ]; then
	echo -ne " (host=$robot_host)"
    fi
    echo -e " \E[36m[$rossetup_robot_ip_config]"; tput sgr0
 
 
    echo -ne "ROS_HOSTNAME="
    echo -ne "\E[1;34m$ROS_HOSTNAME"; tput sgr0
    if [ ! -z $host_ip ] && ! is_ip $ROS_HOSTNAME; then
	echo -ne " (ip=$host_ip)"
    fi
    if [ ! -z $host ]; then
	echo -ne " (host=$host)"
    fi
    if [ ! -z $host_if ]; then
	echo -ne " (iface=$host_if)"
    fi
    echo -e " \E[36m[$rossetup_local_ip_config]"; tput sgr0
 
    echo -n "ROS_WORKSPACE=$ROS_WORKSPACE"
    echo -e " \E[36m[$rossetup_workspace_config]"; tput sgr0
 
    echo "ROS_DISTRO=$ROS_DISTRO"
 
    echo "ROS_PACKAGE_PATH="
    echo $ROS_PACKAGE_PATH | awk -F':' '{ for(i=1;i<=NF;i++){printf "\t%s\n", $i}; }'
    
    echo
    # ping the robot 4 times
    ping -A -c 4 -W 1 $rossetup_robot_ip_config
}
# This function prints out the status of important ros variables
function ros-quick-status(){
    echo -ne "ROS_MASTER_URI="
    echo -ne "\E[1;34m$ROS_MASTER_URI"; tput sgr0
    echo -e " \E[36m[$rossetup_robot_ip_config]"; tput sgr0
 
 
    echo -ne "ROS_HOSTNAME="
    echo -ne "\E[1;34m$ROS_HOSTNAME"; tput sgr0
    echo -e " \E[36m[$rossetup_local_ip_config]"; tput sgr0
 
    echo -n "ROS_WORKSPACE=$ROS_WORKSPACE"
    echo -e " \E[36m[$rossetup_workspace_config]"; tput sgr0
}
 
source "/opt/ros/$rosVersion/setup.bash"
# load configuration from files if they exist
robot


######################## 
# Sourcing ROS
#source ~/catkin_ws/devel/setup.bash
# the right call
# "set-workspace ~/catkin_ws/"   
# Then to robot local for local
# and    robot AVENGER auto
######################################################
# Default editor for 'rosed' = gedit
export EDITOR='gedit'
######################################################
# Setup for git
git config --global user.name 'Robrowski'
git config --global user.email rpdabrowski@wpi.edu
# cache username/PW for 5 hours
git config --global credential.helper 'cache --timeout=18000'  


######################################################
# Random Aliases
alias clean='rm -rf *~ *.*~'
alias bashrc='gedit ~/.bashrc &'
alias svBash='cat ~/.bashrc > useful.bashrc'
alias home='cd ~/catkin_ws/'
alias py='chmod +x *.py' # make all scripts executable


# ROS aliases
alias build='cd ~/catkin_ws ; catkin_make'
alias bag='rosbag record -o'
alias topic_graph='rosrun rqt_graph rqt_graph'
alias save_map='cd ~/catkin_ws/maps; rosrun map_server map_saver -f'


# ROS TurtleBot sim
alias TurtleBotSim='roslaunch turtlebot_gazebo turtlebot_empty_world.launch'
alias TurtleBotKey='roslaunch turtlebot_teleop keyboard_teleop.launch'
alias TurtleBotViz='roslaunch turtlebot_rviz_launchers view_robot.launch'
alias TurtleBotMap='roslaunch turtlebot_gazebo gmapping_demo.launch'
alias TurtleBotSimPlayground='roslaunch turtlebot_gazebo turtlebot_playground.launch'
				      
# ROS Real Turtlebot
alias TurtleRunMinimal='roslaunch turtlebot_bringup minimal.launch'

## Git aliases
alias poop='git'
alias gitHome='cd ~/catkin_ws/src/YellowJeep/'
alias gitLogout='git credential-cache exit'
alias commit='git commit -m'
alias status='git status'
alias push='git push origin'
alias pull='git pull origin'


# ex multiline # note the semicolon 
#alias pl='pwd; ls'  
######################################################
# SSH into turtle bot
#            ["hulk"]="192.168.1.2"
#         ["ironman"]="192.168.1.3"
#  ["captainamerica"]="192.168.1.4"
#  ["thor"]="192.168.1.5"
# ["hawkeye"]="192.168.1.6" works
#  ["blackwidow"]   works


function turtleLogin(){
    echo "Logging into: " $1	    
    echo  ${robots[$1]}
    ssh $1@${robots[$1]}
    expect $1"@"${robots[$1]}"'s password: " #rest isn't working
    send "turtlebot"
}



