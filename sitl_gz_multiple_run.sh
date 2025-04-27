#!/bin/bash
set -e
# --------------------------------------------------
# PX4 SITL Multi-Agent Launcher for Gazebo Harmonic
# ROS 2 Humble Compatible, 2025. 
# --------------------------------------------------

# Structure:
# 	function cleanup()
# 	function spawn_model()
# 	terminal handling scripts
# 	while loop for extracting command options
# 	default world and agent parameter declaration
#	Launch Gazebo Harmonic
# 	Vehicle Spawning

#------------------------
# Cleanup on exit
#------------------------
function cleanup()
{
    pkill -x px4
    pkill -x gz
}

#------------------------
# Cleanup on exit
#------------------------
function spawn_model()
{
	MODEL=$1 # Specified model
	N=$2 # Number of agents
	X=$3
	Y=$4
	X=${X:=0.0}
	Y=${Y:=$((3*${N}))} # Y co-ordinate is 3 times the no. of agents
	SUPPORTED_MODELS=("x500" "rc_cessna" "r1_rover")

	if [[ " ${SUPPORTED_MODELS[*]}" != *"${MODEL}"* ]];
	then
	    echo "ERROR: I am not able to work with the $MODEL model :/ !"
	    echo "Refer to the /Tools/simulation/gz/models directory. Hoping to get there soon..."
	    trap "cleanup" SIGINT SIGTERM EXIT
	    exit
	fi

	working_dir="$build_path/rootfs/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $N in $(pwd)"
	$build_path/bin/px4 -i $N -d "$build_path/etc" >out.log 2>err.log &

	# 'set --' is building a Python command line by adding arguments one-by-one. Hence the python3 ${@} command afterwards
	# here we don't need jinja to create the cookie cutter template
	# so we manually set the MAVLINK params for each model that is spawned
	# Manually set MAVLink ports and IDs for PX4
	export MAVLINK_TCP_PORT=$((4560 + ${N}))
	export MAVLINK_UDP_PORT=$((14560 + ${N}))
	export MAVLINK_ID=$((1 + ${N}))
	export MAVLINK_CAM_UDP_PORT=$((14530 + ${N}))

	# Here we shall spawn {MODEL}
	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"
	gz model --spawn-file=${src_path}/Tools/simulation/gz/models/${MODEL}.sdf --model-name=${MODEL}_${N} -x ${X} -y ${Y} -z 0.83

	popd &>/dev/null
}

#------------------------
# Default Parameters
#------------------------
# Setting the default states - no. of cookies, cookie tray to use, oven, cookie shape lol
num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
target=${TARGET:=px4_sitl} #the baking oven haha
vehicle_model=${VEHICLE_MODEL:="x500"}


pose_exists="false"
if [[ -n "${POSE_MAP}" ]]; then
  # Split the map into individual key-value pairs using a comma as the delimiter
  IFS='|' read -ra pose_map <<< "$POSE_MAP"
  pose_exists="true"
fi

export PX4_SIM_MODEL=${vehicle_model} #cuz gz launches files like make px4_sitl gz_x500


#------------------------
# Paths and Environment
#------------------------
echo "[INFO] SCRIPT parameter is: ${SCRIPT}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../.."

build_path=${src_path}/build/${target}
mavlink_udp_port=14560
mavlink_tcp_port=4560

# Kill previous px4 instances
echo "[INFO] Killing any running PX4 instances..."
pkill -x px4 || true
sleep 1

# we are now sourcing the environment like source install/setup.bash or source devel/setup.bash...
# except there is no setup.bash in the directory, so source gz

export IGN_GAZEBO_RESOURCE_PATH=${src_path}/Tools/simulation/gz/models:$IGN_GAZEBO_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=${src_path}/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH

# To use gazebo_ros ROS2 plugins
if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
	ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
else
	ros_args=""
fi


#------------------------
# Launch Gazebo Harmonic
#------------------------
echo "Starting Gazebo (Harmonic)"
# Notice: using .sdf file now from the 'gz' folder
gz sim ${src_path}/Tools/simulation/gz/${world}.sdf --verbose --gui $ros_args &


#-------------------------------
#        Spawn Vehicles
#-------------------------------
n=0
if [ -z "${SCRIPT}" ]; then
	if [ $num_vehicles -gt 255 ]
	then
		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
		exit 1
	fi

	while [ $n -lt $num_vehicles ]; do
		instance=$(($n + 1))
		if [[ "${pose_exists}" == "true" ]]; then
			pose="${pose_map[$n]}"
			if [[ -n "$pose" ]]; then
				IFS=',' read -r x y <<< "$pose"
				spawn_model ${vehicle_model} ${instance} "${x}" "${y}"
			else
				spawn_model ${vehicle_model} ${instance}
			fi
		else
			spawn_model ${vehicle_model} ${instance}
		fi
		n=${instance}
	done
else
	IFS=,
	for target in ${SCRIPT}; do
		target="$(echo "$target" | tr -d ' ')" #Remove spaces
		target_vehicle=$(echo "$target" | cut -f1 -d:)
		target_number=$(echo "$target" | cut -f2 -d:)

		if [ $n -gt 255 ]
		then
			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi

		m=0
		while [ $m -lt "${target_number}" ]; do
			export PX4_SIM_MODEL=${target_vehicle}
			instance=$(($n + 1))
			# We can now specify a pose for each instance and if none is provided, we use default location
			if [[ "${pose_exists}" == "true" ]]; then
				echo $n
				pose="${pose_map[$n]}"
				if [[ -n "$pose" ]]; then
					IFS=',' read -r x y <<< "$pose"
					spawn_model "${target_vehicle}""${LABEL}" ${instance} "${x}" "${y}"
				else
					spawn_model "${target_vehicle}""${LABEL}" ${instance}
				fi
			else
				spawn_model "${target_vehicle}""${LABEL}" ${instance}
			fi
			n=${instance}
			m=$(($m + 1))
		done
	done

fi
trap "cleanup" SIGINT SIGTERM EXIT
