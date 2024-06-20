#!/bin/bash

# Arrays to store navigation launch commands
navigation_files=("multi_robot_navigation_navfn_nofp.launch.py" "multi_robot_navigation_navfn_fp.launch.py")
navigation_files_payload=("multi_robot_navigation_navfn_fp.launch_payload.py" "multi_robot_navigation_navfn_nofp.launch_payload.py")

# Function to restart Gazebo and ROS2
restart_gazebo_and_ros() {
    echo "Restarting Gazebo and ROS2..."
    pkill -9 gzserver
    pkill -9 gzclient
    pkill -9 ros2
    sleep 2
    # gzserver &
    # sleep 2
}

# Function to close all open gnome-terminal windows
close_all_gnome_terminals() {
    echo "Cerrando todas las consolas de gnome-terminal abiertas..."
    pkill -f gnome-terminal
}

# Function to clean cache or temporary files
clean_cache() {
    echo "Cleaning cache and temporary files..."
    rm -rf ~/.gazebo/log/*
    rm -rf /tmp/gazebo/*
    # rm -rf /tmp/ros2/*
}

# Function to capture the PGID
capture_pgid() {
    local pid=$1
    if [[ -z "$pid" ]]; then
        echo "Invalid PID: $pid"
        exit 1
    fi
    local pgid=$(ps -o pgid= -p $pid | tr -d ' ' | grep -E '^[0-9]+$')
    if [[ -z "$pgid" ]]; then
        echo "Error capturing PGID for PID: $pid"
        exit 1
    fi
    echo $pgid
}

# Function to launch processes
launch_and_capture_pgid() {
    local command=$1
    local pid_file=$2
    echo "Launching command: $command"
    gnome-terminal -- bash -c "$command & echo \$! > $pid_file; wait" &
    sleep 2  # Wait for the terminal and process to start

    if [ -f $pid_file ]; then
        local pid=$(cat $pid_file)
        echo "Captured PID: $pid"
        PGID=$(capture_pgid $pid)
        echo $PGID
    else
        echo "Error capturing PID for command: $command"
        exit 1
    fi
}

# Main loop for each of the 6 scenarios
for scenario in {1..6}; do
    for nav_type in {0..1}; do  # 0 for nofp, 1 for fp
        if [[ $scenario -le 4 ]]; then
            nav_file=${navigation_files[$nav_type]}
        else
            nav_file=${navigation_files_payload[$nav_type]}
        fi
        nav_label=$(echo $nav_file | cut -d'_' -f5)  # extract 'nofp' or 'fp'

        # Repeat each navigation type 5 times
        for repetition in {1..5}; do

            echo -e "**************\nSTARTING TEST\n**************"

            echo "Starting Scenario $scenario, Navigation $nav_label, Repetition $repetition..."

            # Array for PGIDs
            pgids=()

            # Restart Gazebo and clean cache before each simulation
            restart_gazebo_and_ros
            clean_cache

            echo "Waiting 10 seconds to ensure complete cleanup..."
            sleep 10

            # Launch navigation process
            echo "Launching navigation process with file: $nav_file"
            PGID_NAV=$(launch_and_capture_pgid "ros2 launch neo_simulation2 $nav_file" "/tmp/ros2_pid.txt")
            pgids+=($PGID_NAV)

            echo "Waiting 3 seconds for navigation process initialization..."
            sleep 3

            # Launch corresponding simulation scenario
            echo "Launching simulation scenario: exp${scenario}.launch.py"
            PGID_SIM=$(launch_and_capture_pgid "ros2 launch neo_simulation2 exp${scenario}.launch.py; sleep 3; exit" "/tmp/sim_pid.txt")
            pgids+=($PGID_SIM)

            echo "Waiting 5 seconds before launching rviz..."
            sleep 5

            # Launch RViz after the simulation
            echo "Launching RViz..."
            PGID_RVIZ=$(launch_and_capture_pgid "ros2 launch neo_nav2_bringup rviz_launch.py rviz_config:=install/neo_nav2_bringup/share/neo_nav2_bringup/rviz/multi_robot.rviz" "/tmp/rviz_pid.txt")
            pgids+=($PGID_RVIZ)

            echo "Waiting 5 seconds before launching initial poses..."
            sleep 5

            # Launch initial poses script after RViz
            echo "Launching initial poses for scenario $scenario..."
            PGID_INITIAL_POSES=$(launch_and_capture_pgid "bash ~/ros2_humble_wss/thesis_wsp/start_scripts/generate_initialposes_scenario${scenario}.sh; sleep 6; exit" "/tmp/initial_poses_pid.txt")
            pgids+=($PGID_INITIAL_POSES)

            echo "Waiting 8 seconds before launching data collection nodes..."
            sleep 8

            # Collect data for each robot before launching goals
            for robot_id in {0..2}; do
                echo "Collecting data for robot${robot_id}..."
                data_filename="exp_data_${nav_label}_experiment${scenario}_repetition${repetition}_robot${robot_id}.csv"
                data_file_path="~/ros2_humble_wss/thesis_wsp/exp_data/${data_filename}"
                PGID_DATA=$(launch_and_capture_pgid "python3 ~/ros2_humble_wss/thesis_wsp/src/amcl_csv_writer.py /robot${robot_id}/amcl_pose ${data_file_path}" "/tmp/data_pid_${robot_id}.txt")
                pgids+=($PGID_DATA)
            done

            sleep 5

            # Launch goals
            echo "Launching goals for scenario $scenario..."
            PGID_GOALS=$(launch_and_capture_pgid "bash ~/ros2_humble_wss/thesis_wsp/mission_scripts/goals${scenario}.sh; sleep 6; exit" "/tmp/goals_pid.txt")
            pgids+=($PGID_GOALS)

            # Wait before terminating all processes
            echo "Waiting 70 seconds before terminating all processes..."
            sleep 70
            echo "Terminating all processes..."
            for pgid in "${pgids[@]}"; do
                kill -TERM -$pgid 2>/dev/null
            done

            close_all_gnome_terminals

            echo "Scenario $scenario, Navigation $nav_label, Repetition $repetition completed."

            echo -e "**************\nFINISHING TEST\n**************"
        done
    done
done
