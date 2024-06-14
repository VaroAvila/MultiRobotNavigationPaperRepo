#!/bin/bash

# Arrays para guardar los comandos de lanzamiento de la navegación
navigation_files=("multi_robot_navigation_navfn_nofp.launch.py" "multi_robot_navigation_navfn_fp.launch.py")

# Bucle principal para cada uno de los 6 escenarios
for scenario in {1..6}; do
    for nav_type in {0..1}; do  # 0 para nofp, 1 para fp
        nav_file=${navigation_files[$nav_type]}
        nav_label=$(echo $nav_file | cut -d'_' -f5)  # extrae 'nofp' o 'fp'

        # Repetir cada tipo de navegación 5 veces
        for repetition in {1..5}; do
            echo "Starting Scenario $scenario, Navigation $nav_label, Repetition $repetition..."

            # Arrays para PIDs y PGIDs
            pids=()
            pgids=()

            # Función para lanzar procesos
            launch_and_capture_pids() {
                local command=$1
                local pid_file=$2
                gnome-terminal -- bash -c "$command & echo \$! > $pid_file; wait" &
                sleep 2  # Esperar a que se inicie el terminal y el proceso

                if [ -f $pid_file ]; then
                    local pid=$(cat $pid_file)
                    echo $pid
                else
                    echo "Failed to retrieve the PID for command: $command"
                    exit 1
                fi
            }

            # Lanzar el proceso de navegación
            ROS2_PID=$(launch_and_capture_pids "ros2 launch neo_simulation2 $nav_file" "/tmp/ros2_pid.txt")
            pids+=($ROS2_PID)
            PGID=$(ps -o pgid= -p $ROS2_PID | tr -d ' ')
            pgids+=($PGID)

            sleep 4 # Tiempo para asegurar la inicialización del proceso de navegación

            # Lanzar el escenario de simulación correspondiente
            SIM_PID=$(launch_and_capture_pids "ros2 launch neo_simulation2 exp${scenario}.launch.py; sleep 3; exit" "/tmp/sim_pid.txt")
            pids+=($SIM_PID)
            SIM_PGID=$(ps -o pgid= -p $SIM_PID | tr -d ' ')
            pgids+=($SIM_PGID)

            sleep 3 # Pequeño retraso antes de lanzar los goals

            # Lanzar Rviz después de la simulación
            RVIZ_PID=$(launch_and_capture_pids "ros2 launch neo_nav2_bringup rviz_launch.py rviz_config:=install/neo_nav2_bringup/share/neo_nav2_bringup/rviz/multi_robot.rviz" "/tmp/rviz_pid.txt")
            pids+=($RVIZ_PID)
            RVIZ_PGID=$(ps -o pgid= -p $RVIZ_PID | tr -d ' ')
            pgids+=($RVIZ_PGID)

            sleep 2

            # Recolectar datos para cada robot antes de lanzar los goals
            for robot_id in {0..2}; do
                data_filename="exp_data_${nav_label}_experiment${scenario}_repetition${repetition}_robot${robot_id}.csv"
                data_file_path="~/ros2_humble_wss/thesis_wsp/exp_data/${data_filename}"
                DATA_PID=$(launch_and_capture_pids "python3 ~/ros2_humble_wss/thesis_wsp/src/amcl_csv_writer.py /robot${robot_id}/amcl_pose ${data_file_path}" "/tmp/data_pid_${robot_id}.txt")
                pids+=($DATA_PID)
                DATA_PGID=$(ps -o pgid= -p $DATA_PID | tr -d ' ')
                pgids+=($DATA_PGID)
            done

            sleep 5

            # Lanzar objetivos
            GOALS_PID=$(launch_and_capture_pids "bash ~/ros2_humble_wss/thesis_wsp/mission_scripts/goals${scenario}.sh; sleep 6; exit" "/tmp/goals_pid.txt")
            pids+=($GOALS_PID)
            GOALS_PGID=$(ps -o pgid= -p $GOALS_PID | tr -d ' ')
            pgids+=($GOALS_PGID)

            # Cerrar Gazebo específicamente si está corriendo
            sleep 70

            pkill -9 -x gzclient || echo "Gazebo could not be killed"

            # Temporizador para matar todos los procesos después de 70 segundos
            ( sleep 2; for pgid in "${pgids[@]}"; do kill -TERM -$pgid; done ) &

            # Esperar a que todos los procesos terminen
            wait

            sleep 10

            echo "Completed Scenario $scenario, Navigation $nav_label, Repetition $repetition"
        done
    done
done
