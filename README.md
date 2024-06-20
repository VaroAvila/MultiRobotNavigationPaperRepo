This repository contains a set of tools to automate the simulation, testing, and data collection process for the [MultiRobotNavigationRepo](https://github.com/VaroAvila/MultiRobotNavigationThesisRepo) repository.

It has only been tested on an Ubuntu 22.04.4 LTSJammy System with Ros2 Humble. 
<br>
<br>

# Use

<br>

To use it follow the steps on the readme file from the [MultiRobotNavigationRepo](https://github.com/VaroAvila/MultiRobotNavigationThesisRepo) repository and then clone this repository in the ~/ros2_humble_wss/thesis_wsp folder:

```
cd ~/ros2_humble_wss/thesis_wsp

git clone https://github.com/VaroAvila/MultiRobotNavigationPaperRepo
```

Run the automation script with:

```
bash automation_script.sh
```

The simulation pipeline will start with the predefined scenarios. In this case, it uses a set of 6 scenarios and specific navigation files.

The .csv files will the trajectory data will be saved in the 

To test the [path prediction layer](https://github.com/VaroAvila/path_prediction_costmap_layer) and the [footprint layer](https://github.com/VaroAvila/footprint_costmap_layer) check their respective readme files to adapt the parameters according to the scenarios. (e.g. change shared path and speeds, or footprint padding)
<br>
<br>
## Additional tools:
<br>

1. graph_generation.py: generates trajectory graphs for each of the simulation scenarios

2. verification_script.py: generates a .txt file and a .csv file evaluating the results from the .csv trajectory data files. 
<br>

> [!NOTE]
> The graph generation tool is currently in development and might not work.
> 
> The verification script needs to be reviewed.
<br>

> [!WARNING]
> The .csv file generation might generate inconsistent names in a couple of scenarios. this will be fixed in the next updates.
> 
> As a temporary solution a renaming bash script is used

> [!WARNING]
> The multiple nodes running can cause errors in the simulation, like some services not loading when needed, causing some scenarios to fail. This error is related to the Neobotix simulation packages
> 
> The timings set on the automation_script.sh try to reduce this issues as much as possible 
