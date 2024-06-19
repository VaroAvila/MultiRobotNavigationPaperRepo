import pandas as pd
import os
import glob
import re
from math import sqrt

# Define the goal positions and orientations
goals = {
    "ES": ((3.5, 3.2, 0), (0.0, 0.0, -0.115, -1)),
    "BN": ((-2.85, 0.6, 0), (0.0, 0.0, -1, 0.115)),
    "AN": ((-2.65, -1, 0), (0.0, 0.0, -1, 0.115)),
    "CS": ((3.7, 0.3, 0), (0.0, 0.0, -0.115, -1)),
    "DS": ((3.6, 1.6, 0), (0.0, 0.0, -0.115, -1))
}

# Define tolerances for position
position_tolerance = 0.2

# Base directory for the experiment data
base_dir = os.path.expanduser("~/ros2_humble_wss/thesis_wsp/exp_data")

# Define the goal scripts for each robot in each scenario
goal_scripts = {
    "experiment1": {"robot0": "ES", "robot1": "BN", "robot2": "AN"},
    "experiment2": {"robot0": "ES", "robot1": "CS", "robot2": "BN"},
    "experiment3": {"robot0": "BN", "robot1": "DS", "robot2": "AN"},
    "experiment4": {"robot0": "AN", "robot1": "ES", "robot2": "BN"},
    "experiment5": {"robot0": "ES", "robot1": "BN", "robot2": "AN"},
    "experiment6": {"robot0": "AN", "robot1": "DS", "robot2": "BN"}
}

def goal_reached(goal_pos, position, pos_tol):
    pos_distance = sqrt((goal_pos[0] - position[0])**2 + (goal_pos[1] - position[1])**2)
    return pos_distance <= pos_tol

def process_robot_csv(file_path, goal):
    if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
        return "LOAD ERROR", None, None

    df = pd.read_csv(file_path, delimiter='=', decimal=',', engine='python')
    if df.empty:
        return "LOAD ERROR", None, None

    start_time = None
    end_time = None
    started_moving = False

    for index, row in df.iterrows():
        position = (row['position_x'], row['position_y'], row['position_z'])
        if not started_moving and (abs(row['position_x']) > 0 or abs(row['position_y']) > 0):
            started_moving = True
            start_time = row['timestamp']
        if started_moving and goal_reached(goal[0], position, position_tolerance):
            end_time = row['timestamp']
            break

    return None, start_time, end_time

def main():
    scenarios = range(1, 7)
    configurations = ['fp', 'nofp']
    robots = ['robot0', 'robot1', 'robot2']
    summary = []

    for config in configurations:
        for scenario in scenarios:
            file_pattern = f"exp_data_{config}.launch.py_experiment{scenario}_repetition*_robot*.csv"
            matches = glob.glob(os.path.join(base_dir, file_pattern))
            repetitions = sorted(set(re.search(r'repetition(\d+)', match).group(1) for match in matches))
            
            for repetition in repetitions:
                start_times = []
                end_times = []
                status = None

                for robot in robots:
                    file_path = os.path.join(base_dir, f"exp_data_{config}.launch.py_experiment{scenario}_repetition{repetition}_{robot}.csv")
                    goal_key = goal_scripts[f"experiment{scenario}"][robot]
                    goal = goals[goal_key]
                    result, start_time, end_time = process_robot_csv(file_path, goal)
                    if result == "LOAD ERROR":
                        status = "LOAD ERROR"
                        break
                    elif end_time is None:
                        status = "failure"
                    else:
                        if start_time:
                            start_times.append(start_time)
                        if end_time:
                            end_times.append(end_time)
                
                if status is None and len(end_times) == 3:
                    scenario_time = max(end_times) - min(start_times) if start_times and end_times else None
                    summary.append((config, scenario, repetition, scenario_time, "success"))
                else:
                    summary.append((config, scenario, repetition, None, status if status else "failure"))

    summary_df = pd.DataFrame(summary, columns=["Configuration", "Mission", "Repetition", "TimeTaken", "Status"])
    summary_df.to_csv(os.path.join(base_dir, "mission_summary.csv"), index=False)
    summary_df.to_csv(os.path.join(base_dir, "mission_summary.txt"), sep='\t', index=False)

    print("Summary saved to mission_summary.csv and mission_summary.txt")

if __name__ == "__main__":
    main()
