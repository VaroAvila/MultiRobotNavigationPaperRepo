import os

robots = ["robot0", "robot1", "robot2"]
points = {
    "A": (-2.65, -1, 0),
    "B": (-2.85, 0.6, 0),
    "C": (3.7, 0.3, 0),
    "D": (3.6, 1.6, 0),
    "E": (3.5, 3.2, 0),
}
orientations = {
    "N": (0.0, 0.0, -1, 0.115),
    "S": (0.0, 0.0, -0.115, -1),
}

template = """#!/bin/bash
ros2 topic pub -1 /{robot}/goal_pose geometry_msgs/PoseStamped "{{
  'header': {{
    'frame_id': 'map',
    'stamp': {{ 'sec': $(date +%s), 'nanosec': 0 }}
  }},
  'pose': {{
    'position': {{ 'x': {x}, 'y': {y}, 'z': {z} }},
    'orientation': {{ 'x': {ox}, 'y': {oy}, 'z': {oz}, 'w': {ow} }}
  }}
}}"
"""

for robot in robots:
    for point, (x, y, z) in points.items():
        for orientation, (ox, oy, oz, ow) in orientations.items():
            script_content = template.format(robot=robot, x=x, y=y, z=z, ox=ox, oy=oy, oz=oz, ow=ow)
            script_name = f"{robot}_{point}{orientation}.sh"
            with open(script_name, 'w') as script_file:
                script_file.write(script_content)
            os.chmod(script_name, 0o755)
