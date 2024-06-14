#!/bin/bash
ros2 topic pub -1 /robot1/goal_pose geometry_msgs/PoseStamped "{
  'header': {
    'frame_id': 'map',
    'stamp': { 'sec': $(date +%s), 'nanosec': 0 }
  },
  'pose': {
    'position': { 'x': 3.7, 'y': 0.3, 'z': 0 },
    'orientation': { 'x': 0.0, 'y': 0.0, 'z': -1, 'w': 0.115 }
  }
}"
