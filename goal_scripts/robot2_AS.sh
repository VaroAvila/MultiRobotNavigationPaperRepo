#!/bin/bash
ros2 topic pub -1 /robot2/goal_pose geometry_msgs/PoseStamped "{
  'header': {
    'frame_id': 'map',
    'stamp': { 'sec': $(date +%s), 'nanosec': 0 }
  },
  'pose': {
    'position': { 'x': -2.65, 'y': -1, 'z': 0 },
    'orientation': { 'x': 0.0, 'y': 0.0, 'z': -0.115, 'w': -1 }
  }
}"
