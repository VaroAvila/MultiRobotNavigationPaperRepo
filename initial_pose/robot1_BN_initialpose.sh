#!/bin/bash
ros2 topic pub -1 /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped "{
  'header': {
    'frame_id': 'map',
    'stamp': { 'sec': $(date +%s), 'nanosec': 0 }
  },
  'pose': {
    'pose': {
      'position': { 'x': -2.85, 'y': 0.6, 'z': 0 },
      'orientation': { 'x': 0.0, 'y': 0.0, 'z': -1, 'w': 0.115 }
    },
    'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}"
