#!/bin/bash
ros2 topic pub -1 /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped "{
  'header': {
    'frame_id': 'map',
    'stamp': { 'sec': $(date +%s), 'nanosec': 0 }
  },
  'pose': {
    'pose': {
      'position': { 'x': -2.65, 'y': -1, 'z': 0 },
      'orientation': { 'x': 0.0, 'y': 0.0, 'z': -0.115, 'w': -1 }
    },
    'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}"
