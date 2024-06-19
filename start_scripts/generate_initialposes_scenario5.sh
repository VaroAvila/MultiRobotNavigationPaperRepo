#!/bin/bash

# Function to publish initialpose
publish_initialpose() {
    robot=$1
    x=$2
    y=$3
    yaw=$4

    z=$(echo "s($yaw / 2)" | bc -l)
    w=$(echo "c($yaw / 2)" | bc -l)

    ros2 topic pub -1 /$robot/initialpose geometry_msgs/PoseWithCovarianceStamped "{
      'header': {
        'frame_id': 'map',
        'stamp': { 'sec': $(date +%s), 'nanosec': 0 }
      },
      'pose': {
        'pose': {
          'position': { 'x': $x, 'y': $y, 'z': 0.0 },
          'orientation': { 'x': 0.0, 'y': 0.0, 'z': $z, 'w': $w }
        },
        'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      }
    }"
}

publish_initialpose "robot0" 3.7 0.3 3.14
publish_initialpose "robot1" 3.6 1.6 3.14
publish_initialpose "robot2" 3.5 3.2 3.14
