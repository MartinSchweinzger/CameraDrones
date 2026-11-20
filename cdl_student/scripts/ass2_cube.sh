#!/bin/bash

# Set parameters
SESSION_NAME='ass2'
TMUX='byobu' #Options: tmux | byobu
SUPERUSER='aerial' #'student'
CATKIN_WS=${CMAKE_PREFIX_PATH%%:*}/.. #'/media/veracrypt1/workspaces/ros/cdl_2024_student/' #${CMAKE_PREFIX_PATH%%:*}/..
SOURCE=${CATKIN_WS}/devel/setup.bash #'/home/'${SUPERUSER}'/.bashrc'  #'/home/.bashrc' #'/home/'${SUPERUSER}'/.bashrc'

OCTOMAP='cube_synthetic.bt'
OCTOMAP_DISPLAY='cube_synthetic_display.bt'

X_S='1.5'  #start position
Y_S='-1.5'
Z_S='0.75'
X_G='-0.5' #goal position '-1.5' '-0.5'
Y_G='1.5'  #              '-1.5' '1.5'
Z_G='0.75' #              '0.75' '0.75'
  
BAGFILE_FOLDER="/home/student/cdl_2024_student/bags"
BAGFILE="cube_trajectory_wp0_to_wp1"

# General
#'cd /home/'${SUPERUSER}

# Check for session
${TMUX} has-session -t ${SESSION_NAME}

if [ $? != 0 ]
then
  # Create the session
  ${TMUX} new-session -s ${SESSION_NAME} -n core -d

  # Start roscore (0)
  ${TMUX} send-keys -t ${SESSION_NAME}:0 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:0 'roscore' C-m
  sleep 5s

  # visualization (1)
  #   publish octomap from file
  ${TMUX} new-window -n visualization -t ${SESSION_NAME}
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'roslaunch $(rospack find cdl_common)/launch/cdl_assignments/octomap_mapping_a2.launch octomap_file:='"${OCTOMAP_DISPLAY}"' --wait' C-m
  #   trajectory_visualization 1 - smoothed path planned
  ${TMUX} split-window -v -t ${SESSION_NAME}:1
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'rosrun dla2_path_planner dla2_path_planner_trajectory_visualization' C-m
  #   trajectory_visualization 2 - path raw
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.0
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'roslaunch dla2_path_planner trajectory_visualization.launch marker_color:="O"' C-m
  #   3D Visulization - rviz
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.2
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'rosrun rviz rviz -d $(rospack find cdl_common)/scripts/cdl_assignments/rviz/img_pp3.rviz' C-m

  # path_planning + trajectory generation (2)
  #   dla2_path_planner_ros_node - raw + smoothed path
  ${TMUX} new-window -n path_planning -t ${SESSION_NAME}
  ${TMUX} send-keys -t ${SESSION_NAME}:2 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:2 'rosrun dla2_path_planner dla2_path_planner_ros_node --runtime 5.0 --planner RRTStar -o WeightedLengthAndClearanceCombo -f planner_trajectory.txt --info 2 --bt_file '${OCTOMAP} C-m
  #   dla3_trajectory_generator node: smoothed path - (high order polynomial spline) generated path
  ${TMUX} split-window -v -t ${SESSION_NAME}:2
  ${TMUX} send-keys -t ${SESSION_NAME}:2.1 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:2.1 'roslaunch mav_trajectory_generation_example dla3_trajectory_generator.launch' C-m

  # ros_topic_echo (3)
  #   topic echo: /path_planner/planned_trajectory
  ${TMUX} new-window -n ros_topic_echo -t ${SESSION_NAME}
  ${TMUX} send-keys -t ${SESSION_NAME}:3 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:3 'rostopic echo /path_planner/planned_trajectory' C-m
  #   topic echo: /path_planner/planned_trajectory_raw
  ${TMUX} split-window -v -t ${SESSION_NAME}:3
  ${TMUX} send-keys -t ${SESSION_NAME}:3.1 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:3.1 'rostopic echo /path_planner/planned_trajectory_raw' C-m
  # topic echo: /trajectory_generator/smooth_trajectory 
  ${TMUX} split-window -v -t ${SESSION_NAME}:3.1
  ${TMUX} send-keys -t ${SESSION_NAME}:3.2 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:3.2 'rostopic echo /trajectory_generator/smooth_trajectory' C-m
  sleep 5s

  # rostopic_pub (4)
  #   topic: /path_planner/current_position geometry_msgs/Point
  ${TMUX} new-window -n rostopic_pub -t ${SESSION_NAME}
  ${TMUX} send-keys -t ${SESSION_NAME}:4 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:4 'rostopic pub /path_planner/current_position mav_planning_msgs/Point3D "{x: '${X_S}', y: '${Y_S}', z: '${Z_S}'}" --once' C-m
  #   topic: /path_planner/goal_position geometry_msgs/Point
  ${TMUX} split-window -v -t ${SESSION_NAME}:4
  ${TMUX} send-keys -t ${SESSION_NAME}:4.1 'source '${SOURCE} C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:4.1 'rostopic pub /path_planner/goal_position mav_planning_msgs/Point3D "{x: '${X_G}', y: '${Y_G}', z: '${Z_G}'}" --once' C-m

  # BEGIN: traj_sampler (5)
  "${TMUX}" new-window -n traj_sampler -t "${SESSION_NAME}"
  "${TMUX}" send-keys -t "${SESSION_NAME}":5.0 'source '"${SOURCE}" C-m
  "${TMUX}" send-keys -t "${SESSION_NAME}":5.0 'roslaunch mav_trajectory_generation_example dla3_trajectory_sampler.launch --wait' C-m
  #   topic echo: /trajectory_sampler/command/trajectory
  "${TMUX}" split-window -v -t "${SESSION_NAME}":5.0
  "${TMUX}" send-keys -t "${SESSION_NAME}":5.1 'source '"${SOURCE}" C-m
  "${TMUX}" send-keys -t "${SESSION_NAME}":5.1 'rostopic echo /trajectory_sampler/command/trajectory' C-m
  #   topic echo: /trajectory_sampler/full_odometry
  "${TMUX}" split-window -h -t "${SESSION_NAME}":5.1
  "${TMUX}" send-keys -t "${SESSION_NAME}":5.2 'source '"${SOURCE}" C-m
  "${TMUX}" send-keys -t "${SESSION_NAME}":5.2 'rostopic echo /trajectory_sampler/full_odometry' C-m

  # rosbag_record (6)
  "${TMUX}" new-window -n rosbag_record -t "${SESSION_NAME}"
  "${TMUX}" send-keys -t "${SESSION_NAME}":6.0 'source '"${SOURCE}" C-m
  "${TMUX}" send-keys -t "${SESSION_NAME}":6.0 'cd '"${BAGFILE_FOLDER}" C-m
  "${TMUX}" send-keys -t "${SESSION_NAME}":6.0 'rosbag record -O '"${BAGFILE_FOLDER}"'/'${BAGFILE}'.bag /path_planner/planned_trajectory_raw /path_planner/planned_trajectory /trajectory_generator/smooth_trajectory /trajectory_visualization_raw/trajectory_markers_raw /trajectory_visualization/trajectory_markers /trajectory_generator/smooth_trajectory_markers' C-m
  # /trajectory_visualization/trajectory_markers_array /trajectory_visualization_raw/trajectory_markers_raw_array

  ${TMUX} select-window -t ${SESSION_NAME}:5.0
fi
${TMUX} attach -t ${SESSION_NAME}
#sudo -H byobu attach -t dla3

#type "sudo -H byobu kill-server" to kill console

