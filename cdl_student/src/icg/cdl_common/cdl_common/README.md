# Maintainer
cdl common - CAMERA_DRONES_LECTURE Project (2020-2021)<br/>
Authors/Maintainers: Jesús Pestana <jesus.pestana@pro2future.at><br/>
Created on: October 2020

# Description - cdl_common meta-package purpose:

This meta-package holds the following: definition of common messages, startup scripts, rosinstall files and other "text_metadata" information (documentation and so on), as well as software packages necessary for this project (part of them directly developed for this project).

* icg_msgs package : 
  This package holds the definition of common ROS messages in this project.
    
    + FullOdometry / FullOdometryWOCov msgs: this message describes: timestamp, rigid-body frame name, and the position, speed and acceleration (both linear and angular) of a drone (or a rigid object). This are important magnitudes necessary for the navigation control algorithms of a drone, the most important among them are position and speed (both linear and angular), the linear acceleration is also relevant for rejecting wind disturbances.
    
    + IR Marker (Motion Tracking System, MTS, related) msgs : these messages were defined in order to receive information from the MTS about groups of IR markers, so that calibrating the positions of IR markers around the drone's body is possible by the package "icg_marker_constellation_calibrator". In particular I am referring to the following message definitions: MarkerSet.msg, MarkerSetStamped.msg, MarkerSetsStamped.msg, PointVector.msg, PointVectorStamped.msg and PointVectorsStamped.msg. 

* icg_marker_constellation_calibrator package : 
  When flying a drone inside the Motion Tracking System's (MTS) tracking-volume, it is important to be able to define precisely where the center of its reference frame is located. For instance, it is important that the frame is horizontally aligned and pointing towards the front of the drone. This package contains software to use a calibration stand to support us on this task: calibrating/estimating the positions of IR markers around the drone's body is possible by the package "icg_marker_constellation_calibrator".
    
    + icg_single_constellation_calibrator: in order to fix the reference frame used for a tracked object ("asset" in the Optitrack's Motive software) the user needs to define the positions of the object's markers with high precision (ideally with the same or better accuracy of the MTS system itself, in our case sub-mm accuracy) with respect to the desired object reference frame.
    For this purpose, this executable takes as input (markers_calibration_file) the hand-measured positions of the markers with respect to the desired reference frame and (markers_measurement_file) a set of estimated positions of the same group of markers inside the MTS tracking volume. The software matches each marker from both sets (calibration vs measurement / position estimates) to each other; once the matching is done their relative position is registered using a rigid-body transformation (translation + rotation) and the refined positions of the markers with respect to the desired reference frame are provided as output. These set of refined positions can be then used by the MTS system (Optitrack's Motive software), so that the desired reference frame is used from the MTS sofwate itself (Optitrack's Motive software).
    Note: this software is actually necessary, to calibrate the MTS system (Optitrack's Motive software) for the utilization of the calibration stand itself (which is also equipped with IR markers) inside the tracking volume. The calibration stand is used as a known object inside the tracking volume, to fix a second object to be tracked to the calibration stand and measure the position of its markers with respect to the calibration stand. This makes this calibration procedure more versatile (the positions of the markers on the second object do not need to be hand measured) and easier to be repated for similar objects (for instance, for more iterations of the same drone construction with different/unknown marker constellations). 
    Example usage:
      roslaunch icg_marker_constellation_calibrator icg_single_constellation_calibrator.launch markers_calibration_file:=$(rospack find icg_marker_constellation_calibrator)/cfg/calib_stand/markers_calibration.yaml markers_measurement_file:=$(rospack find icg_marker_constellation_calibrator)/cfg/calib_stand/markers_measurement.yaml
    
    + icg_with_calibration_stand_calibrator: once the calibration stand is calibrated properly (using the icg_single_constellation_calibrator software) and set up in the MTS system (Optitrack's Motive software), the calibration stand is used as a known reference frame. The config_file provides the desired rigid-body transformation (position and orientation) of the second object's frame (for instance, of a drone) with respect to the known frame of the calibration stand. 
    The calibration stand subscribes to the "all marker sets" topics of the MTS interface node (mocap_node). It provides as output the position of the second object's (drone's) markers in its desired reference frame. These set of markers' positions can be then used by the MTS system (Optitrack's Motive software), so that desired reference frame is used.
    Example usage:
      roscore
      rosbag play $(rospack find icg_marker_constellation_calibrator)/test_data/calib_stand_test.bag 
      roslaunch cdl_common mocap.launch
      roslaunch icg_marker_constellation_calibrator icg_with_calibration_stand_calibrator.launch
      Note: in addition to this, the calibration stand tracked object (asset in the Optitrack's Motive software) is to be configured properly, and it's asset ID provided to the configuration of the mocap_node (so that its pose is republished in the ROS network). Also, the markers of the object to be tracked need to be part of an object in the Motive software, so that they are streamed as a single marker set, defined by the asset's name (this information is part of the config_file of the icg_with_calibration_stand_calibrator as well).
      Note-2: when the parameter use_marker_positions_file is to true (and the marker_positions_file defined) in the launchfile, then the behavior of this software differs in that the LED constellation loaded from the file is respected. This means that the relative position between LEDs is kept the same, and the output is the rotation of this constellation considering the pose of the calibration stand and the reference frame desired position defined by the config_file.

* icg_pose_derivator package : The main purpose of the Pose Derivator is to obtain the velocity and the acceleration out 
 of pose measurement data streams, such as the one obtained when using a Motion Tracking System (MTS).
 For this purpose we are using numeric deritatives, the following methods have been tested:
   + Savitzky-Golay Filter (SGF), more documentation in file: test/main_test_savitzky_golay_filter_table.cpp
   + Central-Finite-Differences (CFD), more documentation in file: test/main_test_num_deriv_finit_diffs.cpp
 The node is implemented to use either SGF or {CFD, then SGF} for the numerical derivation. However, the use of  
 only SGF is recommended. This is because CFD amplifies the effect of noise in the derivatives - which renders 
 the method useless, unless perfect clean data is utilized. Both of these algorithms are implemented as 
 convolution filters.

# Included startup script files:
On the usage of scripts and procedures related to the use of this software bundle (safe_track_2020 ROS workspace) and (kalibr_ws ROS workspace). The following is a list of the available startup scripts for this project, they allow to start ROS architectures with a single terminal command. The startup scripts were written: (1) to make the startup of ROS architectures faster, (2) for the later capability of setting them to be started automatically after boot-up and (3) to avoid errors when they are started by hand, node by node. 
Note: Sometimes starting a whole ROS achitecture with a single launch-file can also lead to startup errors.

* [calibration routines]:

  + hand-eye-calibration:
    - start_hand_eye_calibration.sh : the hand-eye-calibration requires the simultaneous acquisition of T_target_camera (pose of the camera with respect to a fiducial marker / target) and T_room_camera-object (pose of the MTS tracked asset/object to which the camera is fixed, with respect to the MTS room / tracking volume). In this setting the marker is not to be moved, meanwhile the camera is moved looking at the target from different angles. The output of this process is the pose T_camera-object_camera (pose of the camera projection center with respect to the of the MTS tracked asset/object to which the camera is fixed). This pose allows to know the position and orientation of the camera projection center inside the MTS tracking volume. The hand_eye_calibration in performed in 3 steps (see startup script):
      (1) handeye_tf.launch : acquisition of a calibration dataset, set of poses: T_room_camera-object and T_target_camera.
        (1.a) acquisition of a test dataset
        (1.b) a dataset file can be opened, and new data can be appended at its end
      (2) handeye_file.launch : processing the calibration data. Note: the result of the calibration can be compared against measureing the resulting pose against hand-made measurements, T_camera-object_camera . 
      (3) handeye_visualizer.launch : evaluating the result (this can be done on a different dataset by the way). The visualizer outputs the spread of the estimation of the marker position inside the room for the utilized test set. This spread (measured as an RMSE) allows to quantitatively evaluate the quality of the obtained hand-eye-calibration.
    Note: we are using and AprilGrid fiducial marker and the kalibr validator node to acquire the poses T_target_camera.
    - start_hand_eye_calibration_simulated.sh : the hand-eye-calibration software that we are using works quite well, however the code had some badly chosen variable names and severe code copy-pasting (rather than proper function encapsulation). For this reason: (1) the code was cleaned up and (2) a test routine was created (namely this startup script).
    The test routine consists on: 
      (1) handeye_simdata_gen.launch : generating test data, 
      (2) handeye_file.launch : processing the data 
      (3) handeye_visualizer.launch : evaluating the result (this can be done on a different dataset by the way). The visualizer outputs the spread of the estimation of the marker position inside the room for the utilized test set. This spread (measured as an RMSE) allows to quantitatively evaluate the quality of the obtained hand-eye-calibration.

* [icg_pose_derivator]
  + test_icg_pose_derivator.sh / test_icg_pose_derivator_msf.sh (inside icg_pose_derivator package) : script to playback a rosbag with pose data acquired using the motion tracking system of an object equipped with an IMU (and an onboard computer acquiring data), such as a drone. The script starts an EKF (the MSF) and several icg_pose_derivator nodes; and records all output full_odometry streams. The script also leaves several byobu tabs open with commands to convert the data in the rosbag to csv files, and a command to start a python-based plotting script that reads these csv files.

* [other sensors]:
  [DEPRECATED] + start_ros_camera_system.sh / stop_ros_camera_system.sh : start and stop of the acquisition of stereo image datasets using the ZED camera ( https://www.stereolabs.com/zed/ ). This will be useful, if the ZED camera is implemented as stereo sensor to acquire point-clouds in some part of our processing architecture (for instance, relative localization of the camera system against the hallway).

# Other instructions (not included as startup scripts):
[EMPTY]



