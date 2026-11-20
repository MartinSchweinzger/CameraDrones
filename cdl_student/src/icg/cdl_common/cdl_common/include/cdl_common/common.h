#ifndef __CDL_MSGS_COMMON_H__
#define __CDL_MSGS_COMMON_H__

#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <icg_msgs/NaviControlMode.h>

namespace CdlMsgsCommon
{
    enum STATES
    {
        MAV_START_MISSION = 1,
        MAV_WAIT = 2,
        // MAV_ABORT = 49, // UNUSED
        MAV_STARTING_MISSION = 50,
        MAV_IDLE = 51,
        MAV_WAITING = 52,
        MAV_WAITING_DONE = 53,
        MAV_ABORTED = 99,
        // NAVI - navigation_controller
        NAVI_HOVER = 100,
        NAVI_STOP = 101,
        NAVI_POINT_TO_POINT_NAVIGATION = 102,
        NAVI_FLY_TRAJECTORY = 103,
        //NAVI_TAKE_OFF = 104,
        //NAVI_LAND = 105,
        NAVI_ABORT = 106,
        NAVI_IDLE = icg_msgs::NaviControlMode::NAVI_IDLE,
        NAVI_AWAITING_COMMANDS = icg_msgs::NaviControlMode::NAVI_AWAITING_COMMANDS,
        NAVI_HOVERING = icg_msgs::NaviControlMode::NAVI_HOVERING,
        NAVI_FLYING_POINT_TO_POINT = icg_msgs::NaviControlMode::NAVI_FLYING_POINT_TO_POINT,
        NAVI_WAYPOINT_REACHED = icg_msgs::NaviControlMode::NAVI_WAYPOINT_REACHED,
        NAVI_FLYING_TRAJECTORY = icg_msgs::NaviControlMode::NAVI_FLYING_TRAJECTORY,
        NAVI_TRAJECTORY_FINISHED = icg_msgs::NaviControlMode::NAVI_TRAJECTORY_FINISHED,
        NAVI_ABORTED = icg_msgs::NaviControlMode::NAVI_ABORTED,
        // NAVI_LANDING = 158,
        // NAVI_FLYING_IDLE = 159,
        // NAVI_TAKING_OFF = 160,
        // trajectory planning
        PLANNER_SET_IDLE = 200,
        PLANNER_PLAN_TRAJECTORY = 201,
        PLANNER_SMOOTH_TRAJECTORY = 202,
        PLANNER_IDLE = 250,
        PLANNER_PLANNING_TRAJECTORY = 251,
        PLANNER_TRAJECTORY_PLANNED = 252,
        PLANNER_SMOOTHING_TRAJECTORY = 253,
        PLANNER_TRAJECTORY_SMOOTHED = 254,
        // task_scheduler_2
        TASK_SCHEDULER_2_SET_IDLE = 300,
        TASK_SCHEDULER_2_START = 301,
        TASK_SCHEDULER_2_IDLE = 350,
        TASK_SCHEDULER_2_STARTED = 351,
        TASK_SCHEDULER_2_STOPPED = 352,
        // icg_rosbag_recorder
        ROSBAG_RECORDER_START   = 1101,
        ROSBAG_RECORDER_STOP    = 1102,
        ROSBAG_RECORDER_RESTART = 1103,
        ROSBAG_RECORDER_LOAD_CONFIG_FILE = 1104,
        ROSBAG_RECORDER_UNKNOWN      = 1150,
        ROSBAG_RECORDER_IDLE         = 1151,
        ROSBAG_RECORDER_RUNNING      = 1152,
        ROSBAG_RECORDER_START_FAILED = 1153,
        ROSBAG_RECORDER_STOP_FAILED  = 1154,
        ROSBAG_RECORDER_ABORTED      = 1155,
        ROSBAG_RECORDER_CONFIG_FILE_LOADED = 1156,
        // fly_sm
        //   (fly_sm) commands:
        FLY_SM_TAKE_OFF = 1001,
        FLY_SM_LAND = 1002,
        FLY_SM_HOVER = 1003,
        FLY_SM_FLY_FORWARD = 1004,
        FLY_SM_FLY_BACKWARD = 1005,
        FLY_SM_FLY_UP = 1006,
        FLY_SM_FLY_DOWN = 1007,
        FLY_SM_FLY_LEFT = 1008,
        FLY_SM_FLY_RIGHT = 1009,
        FLY_SM_ROTATE_CW = 1010,
        FLY_SM_ROTATE_CCW = 1011,
        FLY_SM_RELEASE_CONTROL = 1012,
        FLY_SM_INITIALIZE = 1013,
        FLY_SM_SET_FAST_MODE = 1014,
        FLY_SM_SET_SLOW_MODE = 1015,
        // FLY_SM_FLY_ABORT = 1020,
        //   (fly_sm) states:
        FLY_SM_INITIALIZING = 1051,
        FLY_SM_INITIALIZED  = 1052,
        FLY_SM_LANDED         = 1053,
        FLY_SM_TAKING_OFF     = 1054,
        FLY_SM_HOVERING = 1055,
        FLY_SM_LANDING = 1056,
        FLY_SM_FLYING = 1057,
        FLY_SM_CONTROL_RELEASED = 1058,
        FLY_SM_FAST_MODE_SET = 1059,
        FLY_SM_SLOW_MODE_SET = 1060,
        FLY_SM_ABORTED = 1099,
        // face_detector
        FACE_DETECTOR_START = 1201,
        FACE_DETECTOR_STOP = 1202,
        FACE_DETECTOR_UNKNOWN = 1250,
        FACE_DETECTOR_IDLE = 1251,
        FACE_DETECTOR_RUNNING = 1252,
        FACE_DETECTOR_START_FAILED = 1253,
        FACE_DETECTOR_STOP_FAILED = 1254,
        FACE_DETECTOR_ABORTED = 1255,
        // icg_image_saver
        IMAGESAVER_SAVE_IMAGE  = 1701,
        IMAGESAVER_SAVED_IMAGE = 1751,
        // apriltag_detector
        APRILTAG_SET_IDLE = 600,
        APRILTAG_SEARCH_MARKERS = 604,
        APRILTAG_IDLE = 650,
        APRILTAG_SEARCHING_MARKERS = 658
    };

    inline std_msgs::Int32 createMsg(int val)
    {
        std_msgs::Int32 msg;
        msg.data = val;
        return msg;
    }
}

// namespace
#endif // __CDL_MSGS_COMMON_H__
