/*
 * icg_apriltag_marker_detector_ros.h
 *
 * Created on: Mai 16, 2019
 * Author: Michael Maurer
 * Modified: Jesus Pestana Puerta (November 2019)
 * Modified: Jun Zhang (October 2024)
 *
 */

#ifndef ICGAPRILTAGMARKERDETECTORROS_H_
#define ICGAPRILTAGMARKERDETECTORROS_H_

#include <Eigen/Eigen>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/PoseStamped.h>
#include <cdl_common/common.h>
#include <icg_msgs/AprilTagDetections.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// apriltags
#include <TagDetector.h>
#include <CameraUtil.h>

// config file
#include <ros/package.h>    // ros::package::getPath(...) function
#include <yaml-cpp/yaml.h>

class IcgApriltagMarkerDetectorRos
{
public:
    IcgApriltagMarkerDetectorRos(ros::NodeHandle n, ros::NodeHandle pn);
    ~IcgApriltagMarkerDetectorRos();

    bool initWithFile(std::string config_file_ = std::string("config_default.yaml") );
    void startDetectingMarkers();
    void stopDetectingMarkers();

private:
    bool debug_im;
    bool detecting_markers;
    image_transport::CameraPublisher camera_pub;

    ros::NodeHandle pnode_;
    ros::NodeHandle node_;

    std::string config_file;
    std::string this_package_path;

    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber image_sub_;
    ros::Subscriber sub_in_;
    ros::Publisher pub_out_;
    ros::Publisher pub_detected_markers_;
    // ROS tf broadcaster
    tf2_ros::TransformBroadcaster tfbc;

    // Callbacks
    void onNewImage(const sensor_msgs::Image::ConstPtr& p_img, const sensor_msgs::CameraInfo::ConstPtr& p_info);
    void onIn(const std_msgs::Int32::ConstPtr& p_msg);

    //Functions

    //Variables Apriltag
    std::string tag_family;
    std::map<int,float> possible_markers_;
    TagFamily apriltag_family_;
    TagDetectorParams apriltag_params_;
    TagDetector* apriltag_detector_;
};

#endif // ICGAPRILTAGMARKERDETECTORROS_H_