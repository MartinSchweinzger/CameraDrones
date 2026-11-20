/*
 * icg_apriltag_marker_detector_ros.cpp
 *
 * Created on: Mai 16, 2019
 * Author: Michael Maurer
 * Modified: Jesus Pestana Puerta (November 2019)
 * Modified: Jun Zhang (October 2024)
 *
 */

#include <icg_apriltag_marker_detector/icg_apriltag_marker_detector_ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <iterator>
#include <Eigen/Core>

#include <opencv2/core/version.hpp>
#if (CV_VERSION_EPOCH == 2)
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/opencv.hpp>
    #include <opencv2/core/core.hpp>
    #include <opencv2/core/eigen.hpp>
    #include <opencv2/core/version.hpp>
#elif (CV_VERSION_MAJOR == 3)
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
    #include <opencv2/calib3d.hpp>
    #include <opencv2/core/eigen.hpp>
#elif (CV_VERSION_MAJOR == 4)
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
    #include <opencv2/calib3d.hpp>
    #include <opencv2/core/eigen.hpp>
#endif

void printMarkerDetection(const TagDetection &tag, int detection_number = 0, std::string extra_info_str=std::string("")) {
        ROS_INFO_STREAM("Detection[" << detection_number << "] - " << extra_info_str);
        ROS_INFO_STREAM("  good: " << tag.good );
        ROS_INFO_STREAM("  obsCode: " << tag.obsCode );
        ROS_INFO_STREAM("  code: " << tag.code );
        ROS_INFO_STREAM("  id: " << tag.id );
        ROS_INFO_STREAM("  hammingDistance: " << tag.hammingDistance );
        ROS_INFO_STREAM("  observedPerimeter: " << tag.observedPerimeter << " [px]" );
        ROS_INFO_STREAM("  rotation: " << tag.rotation );
        ROS_INFO_STREAM("  p[0]: [" << tag.p[0].x << ", " << tag.p[0].y << "]" );
        ROS_INFO_STREAM("  p[1]: [" << tag.p[1].x << ", " << tag.p[1].y << "]" );
        ROS_INFO_STREAM("  p[2]: [" << tag.p[2].x << ", " << tag.p[2].y << "]" );
        ROS_INFO_STREAM("  p[3]: [" << tag.p[3].x << ", " << tag.p[3].y << "]" );
        ROS_INFO_STREAM("  cxy: [" << tag.cxy.x << ", " << tag.cxy.y << "]" );
}

IcgApriltagMarkerDetectorRos::IcgApriltagMarkerDetectorRos(ros::NodeHandle n, ros::NodeHandle pn):
    debug_im(false),
    detecting_markers(false),
    it_(pn),
    pnode_(pn),
    node_(n)
{
    apriltag_detector_ = NULL;
    sub_in_ = pnode_.subscribe( "in", 50, &IcgApriltagMarkerDetectorRos::onIn, this);
    pub_out_ = pnode_.advertise<std_msgs::Int32>("out",10);
    ROS_INFO("TSI: ICG_APRILTAG_MARKER_DETECTOR_IDLE");
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::APRILTAG_IDLE ) );
    // pub_camera_pose_ = pnode_.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
    pub_detected_markers_ = pnode_.advertise<icg_msgs::AprilTagDetections>("detected_markers", 1);

    pnode_.param( std::string("config_file"), config_file, std::string("config_default.yaml") ); 
    this_package_path = ros::package::getPath("icg_apriltag_marker_detector");
	std::string config_file_path = this_package_path + "/config/" + config_file;
	ROS_INFO_STREAM("config_file_path: " << config_file_path);
    initWithFile(config_file_path);
}


IcgApriltagMarkerDetectorRos::~IcgApriltagMarkerDetectorRos()
{
    // delete apriltag_detector_ object
    if (apriltag_detector_ != NULL)
    {
        delete apriltag_detector_;
        apriltag_detector_ = NULL;
    }
}

bool IcgApriltagMarkerDetectorRos::initWithFile(std::string config_file_ ) {
    YAML::Node node_root = YAML::LoadFile(config_file_);
    
    // Apriltag detector init
    YAML::Node tag_family_node = node_root["tag_family"];
    std::string tag_family = tag_family_node.as<std::string>();
    apriltag_family_.init(tag_family.c_str());
	// ** Original Setting **
	apriltag_params_.newQuadAlgorithm = 1;
    apriltag_params_.refineQuads = false;
    apriltag_params_.refineBad = false;
    // -- Original Setting --
	// ** Test Setting 01 **
    // apriltag_params_.newQuadAlgorithm = true;
    // apriltag_params_.refineQuads = true;
    // apriltag_params_.refineBad = true;
    // -- Test Setting 01 --
	// ** Test Setting 02 **
    // apriltag_params_.newQuadAlgorithm = false;
    // apriltag_params_.refineQuads = false;
    // apriltag_params_.refineBad = false;
    // -- Test Setting 02 --
    apriltag_detector_ = new TagDetector(apriltag_family_, apriltag_params_);

    YAML::Node node = node_root["possible_markers"];
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        // uint64_t tag_id = it->first.as<uint64_t>();
        int tag_id = it->first.as<int>();
        double   tag_size = it->second.as<double>();

        //ROS_INFO_STREAM("tag_id: " << tag_id << ", tag_size: " << tag_size);
        possible_markers_[tag_id] = tag_size;
    }

    ROS_INFO("Configuration loaded from file...");
    ROS_INFO_STREAM("tag_family: " << tag_family);
    for (std::map<int,float>::const_iterator it = possible_markers_.begin(); it != possible_markers_.end(); ++it) {
        ROS_INFO_STREAM("possible_markers_[" << it->first << "] => " << it->second);
    }

    return true;
}

void IcgApriltagMarkerDetectorRos::startDetectingMarkers()
{
    if (!detecting_markers) image_sub_ = it_.subscribeCamera( "image_raw", 1, &IcgApriltagMarkerDetectorRos::onNewImage, this);
    ROS_INFO_STREAM("Subscribed to cam_info topic: " << image_sub_.getInfoTopic() );
    ROS_INFO_STREAM("Subscribed to image topic topic: " << image_sub_.getTopic() );
    
    ROS_INFO("Start detecting Apriltag markers!");
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::APRILTAG_SEARCHING_MARKERS ) );
    if (debug_im) {
        if (!detecting_markers) camera_pub = it_.advertiseCamera("marker_image_with_corners",3);
    }
    detecting_markers = true;
}


void IcgApriltagMarkerDetectorRos::stopDetectingMarkers()
{
    if (detecting_markers) {
        image_sub_.shutdown();
        camera_pub.shutdown();
    }
    ROS_INFO("Stopped detecting Apriltag markers!");
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::APRILTAG_IDLE ) );
    detecting_markers = false;
}


void IcgApriltagMarkerDetectorRos::onNewImage(const sensor_msgs::Image::ConstPtr& p_img, const sensor_msgs::CameraInfo::ConstPtr& p_info)
{
    bool debug_output = false;

    // get image
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy( *p_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //image_ = cv_ptr->image;
    //cv::Mat im;
    //cv::resize(cv_ptr->image,im,cv::Size(),2.0,2.0,cv::INTER_CUBIC);
    
    // detect marker
    cv::Point2d opticalCenter, focalLength;
    opticalCenter.x = p_info->K[2];
    opticalCenter.y = p_info->K[5];
    focalLength.x = p_info->K[0];
    focalLength.y = p_info->K[4];

    TagDetectionArray detections;
    apriltag_detector_->process( cv_ptr->image, opticalCenter, detections);
    if (debug_output) {
        ROS_INFO_STREAM("Got " << detections.size() << " detections");
    }

    // Selects currently only one possible marker out of detections
    // TODO: robust version with all possible detected markers
    TagDetection detection;
    bool foundMarker = false;
    bool print_possible_markers = false;
    bool print_impossible_markers = false;
    bool print_impossible_markers_02 = true;
    bool print_hamming_distance_hist = false;
    int num_possible_markers = 0;
    int num_impossible_markers = 0;
    static std::vector<ulong> hamming_distance_hist(6, 0);
    for (size_t i=0; i<detections.size(); ++i)
    {
        if (print_hamming_distance_hist) {
            uint hd = detections[i].hammingDistance;
            /*
            if (hamming_distance_hist.find(hd) != hamming_distance_hist.end()) {
                hamming_distance_hist[hd] = 0;
            }
            */
            hamming_distance_hist[hd]++;
        }

        if (possible_markers_.find(detections[i].id) != possible_markers_.end())
        {
            if (print_possible_markers) {
                printMarkerDetection( detections[i], i, "possible marker" );
            }
            if (!foundMarker) {
                detection = detections[i];
            }
            foundMarker = true;
            num_possible_markers++;
        } else {
            if (print_impossible_markers) {
                printMarkerDetection( detections[i], i, "IMPOSSIBLE marker" );
            }
            if (print_impossible_markers_02) {
                ROS_INFO_STREAM("Found IMPOSSIBLE marker with id:" << detections[i].id);
            }
            num_impossible_markers++;
        }
        
    } 
    if ( print_possible_markers && num_possible_markers>0 &&
         print_impossible_markers && num_impossible_markers>0 ) {
        ROS_INFO(" ");
    }
    if (print_hamming_distance_hist) {
        ROS_INFO("hamming_distance_hist");
        for (int hd=0; hd<6; hd++) {
            ROS_INFO_STREAM("  hd: " << hd << ", num_instances: " << hamming_distance_hist[hd] );
        }
    }

    if (!foundMarker)
        return;

    cv_bridge::CvImage *p_img_rgb;
    if (debug_im) {
        p_img_rgb = new cv_bridge::CvImage( p_img->header, std::string("mono8"), cv_ptr->image );
    }

    bool publish_poses = true;
    icg_msgs::AprilTagDetectionsPtr p_all_tags_msg( new icg_msgs::AprilTagDetections );
    p_all_tags_msg->detections.reserve(detections.size());
    p_all_tags_msg->header.stamp = p_img->header.stamp;
    p_all_tags_msg->header.frame_id = p_img->header.frame_id;
    for (size_t i=0; i<detections.size(); ++i) {
        TagDetection &tag = detections[i];

        if (possible_markers_.find(tag.id) != possible_markers_.end())
        {
            // do nothing, continue execution of the for-loop
        } else {
            // this markers should not be considered, it is likely an outlier,
			//   but importantly we do not even know its size.
            continue;
        }

        p_all_tags_msg->detections.push_back( icg_msgs::AprilTagDetection() );
        icg_msgs::AprilTagDetection &tag_msg = p_all_tags_msg->detections.back();

        tag_msg.id = tag.id;
        tag_msg.tag_size = possible_markers_[tag.id];

        for (int j=0; j<4; j++)  {
            tag_msg.corners2d[j].x = tag.p[j].x;
            tag_msg.corners2d[j].y = tag.p[j].y;
            if (debug_im) {
                // p_img_rgb;
                cv::Point p1a(tag.p[j].x-5, tag.p[j].y);
                cv::Point p1b(tag.p[j].x+5, tag.p[j].y);
                cv::line( p_img_rgb->image, p1a, p1b, cv::Scalar(255)); //crosshair horizontal
                cv::Point p2a(tag.p[j].x,   tag.p[j].y-5);
                cv::Point p2b(tag.p[j].x,   tag.p[j].y+5);
                cv::line( p_img_rgb->image, p2a, p2b, cv::Scalar(255)); //crosshair vertical
            }
        }
        tag_msg.cxy.x = tag.cxy.x;
        tag_msg.cxy.y = tag.cxy.y;

        tag_msg.hxy.x = tag.hxy.x;
        tag_msg.hxy.y = tag.hxy.y;

        // https://stackoverflow.com/a/20981227/896495
        if (tag.homography.isContinuous()) {
            // cv::Mat to std::vector
            //tag_msg.homography.assign(
            //    (float*)tag.homography.datastart, 
            //    (float*)tag.homography.dataend);
            // cv::Mat to boost::array
            std::copy(
                (float*)tag.homography.datastart, 
                (float*)tag.homography.dataend,
                tag_msg.homography.begin());
        } else {
            for (int j=0; j<9; j++) {
                tag_msg.homography[j] = tag.homography.at<float>(j);
            }
        }

        tag_msg.good = tag.good;
        tag_msg.obsCode = tag.obsCode;
        tag_msg.code = tag.code;
        tag_msg.hammingDistance = tag.hammingDistance;
        tag_msg.rotation = tag.rotation;
        tag_msg.observedPerimeter = tag.observedPerimeter;
        
        Eigen::Vector3d t_c_m( Eigen::Vector3d::Zero() );
        Eigen::Quaterniond q_c_m( Eigen::Quaterniond::Identity() );
        if (publish_poses) {
            cv::Mat rcv_c_m, tcv_c_m;
            CameraUtil::homographyToPoseCV(focalLength.x, focalLength.y, possible_markers_[tag.id],
                                        tag.homography,
                                        rcv_c_m, tcv_c_m);
            cv::cv2eigen(tcv_c_m,t_c_m);
            Eigen::Vector3d r_c_m;
            cv::cv2eigen(rcv_c_m, r_c_m);
            q_c_m = Eigen::Quaterniond( Eigen::AngleAxisd( r_c_m.norm(), r_c_m.normalized()) );

            // Eigen::Vector3d t_w_m( 0.0, 0.0, 0.0);
            // Eigen::Matrix3d R_w_m = Eigen::MatrixXd::Identity(3,3);
            // R_w_m = marker_estimated_pose.setRotation(R_w_c * R_c_m);
            // t_w_m = marker_estimated_pose.setCameraCenter(t_w_c + R_w_c * t_c_m);

            if (debug_output) {
                ROS_INFO_STREAM("tag.id: " << tag.id);
                ROS_INFO_STREAM("  R_c_m: " << Eigen::Matrix3d(q_c_m) );
                ROS_INFO_STREAM("  t_c_m: " << t_c_m.transpose() );
            }

            if (false) { //Publish in "~/camera_pose topic"
                geometry_msgs::PoseStampedPtr p_camera_pose_( new geometry_msgs::PoseStamped );
                p_camera_pose_->header.stamp = p_img->header.stamp;
                p_camera_pose_->header.frame_id = p_img->header.frame_id;
                mav_msgs::pointEigenToMsg( t_c_m, &(p_camera_pose_->pose.position) );
                mav_msgs::quaternionEigenToMsg( q_c_m, &(p_camera_pose_->pose.orientation) );
                // pub_camera_pose_.publish(p_camera_pose_);
            }
        }
        mav_msgs::pointEigenToMsg( t_c_m, &(tag_msg.pose.position) );
        mav_msgs::quaternionEigenToMsg( q_c_m, &(tag_msg.pose.orientation) );

        // publish marker as tf
        geometry_msgs::TransformStamped tf_stmpd_;
        tf_stmpd_.header = p_all_tags_msg->header;     
        {       
            std::ostringstream ss;
            ss << "marker_" << tag_msg.id;
            tf_stmpd_.child_frame_id = ss.str();
        }
        tf_stmpd_.transform.rotation = tag_msg.pose.orientation;
        tf_stmpd_.transform.translation.x = tag_msg.pose.position.x;
        tf_stmpd_.transform.translation.y = tag_msg.pose.position.y;
        tf_stmpd_.transform.translation.z = tag_msg.pose.position.z;
        tfbc.sendTransform( tf_stmpd_ );
    }
    pub_detected_markers_.publish(p_all_tags_msg);
    if (debug_im) {
        sensor_msgs::ImagePtr img_rgb_msg = p_img_rgb->toImageMsg();
        camera_pub.publish( img_rgb_msg, p_info);
    }
}

void IcgApriltagMarkerDetectorRos::onIn(const std_msgs::Int32::ConstPtr& p_msg) 
{
    // filter task_scheduler commands to the range of this node
    if ( !( ( 600 <= p_msg->data ) && ( p_msg->data < 650 ) ) )
        return;
    switch (p_msg->data) {
    case CdlMsgsCommon::APRILTAG_SEARCH_MARKERS: {
        ROS_INFO("TSI: APRILTAG_SEARCH_MARKERS");
        startDetectingMarkers();
        } break;
    case CdlMsgsCommon::APRILTAG_SET_IDLE: {
        ROS_INFO("TSI: APRILTAG_SET_IDLE");
        stopDetectingMarkers();
        } break;    
        default: {
        ROS_INFO("TSI: Unknown task scheduler command.");
        } break;
    }
}