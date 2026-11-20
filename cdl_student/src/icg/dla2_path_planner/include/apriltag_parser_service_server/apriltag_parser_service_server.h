/*
 *  Apriltag Parser Service Server - apriltag_parser_service_server.h
 *
 *  Author: Jun Zhang <jun.zhang@tugraz.at>
 *  Created on: October, 2024
 *
 */

#ifndef PARSE_APRILTAG_SERVICE_SERVER_H_
#define PARSE_APRILTAG_SERVICE_SERVER_H_

#include "ros/ros.h"
#include <ros/package.h>  

#include <icg_srvs/SetAprilTagParameter.h>

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

class ParsePariltagServiceServer
{
public:
    ParsePariltagServiceServer(ros::NodeHandle &n, ros::NodeHandle &pn);
    ~ParsePariltagServiceServer();

private:
    ros::NodeHandle &pnode_;
    ros::NodeHandle &node_;

    ros::ServiceServer service_parse_apriltag_info;
    bool serviceServerParseAprilTagInfo(icg_srvs::SetAprilTagParameter::Request  &req, 
                                        icg_srvs::SetAprilTagParameter::Response &res );
};

#endif // PARSE_APRILTAG_SERVICE_SERVER_H_
