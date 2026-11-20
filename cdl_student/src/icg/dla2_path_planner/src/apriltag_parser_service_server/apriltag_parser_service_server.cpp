/*
 *  Apriltag Parser Service Server - apriltag_parser_service_server.cpp
 *
 *  Author: Jun Zhang <jun.zhang@tugraz.at>
 *  Created on: October, 2024
 *
 */

#include <apriltag_parser_service_server/apriltag_parser_service_server.h>

ParsePariltagServiceServer::ParsePariltagServiceServer(ros::NodeHandle &n, ros::NodeHandle &pn) :
    pnode_(pn),
    node_(n)
{   
    service_parse_apriltag_info = pnode_.advertiseService("parse_apriltag_info", &ParsePariltagServiceServer::serviceServerParseAprilTagInfo, this);
}

ParsePariltagServiceServer::~ParsePariltagServiceServer() {

}


bool ParsePariltagServiceServer::serviceServerParseAprilTagInfo(icg_srvs::SetAprilTagParameter::Request  &req, 
                                                     icg_srvs::SetAprilTagParameter::Response &res )
{

    const int marker_id = req.id;

    // get the goal waypoint and heading from marker id
    geometry_msgs::Point &gw = res.goal_position; // gw: goal waypoint
    switch (marker_id) {
    case 14: {
        gw.x = 1.5; gw.y = -0.5; gw.z = 1.1; // gw.y = 1.0;
        res.yaw = 0.0;
        res.success = true;
        } break;
    case 16: {
        gw.x = -0.5; gw.y = 1.5; gw.z = 1.1;
        res.yaw = 3.14159;
        res.success = true;
        } break;
    case 18: {
        gw.x = -1.5; gw.y = -1.5; gw.z = 1.1;
        res.yaw = -1.5708;
        res.success = true;
        } break;
    }

    res.message = "Success!";
    res.success = true;

    return res.success;
}
