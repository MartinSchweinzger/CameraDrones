/*
 *  Apriltag Parser Service Server ROS node - apriltag_parser_service_server_ros_node.cpp
 *
 *  Author: Jun Zhang <jun.zhang@tugraz.at>
 *  Created on: October, 2024
 *
 */

#include <iostream>
#include "ros/ros.h"
#include <apriltag_parser_service_server/apriltag_parser_service_server.h>

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "apriltag_parser_service_server");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::cout << "[ROSNODE] Starting ApriltagParserServiceServer" << std::endl;

    ParsePariltagServiceServer apriltag_parser_service_server(n,pn);
 
    try
    {
        //Read messages
        ros::spin();
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}

