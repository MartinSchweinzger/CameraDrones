/*
 * DLA2 Path Planner ROS - distance_map.h
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Jan 8, 2019
 *
 */

#include <ros/ros.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <iostream>
#include <memory> // std::shared_ptr

#ifndef DLA2_DISTANCE_MAP_H_
#define DLA2_DISTANCE_MAP_H_

class DLA2DistanceMap
{
public:
    DLA2DistanceMap(const std::string &bt_file_path, float maxDist = 5.0, bool unknownAsOccupied = true);
    ~DLA2DistanceMap();

    const DynamicEDTOctomap &getDistanceMap() const { return *p_distmap; }
    const octomap::OcTree &getOcTree() const { return *p_tree; }
    const octomap::point3d &getMapMin() const { return min; }
    const octomap::point3d &getMapMax() const { return max; }
    const double &getResolution() const { return resolution; }

private:
    std::shared_ptr<octomap::OcTree> p_tree;
    std::shared_ptr<DynamicEDTOctomap> p_distmap;
    octomap::point3d min, max; // map extreme points
    double resolution;
};

#endif // DLA2_DISTANCE_MAP_H_