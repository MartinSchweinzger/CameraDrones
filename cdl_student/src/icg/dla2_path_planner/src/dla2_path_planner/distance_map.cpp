/*
 * DLA2 Path Planner ROS - distance_map.cpp
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Jan 8, 2019
 *
 */

#include <dla2_path_planner/distance_map.h>

DLA2DistanceMap::DLA2DistanceMap(const std::string &bt_file_path, float maxDist, bool unknownAsOccupied) :
    p_tree( std::shared_ptr<octomap::OcTree>( new octomap::OcTree(0.05) ) )
{
  ROS_INFO_STREAM("DLA2DistanceMap: parameter maxDist is "<<maxDist<<" [m].");
  ROS_INFO_STREAM("DLA2DistanceMap: parameter unknownAsOccupied is "<<unknownAsOccupied<<" [bool].");

  //read in octotree
  p_tree->readBinary( bt_file_path.c_str() );
  ROS_INFO_STREAM("DLA2DistanceMap: Read in tree, "<<p_tree->getNumLeafNodes()<<" leaves.");
  resolution = p_tree->getResolution();

  //get maximum dimensions of the octotree
  double x,y,z;
  p_tree->getMetricMin(x,y,z);
  min = octomap::point3d(x,y,z);
  ROS_INFO_STREAM("DLA2DistanceMap: octree metric min point: "<<x<<","<<y<<","<<z<<" [m]");
  p_tree->getMetricMax(x,y,z);
  max = octomap::point3d(x,y,z);
  ROS_INFO_STREAM("DLA2DistanceMap: octree metric max point: "<<x<<","<<y<<","<<z<<" [m]");

  //initialize p_distmap
  //- the first argument ist the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  p_distmap = std::shared_ptr<DynamicEDTOctomap>( 
      new DynamicEDTOctomap(maxDist, p_tree.get(), min, max, unknownAsOccupied) );

  //This computes the distance map
  p_distmap->update(); 
}

DLA2DistanceMap::~DLA2DistanceMap() {

}
