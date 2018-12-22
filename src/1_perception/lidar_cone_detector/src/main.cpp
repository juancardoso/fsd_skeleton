/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits <britss@ethz.ch>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "cone_detector_handle.hpp"
#include <sensor_msgs/PointCloud2.h>

typedef ns_cone_detector::ConeDetectorHandle ConeDetectorHandle;


void velodynePointsHandler(const sensor_msgs::PointCloud2::ConstPtr& pointCloud )
{
  	ROS_INFO("Receiving infos from velodyne_points: Height:%d Width:%d", pointCloud->height, pointCloud->width); 
}

int main(int argc, char **argv) {
 ros::init(argc, argv, "cone_detector");
  ros::NodeHandle nodeHandle("~");
  ConeDetectorHandle myConeDetectorHandle(nodeHandle);
  ros::Subscriber lidar = nodeHandle.subscribe("/velodyne_points",100,velodynePointsHandler);
  ros::Rate loop_rate(myConeDetectorHandle.getNodeRate());
  while (ros::ok()) {

    myConeDetectorHandle.run();

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

