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
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>


typedef ns_cone_detector::ConeDetectorHandle ConeDetectorHandle;

void camLeftHandler(const sensor_msgs::Image::ConstPtr& image)
{
	ROS_DEBUG("Height:%d Width:%d", image->height, image->width);
}

void camRightHandler(const sensor_msgs::Image::ConstPtr& image)
{
	ROS_DEBUG("Height:%d Width:%d", image->height, image->width);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "cone_detector");
	ros::NodeHandle nodeHandle("~");

	ConeDetectorHandle myConeDetectorHandle(nodeHandle);
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Subscriber camLeft = it.subscribe("/zed/left/image_raw", 5, camLeftHandler);
	image_transport::Subscriber camRight = it.subscribe("/zed/right/image_raw", 5, camRightHandler);
	ros::Rate loop_rate(myConeDetectorHandle.getNodeRate());
	while (ros::ok()) {

		myConeDetectorHandle.run();

		ros::spinOnce();                // Keeps node alive basically
		loop_rate.sleep();              // Sleep for loop_rate
	}
	return 0;
}

