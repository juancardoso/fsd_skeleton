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
#include "velocity_estimator_handle.hpp"
#include "fsd_common_msgs/CarStateDt.h"
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Pose.h>
// #include <nav_msgs/Odometry.h>

namespace ns_velocity_estimation {

// Constructor
VelocityEstimatorHandle::VelocityEstimatorHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  publishToTopics();
  subscribeToTopics();
}

// Getters
int VelocityEstimatorHandle::getNodeRate() const { return node_rate_; }

// Methods
void VelocityEstimatorHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  
  if (!nodeHandle_.param<std::string>("velocity_estimation_topic_name",
                                      velocity_estimation_topic_name_,
                                      "/estimation/velocity_estimation/velocity_estimate")) {
    ROS_WARN_STREAM("Did not load velocity_estimation_topic_name. Standard value is: " << velocity_estimation_topic_name_);
  }

  if (!nodeHandle_.param<std::string>("ground_truth_republisher",
                                      ground_truth_name_,
                                      "/ground_truth_republisher")) {
    ROS_WARN_STREAM("Did not load ground_truth_republisher. Standard value is: " << ground_truth_name_);
  }

  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void VelocityEstimatorHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  velocityEstimationPublisher = nodeHandle_.advertise<fsd_common_msgs::CarStateDt>(velocity_estimation_topic_name_, 1);
}

void VelocityEstimatorHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  groundTruthSubscriber = nodeHandle_.subscribe(ground_truth_name_, 1, &VelocityEstimatorHandle::GroundTruthCallback,this);
}

void VelocityEstimatorHandle::run() {
  velocityEstimator_.runAlgorithm();
  sendVelocityEstimate();
}

void VelocityEstimatorHandle::sendVelocityEstimate() {
  velocity_estimate_.car_state_dt.x = velocityEstimator_.getVelocityEstimate().x;
  velocity_estimate_.car_state_dt.y = velocityEstimator_.getVelocityEstimate().y;
  velocity_estimate_.car_state_dt.theta = velocityEstimator_.getVelocityEstimate().theta;
  velocity_estimate_.header.stamp = ros::Time::now();
  velocityEstimationPublisher.publish(velocity_estimate_);
  ROS_INFO("Velocity estimate sent");
}


void VelocityEstimatorHandle::GroundTruthCallback(const fsd_common_msgs::CarStateDt msg){
  ROS_INFO("Teste");
}

// void VelocityEstimatorHandle::GroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg){
//   ROS_INFO("Seq: [%d]", msg->header.seq);
// 	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
// 	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
// 	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
// }
}
