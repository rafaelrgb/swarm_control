/**
 *  This source file implements the SwarmControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 30/10/2016
 *  Modified on: 30/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "SwarmControllerNode.h"

namespace aula6
{

SwarmControllerNode::SwarmControllerNode(ros::NodeHandle *nh)
  : Node(nh, 30)
{
  vel_pub_ = nh->advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  sonar_sub_ = nh->subscribe("RosAria/sonar", 1, &SwarmControllerNode::sonarCb, this);
}

SwarmControllerNode::~SwarmControllerNode()
{
  vel_pub_.shutdown();
  sonar_sub_.shutdown();
}

void SwarmControllerNode::controlLoop()
{
  double light(ambient_->getMinimum() + counter_++ * (ambient_->range() / 50));
  ambient_->setInputValue(light);
  engine_->process();
  double output(power_->getOutputValue());
  //ROS_INFO("Ambient.input = %f -> Power.output = %f", light, output);
  if (light > 1)
  {
    counter_ = 0;
  }
  publishVelocity(100 * output, output);
  for ( int i(0); i < ultrassonics_.size(); i++ )
  {
    //ROS_WARN("%d: %f", i, ultrassonics_[i]);
  }
}

void SwarmControllerNode::publishVelocity(double vel_x, double vel_theta)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vel_x;
  msg.angular.z = vel_theta;
  vel_pub_.publish(msg);
}

void SwarmControllerNode::sonarCb(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  ultrassonics_.clear();
  for (int i = 0; i < msg->points.size(); i++)
  {
    geometry_msgs::Point32 point(msg->points[i]);
    ultrassonics_.push_back(sqrt(pow(point.x, 2) + pow(point.y, 2)));
  }
  //ROS_INFO("Distancia do quarto sensor: %f", ultrassonics_[3]);
}

}
