/**
 *  This source file implements the ConversorNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 04/11/2016
 *  Modified on: 04/11/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "ConversorNode.h"


ConversorNode::ConversorNode(ros::NodeHandle *nh)
  : Node(nh, 10)
{
    for ( int i = 0; i <= 2; i++ )
    {
        swarm_control::UavPosition pos;
        pos.id = i;
        pos.position.x = 0.0;
        pos.position.y = 0.0;
        pos.position.z = 0.0;
        positions_.positions.push_back(pos);
    }

    uav0_position_sub_ = nh->subscribe("/uav0/mavros/local_position/odom", 1, &ConversorNode::uav0PositionCb, this);
    uav1_position_sub_ = nh->subscribe("/uav1/mavros/local_position/odom", 1, &ConversorNode::uav1PositionCb, this);
    uav2_position_sub_ = nh->subscribe("/uav2/mavros/local_position/odom", 1, &ConversorNode::uav2PositionCb, this);
    uav_positions_pub_ = nh->advertise<swarm_control::UavPositionArray>("/uav_positions", 1);
}

ConversorNode::~ConversorNode()
{
    uav0_position_sub_.shutdown();
    uav1_position_sub_.shutdown();
    uav2_position_sub_.shutdown();
    uav_positions_pub_.shutdown();
}

void ConversorNode::controlLoop()
{
    publishUavPositions();
}

void ConversorNode::publishUavPositions()
{
    uav_positions_pub_.publish(positions_);
}

void ConversorNode::uav0PositionCb( const nav_msgs::OdometryConstPtr &msg )
{
    positions_.positions[0].position.x = msg->pose.pose.position.x;
    positions_.positions[0].position.y = msg->pose.pose.position.y;
    positions_.positions[0].position.z = msg->pose.pose.position.z;
}

void ConversorNode::uav1PositionCb( const nav_msgs::OdometryConstPtr &msg )
{
    positions_.positions[1].position.x = msg->pose.pose.position.x;
    positions_.positions[1].position.y = msg->pose.pose.position.y;
    positions_.positions[1].position.z = msg->pose.pose.position.z;
}

void ConversorNode::uav2PositionCb( const nav_msgs::OdometryConstPtr &msg )
{
    positions_.positions[2].position.x = msg->pose.pose.position.x;
    positions_.positions[2].position.y = msg->pose.pose.position.y;
    positions_.positions[2].position.z = msg->pose.pose.position.z;
}
