/**
 *  This header file defines the ConversorNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 04/11/2016
 *  Modified on: 04/11/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _CONVERSOR_NODE_H_
#define _CONVERSOR_NODE_H_

#include <string>
#include <math.h>
#include "Node.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <swarm_control/UavPosition.h>
#include <swarm_control/UavPositionArray.h>

class ConversorNode : public Node
{
public:
  ConversorNode(ros::NodeHandle *nh);
  virtual ~ConversorNode();

private:
  virtual void controlLoop();

  swarm_control::UavPositionArray positions_;

  // ROS objects
  ros::Subscriber uav0_position_sub_;
  ros::Subscriber uav1_position_sub_;
  ros::Subscriber uav2_position_sub_;
  ros::Publisher uav_positions_pub_;

  // Member functions
  void uav0PositionCb( const nav_msgs::OdometryConstPtr &msg );
  void uav1PositionCb( const nav_msgs::OdometryConstPtr &msg );
  void uav2PositionCb( const nav_msgs::OdometryConstPtr &msg );
  void publishUavPositions();

};

#endif // _CONVERSOR_NODE_H_
