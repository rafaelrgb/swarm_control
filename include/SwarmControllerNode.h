/**
 *  This header file defines the SwarmControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 30/10/2016
 *  Modified on: 30/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _SWARM_CONTROLLER_NODE_H_
#define _SWARM_CONTROLLER_NODE_H_

#include <string>
#include <math.h>
#include "Node.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <swarm_control/UavPosition.h>
#include <swarm_control/UavPositionArray.h>

#define MAXVEL   5.0
#define MINVEL   -5.0

#define VISION_DISTANCE 100.0

class SwarmControllerNode : public Node
{
public:
  SwarmControllerNode(ros::NodeHandle *nh);
  virtual ~SwarmControllerNode();

private:
  virtual void controlLoop();

  // UAV id
  int id_;

  // Enable control
  bool enableControl_;

  // Rules weights
  double r1_;
  double r2_;
  double r3_;
  double r4_;

  // Position of elements in the system
  geometry_msgs::Point32 position_;
  geometry_msgs::Point32 migrationPoint_;
  swarm_control::UavPositionArray neighbors_;

  // Flight mode
  std::string mode_;
  bool guided_;
  bool armed_;

  // ROS objects
  ros::Subscriber mavros_state_sub_;    // Subscriber to flight mode
  ros::Subscriber migration_point_sub_; // Subscriber to migration point
  ros::Subscriber odom_sub_;            // Subscriber to odometry
  ros::Subscriber enable_control_sub_;  // Subscriber to enable or disable control
  ros::Subscriber uav_positions_sub_;    // Subscriber all uav positions
  ros::Publisher cmd_vel_pub_;          // Velocity publisher

  // Member functions
  void mavrosStateCb( const mavros_msgs::StateConstPtr &msg );
  void migrationPointCb( const geometry_msgs::Point32ConstPtr &msg );
  void odomCb( const nav_msgs::OdometryConstPtr &msg );
  void enableControlCb( const std_msgs::BoolConstPtr &msg );
  void uavPositionsCb( const swarm_control::UavPositionArrayConstPtr &msg );
  void publishVelocity( double velX, double velY );

  // Rules
  geometry_msgs::Point32* rule1();
  geometry_msgs::Point32* rule2();
  geometry_msgs::Point32* rule3();
  geometry_msgs::Point32* rule4();

};

#endif // _SWARM_CONTROLLER_NODE_H_
