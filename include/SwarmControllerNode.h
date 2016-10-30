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

#include "Node.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>

class SwarmControllerNode : public Node
{
public:
  SwarmControllerNode(ros::NodeHandle *nh);
  virtual ~SwarmControllerNode();

private:
  virtual void controlLoop();

  ros::Subscriber mavros_state_sub_;    // Subscriber to flight mode
  ros::Subscriber migration_point_sub_; // Subscriber to migration point
  ros::Subscriber odom_sub_;            // Subscriber to odometry
  ros::Publisher rc_override_pub_;      // RC publisher

  ros::Publisher vel_pub_;
  ros::Subscriber sonar_sub_;
  std::vector<double> ultrassonics_;
  void publishVelocity(double vel_x, double vel_theta);
  void sonarCb( const sensor_msgs::PointCloud::ConstPtr& msg);


};

#endif // _SWARM_CONTROLLER_NODE_H_
