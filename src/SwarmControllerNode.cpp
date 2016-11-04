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


SwarmControllerNode::SwarmControllerNode(ros::NodeHandle *nh)
  : Node(nh, 30)
{
    enableControl_ = false;
    migrationPoint_.x = 0.0;
    migrationPoint_.y = 0.0;
    position_.x = 0.0;
    position_.y = 0.0;
    r1_ = 0.0;
    r2_ = 0.0;
    r3_ = 0.0;
    r4_ = 1.0;
    ros::param::get("swarm_controller_node/uav_id", id_);

    // Add some neighbors
    /*
    geometry_msgs::Point32 point;
    geometry_msgs::Point32 point2;
    geometry_msgs::Point32 point3;
    point.x = 0.0;
    point.y = 0.0;
    neighbors_.push_back(point);
    point2.x = -6.0;
    point2.y = 6.0;
    neighbors_.push_back(point2);
    point3.x = 8.0;
    point3.y = -1.0;
    neighbors_.push_back(point3);*/

    mavros_state_sub_ = nh->subscribe("mavros/state", 1, &SwarmControllerNode::mavrosStateCb, this);
    migration_point_sub_ = nh->subscribe("/migration_point", 1, &SwarmControllerNode::migrationPointCb, this);
    odom_sub_ = nh->subscribe("mavros/local_position/odom", 1, &SwarmControllerNode::odomCb, this);
    enable_control_sub_ = nh->subscribe("enable_control", 1, &SwarmControllerNode::enableControlCb, this);
    uav_position_sub_ = nh->subscribe("/uav_position", 1, &SwarmControllerNode::uavPositionCb, this);
    rc_override_pub_ = nh->advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    cmd_vel_pub_ = nh->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    position_pub_ = nh->advertise<swarm_control::UavPosition>("/uav_position", 10);
}

SwarmControllerNode::~SwarmControllerNode()
{
    mavros_state_sub_.shutdown();
    migration_point_sub_.shutdown();
    odom_sub_.shutdown();
    enable_control_sub_.shutdown();
    uav_position_sub_.shutdown();
    rc_override_pub_.shutdown();
    cmd_vel_pub_.shutdown();
    position_pub_.shutdown();
}

void SwarmControllerNode::controlLoop()
{
    // Print information on screen
    char tab2[1024];
    strncpy(tab2, mode_.c_str(), sizeof(tab2));
    tab2[sizeof(tab2) - 1] = 0;
    //ROS_INFO("Objective = (%f , %f) \n Roll = %f | Pitch = %f\n Mode = %s \n UavX = %f | UavY = %f\n UAV_ID = %d | enableControl = %d\n Neighbors = %d",
           //  migrationPoint_.x, migrationPoint_.y, roll_, pitch_, tab2, position_.x, position_.y, id_, enableControl_, neighbors_.size());

    // Calculate each rule's influence
    geometry_msgs::Point32 *v1 = rule1();
    geometry_msgs::Point32 *v2 = rule2();
    geometry_msgs::Point32 *v3 = rule3();
    geometry_msgs::Point32 *v4 = rule4();

    // Combine the rules
    geometry_msgs::Point32 vRes;
    vRes.x = v1->x + v2->x + v3->x + v4->x;
    vRes.y = v1->y + v2->y + v3->y + v4->y;
    vRes.z = v1->z + v2->z + v3->z + v4->z;

    // Calculate Roll and Pitch depending on the mode
    if (mode_ == "LOITER"){
        roll_ = BASERC - vRes.x * FACTOR;
        pitch_ = BASERC + vRes.y * FACTOR;
    }else{
        roll_ = BASERC;
        pitch_ = BASERC;
    }

    // Limit the Roll
    if (roll_ > MAXRC)
    {
        roll_ = MAXRC;
    } else if (roll_ < MINRC)
    {
        roll_ = MINRC;
    }

    // Limit the Pitch
    if (pitch_ > MAXRC)
    {
        pitch_ = MAXRC;
    } else if (pitch_ < MINRC)
    {
        pitch_ = MINRC;
    }

    //publishRCOverride();

    // Limit the Velocity in x
    if (vRes.x > MAXVEL)
    {
        vRes.x = MAXVEL;
    } else if (vRes.x < MINVEL)
    {
        vRes.x = MINVEL;
    }

    // Limit the Velocity in y
    if (vRes.y > MAXVEL)
    {
        vRes.y = MAXVEL;
    } else if (vRes.y < MINVEL)
    {
        vRes.y = MINVEL;
    }

    //ROS_INFO("vRes.x = %f | vRes.y = %f | vRes.z = %f\n", vRes.x, vRes.y, vRes.z);

    // PUBLISH VELOCITY
    if ( enableControl_ == true )
    {
        publishVelocity(vRes.x, vRes.y);
    }

    publishPosition();
}

void SwarmControllerNode::publishRCOverride()
{
    // Create RC msg
    mavros_msgs::OverrideRCIn msg;

    msg.channels[0] = roll_;     //Roll
    msg.channels[1] = pitch_;    //Pitch
    msg.channels[2] = BASERC;   //Throttle
    msg.channels[3] = 0;        //Yaw
    msg.channels[4] = 0;
    msg.channels[5] = 0;
    msg.channels[6] = 0;
    msg.channels[7] = 0;

    rc_override_pub_.publish(msg);
}

void SwarmControllerNode::publishVelocity( double velX, double velY )
{
    // Create Twist msg
    geometry_msgs::TwistStamped msg;

    msg.twist.linear.x = velX;
    msg.twist.linear.y = velY;

    cmd_vel_pub_.publish(msg);
}

void SwarmControllerNode::publishPosition()
{
    // Create UavPosition msg
    swarm_control::UavPosition msg;

    msg.id = id_;
    msg.position.x = position_.x;
    msg.position.y = position_.y;
    msg.position.z = position_.z;

    position_pub_.publish(msg);
}

void SwarmControllerNode::mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode_ = msg->mode;
    guided_ = msg->guided==128;
    armed_ = msg->armed==128;
}

void SwarmControllerNode::migrationPointCb( const geometry_msgs::Point32ConstPtr &msg )
{
    migrationPoint_.x = msg->x;
    migrationPoint_.y = msg->y;
}

void SwarmControllerNode::odomCb( const nav_msgs::OdometryConstPtr &msg )
{
    position_.x = msg->pose.pose.position.x;
    position_.y = msg->pose.pose.position.y;
}

void SwarmControllerNode::enableControlCb( const std_msgs::BoolConstPtr &msg )
{
    enableControl_ = msg->data;
}

void SwarmControllerNode::uavPositionCb( const swarm_control::UavPositionConstPtr &msg )
{
    if ( msg->id != id_ )
    {
      bool updated = false;

      for ( int i = 0; i < neighbors_.size(); i++ )
      {
        if ( neighbors_[i].id == msg->id )
        {
          neighbors_[i].position.x = msg->position.x;
          neighbors_[i].position.y = msg->position.y;
          neighbors_[i].position.z = msg->position.z;

          updated = true;
        }
      }

      if ( updated == false )
      {
        swarm_control::UavPosition newPosition;

        newPosition.id = msg->id;
        newPosition.position.x = msg->position.x;
        newPosition.position.y = msg->position.y;
        newPosition.position.z = msg->position.z;

        neighbors_.push_back(newPosition);
      }

      if ( id_ == 0 )
      {
        ROS_INFO("This id: %d", id_);
        ROS_INFO("Message id: %d", msg->id);
        ROS_INFO("Updated: %d", updated);
      }
    }
}


// REYNOLDS RULES

// Rule 1: Flocking
geometry_msgs::Point32* SwarmControllerNode::rule1()
{
    geometry_msgs::Point32 v1;

    v1.x = 0.0;
    v1.y = 0.0;
    v1.z = 0.0;

    int n = neighbors_.size();

    if ( n > 0 )
    {
        geometry_msgs::Point32 centerOfMass;

        centerOfMass.x = 0.0;
        centerOfMass.y = 0.0;
        centerOfMass.z = 0.0;

        for ( int i(0); i < n; i++ )
        {
            centerOfMass.x += neighbors_[i].position.x;
            centerOfMass.y += neighbors_[i].position.y;
            centerOfMass.z += neighbors_[i].position.z;
        }

        centerOfMass.x *= ( 1 / n );
        centerOfMass.y *= ( 1 / n );
        centerOfMass.z *= ( 1 / n );

        v1.x = centerOfMass.x - position_.x;
        v1.y = centerOfMass.y - position_.y;
        v1.z = centerOfMass.z - position_.z;
    }

    v1.x *= r1_;
    v1.y *= r1_;
    v1.z *= r1_;

    geometry_msgs::Point32 *v1Ptr = &v1;

    return v1Ptr;
}


// Rule 2: Collision Avoidance
geometry_msgs::Point32* SwarmControllerNode::rule2()
{
    geometry_msgs::Point32 v2;

    v2.x = 0.0;
    v2.y = 0.0;
    v2.z = 0.0;

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            double d = sqrt( pow( (neighbors_[i].position.x - position_.x), 2 ) +
                             pow( (neighbors_[i].position.y - position_.y), 2 ) +
                             pow( (neighbors_[i].position.z - position_.z), 2 ) );

            if ( d < VISION_DISTANCE )
            {
                double dif = VISION_DISTANCE - d;

                geometry_msgs::Point32 v;

                v.x = neighbors_[i].position.x - position_.x;
                v.y = neighbors_[i].position.y - position_.y;
                v.z = neighbors_[i].position.z - position_.z;

                double vm = sqrt( pow( (v.x), 2 ) + pow( (v.y), 2 ) + pow( (v.z), 2 ) );

                v.x /= vm;
                v.y /= vm;
                v.z /= vm;

                v.x *= dif;
                v.y *= dif;
                v.z *= dif;

                v2.x -= v.x;
                v2.y -= v.y;
                v2.z -= v.z;
            }
        }
    }

    v2.x *= r2_;
    v2.y *= r2_;
    v2.z *= r2_;

    geometry_msgs::Point32 *v2Ptr = &v2;

    return v2Ptr;
}

// Rule 3: Velocity Matching
geometry_msgs::Point32* SwarmControllerNode::rule3()
{
    geometry_msgs::Point32 v3;

    v3.x = 0.0;
    v3.y = 0.0;
    v3.z = 0.0;

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            v3.x += neighbors_[i].position.x;
            v3.y += neighbors_[i].position.y;
            v3.z += neighbors_[i].position.z;
        }

        v3.x *= ( 1 / n );
        v3.y *= ( 1 / n );
        v3.z *= ( 1 / n );

        v3.x = v3.x - position_.x;
        v3.y = v3.y - position_.y;
        v3.z = v3.z - position_.z;
    }

    v3.x *= r3_;
    v3.y *= r3_;
    v3.z *= r3_;

    geometry_msgs::Point32 *v3Ptr = &v3;

    return v3Ptr;
}


// Rule 4: Migration
geometry_msgs::Point32* SwarmControllerNode::rule4()
{
    geometry_msgs::Point32 v4;

    v4.x = 0.0;
    v4.y = 0.0;
    v4.z = 0.0;

    v4.x = migrationPoint_.x - position_.x;
    v4.y = migrationPoint_.y - position_.y;
    v4.z = migrationPoint_.z - position_.z;

    v4.x *= r4_;
    v4.y *= r4_;
    v4.z *= r4_;

    //ROS_INFO("v4.x = %f | v4.y = %f | v4.z = %f\n", v4.x, v4.y, v4.z);

    geometry_msgs::Point32 *v4Ptr = &v4;

    return v4Ptr;
}
