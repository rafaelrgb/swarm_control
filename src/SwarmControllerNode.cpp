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
  : Node(nh, 10)
{
    enableControl_ = false;
    migrationPoint_.x = 0.0;
    migrationPoint_.y = 0.0;
    position_.x = 0.0;
    position_.y = 0.0;
    r1_ = 1.0;
    r2_ = 1.0;
    r3_ = 0.0;
    r4_ = 1.0;
    ros::param::get("swarm_controller_node/uav_id", id_);

    mavros_state_sub_ = nh->subscribe("mavros/state", 1, &SwarmControllerNode::mavrosStateCb, this);
    migration_point_sub_ = nh->subscribe("/migration_point", 1, &SwarmControllerNode::migrationPointCb, this);
    odom_sub_ = nh->subscribe("mavros/local_position/odom", 1, &SwarmControllerNode::odomCb, this);
    enable_control_sub_ = nh->subscribe("enable_control", 1, &SwarmControllerNode::enableControlCb, this);
    uav_positions_sub_ = nh->subscribe("/uav_positions", 1, &SwarmControllerNode::uavPositionsCb, this);
    cmd_vel_pub_ = nh->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
}

SwarmControllerNode::~SwarmControllerNode()
{
    mavros_state_sub_.shutdown();
    migration_point_sub_.shutdown();
    odom_sub_.shutdown();
    enable_control_sub_.shutdown();
    uav_positions_sub_.shutdown();
    cmd_vel_pub_.shutdown();
}

void SwarmControllerNode::controlLoop()
{
    // Print information on screen
    if ( id_ == 0 )
    {
        char tab2[1024];
        strncpy(tab2, mode_.c_str(), sizeof(tab2));
        tab2[sizeof(tab2) - 1] = 0;
        ROS_INFO("UAV %d: \n Mode = %s | enableControl = %d \n Position = (%f, %f) | Objective = (%f , %f)",
                 id_, tab2, enableControl_, position_.x, position_.y, migrationPoint_.x, migrationPoint_.y);
    }

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

    if ( id_ == 0 )
    {
        ROS_INFO("vRes = (%f, %f, %f)", vRes.x, vRes.y, vRes.z);
    }

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

    // PUBLISH VELOCITY
    if ( enableControl_ == true )
    {
        publishVelocity(vRes.x, vRes.y);
    }


    // TESTE: Imprimir os neighbors para ver se o robô está detectando corretamente
    /*int n = neighbors_.positions.size();
    if ( n > 0 )
    {
        ROS_INFO("UAV %d neighbors:", id_);
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_.positions[i].id != id_ )
            {
                ROS_INFO("[\n\tid: %d,\n\tx: %f,\n\ty: %f,\n\tz: %f\n]",
                         neighbors_.positions[i].id,
                         neighbors_.positions[i].position.x,
                         neighbors_.positions[i].position.y,
                         neighbors_.positions[i].position.z);
            }
        }
    }*/
}

void SwarmControllerNode::publishVelocity( double velX, double velY )
{
    geometry_msgs::TwistStamped msg;

    msg.twist.linear.x = velX;
    msg.twist.linear.y = velY;

    cmd_vel_pub_.publish(msg);
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

void SwarmControllerNode::uavPositionsCb( const swarm_control::UavPositionArrayConstPtr &msg )
{
    neighbors_.positions.clear();
    for ( int i = 0; i < msg->positions.size(); i ++ )
    {
        swarm_control::UavPosition pos;
        pos.id = msg->positions[i].id;
        pos.position.x = msg->positions[i].position.x;
        pos.position.y = msg->positions[i].position.y;
        pos.position.z = msg->positions[i].position.z;
        neighbors_.positions.push_back(pos);
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

    int n = neighbors_.positions.size();

    if ( n > 0 )
    {
        geometry_msgs::Point32 centerOfMass;

        centerOfMass.x = 0.0;
        centerOfMass.y = 0.0;
        centerOfMass.z = 0.0;

        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_.positions[i].id != id_ )
            {
                centerOfMass.x += neighbors_.positions[i].position.x;
                centerOfMass.y += neighbors_.positions[i].position.y;
                centerOfMass.z += neighbors_.positions[i].position.z;
            }
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

    if ( id_ == 0 )
    {
        ROS_INFO("v1 = (%f, %f, %f)", v1.x, v1.y, v1.z);
    }

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

    int n = neighbors_.positions.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_.positions[i].id != id_ )
            {
                double d = sqrt( pow( (neighbors_.positions[i].position.x - position_.x), 2 ) +
                                 pow( (neighbors_.positions[i].position.y - position_.y), 2 ) +
                                 pow( (neighbors_.positions[i].position.z - position_.z), 2 ) );

                if ( d < VISION_DISTANCE )
                {
                    double dif = VISION_DISTANCE - d;

                    geometry_msgs::Point32 v;

                    v.x = neighbors_.positions[i].position.x - position_.x;
                    v.y = neighbors_.positions[i].position.y - position_.y;
                    v.z = neighbors_.positions[i].position.z - position_.z;

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
    }

    v2.x *= r2_;
    v2.y *= r2_;
    v2.z *= r2_;

    if ( id_ == 0 )
    {
        ROS_INFO("v2 = (%f, %f, %f)", v2.x, v2.y, v2.z);
    }

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

    int n = neighbors_.positions.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_.positions[i].id != id_ )
            {
                v3.x += neighbors_.positions[i].position.x;
                v3.y += neighbors_.positions[i].position.y;
                v3.z += neighbors_.positions[i].position.z;
            }
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

    if ( id_ == 0 )
    {
        ROS_INFO("v3 = (%f, %f, %f)", v3.x, v3.y, v3.z);
    }

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

    if ( id_ == 0 )
    {
        ROS_INFO("v4 = (%f, %f, %f)", v4.x, v4.y, v4.z);
    }

    geometry_msgs::Point32 *v4Ptr = &v4;

    return v4Ptr;
}
