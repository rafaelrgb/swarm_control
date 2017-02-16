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
    dx_ = 0.0;
    dy_ = 0.0;
    initialDeltasCalculated_ = false;
    migrationPoint_.x = 0.0;
    migrationPoint_.y = 0.0;
    position_.x = 0.0;
    position_.y = 0.0;
    position_.z = 0.0;
    vRes_.x = 0.0;
    vRes_.y = 0.0;
    vRes_.z = 0.0;
    r1_ = 1.0;
    r2_ = 1.0;
    r3_ = 1.0;
    r4_ = 1.0;
    ros::param::get("swarm_controller_node/uav_id", id_);

    mavros_state_sub_ = nh->subscribe("mavros/state", 1, &SwarmControllerNode::mavrosStateCb, this);
    migration_point_sub_ = nh->subscribe("/migration_point", 1, &SwarmControllerNode::migrationPointCb, this);
    global_position_sub_ = nh->subscribe("mavros/global_position/global", 1, &SwarmControllerNode::globalPositionCb, this);
    odom_sub_ = nh->subscribe("mavros/local_position/odom", 1, &SwarmControllerNode::odomCb, this);
    enable_control_sub_ = nh->subscribe("/enable_control", 1, &SwarmControllerNode::enableControlCb, this);
    uav_positions_sub_ = nh->subscribe("/uav_positions", 1, &SwarmControllerNode::uavPositionsCb, this);
    uav_position_pub_ = nh->advertise<swarm_control::UavPosition>("uav_position", 10);
    cmd_vel_pub_ = nh->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);


    v1_pub_ = nh->advertise<geometry_msgs::Point32>("v1", 10);
    v2_pub_ = nh->advertise<geometry_msgs::Point32>("v2", 10);
    v3_pub_ = nh->advertise<geometry_msgs::Point32>("v3", 10);
    v4_pub_ = nh->advertise<geometry_msgs::Point32>("v4", 10);
    vRes_pub_ = nh->advertise<geometry_msgs::Point32>("vRes", 10);
}

SwarmControllerNode::~SwarmControllerNode()
{
    mavros_state_sub_.shutdown();
    migration_point_sub_.shutdown();
    global_position_sub_.shutdown();
    odom_sub_.shutdown();
    enable_control_sub_.shutdown();
    uav_positions_sub_.shutdown();
    uav_position_pub_.shutdown();
    cmd_vel_pub_.shutdown();

    v1_pub_.shutdown();
    v2_pub_.shutdown();
    v3_pub_.shutdown();
    v4_pub_.shutdown();
    vRes_pub_.shutdown();
}

void SwarmControllerNode::controlLoop()
{
    // Print information on screen
    int chosenUav = 2;
    if ( id_ == chosenUav )
    {
        char tab2[1024];
        strncpy(tab2, mode_.c_str(), sizeof(tab2));
        tab2[sizeof(tab2) - 1] = 0;
        //ROS_INFO("UAV %d: \n Mode = %s | enableControl = %d \n Position = (%f, %f) | Objective = (%f , %f)",
          //       id_, tab2, enableControl_, position_.x, position_.y, migrationPoint_.x, migrationPoint_.y);
    }

    // Calculate each rule's influence
    geometry_msgs::Point32 v1 = rule1();
    geometry_msgs::Point32 v2 = rule2();
    geometry_msgs::Point32 v3 = rule3();
    geometry_msgs::Point32 v4 = rule4();


    if ( id_ == chosenUav )
    {
        //ROS_INFO("v1 = (%f, %f, %f)", v1.x, v1.y, v1.z);
        //ROS_INFO("v2 = (%f, %f, %f)", v2.x, v2.y, v2.z);
        //ROS_INFO("v3 = (%f, %f, %f)", v3.x, v3.y, v3.z);
        //ROS_INFO("v4 = (%f, %f, %f)", v4.x, v4.y, v4.z);
    }


    // Combine the rules
    vRes_.x = v1.x + v2.x + v3.x + v4.x;
    vRes_.y = v1.y + v2.y + v3.y + v4.y;
    vRes_.z = v1.z + v2.z + v3.z + v4.z;


    // Limit vRes
    double norm = sqrt( pow( (vRes_.x), 2 ) +
                     pow( (vRes_.y), 2 ) +
                     pow( (vRes_.z), 2 ) );
    if ( norm >= 5 )
    {
        vRes_.x *= (5 / norm);
        vRes_.y *= (5 / norm);
        vRes_.z *= (5 / norm);
    }


    if ( id_ == 0 )
    {
        //ROS_INFO("vRes = (%f, %f, %f)", vRes.x, vRes.y, vRes.z);
    }


    // Publish velocity for diagnostics purpose
    v1_pub_.publish(v1);
    v2_pub_.publish(v2);
    v3_pub_.publish(v3);
    v4_pub_.publish(v4);
    vRes_pub_.publish(vRes_);




    // Limit the Velocity in x
    if (vRes_.x > MAXVEL)
    {
        vRes_.x = MAXVEL;
    } else if (vRes_.x < MINVEL)
    {
        vRes_.x = MINVEL;
    }

    // Limit the Velocity in y
    if (vRes_.y > MAXVEL)
    {
        vRes_.y = MAXVEL;
    } else if (vRes_.y < MINVEL)
    {
        vRes_.y = MINVEL;
    }

    // PUBLISH VELOCITY
    if ( enableControl_ == true )
    {
        publishVelocity(vRes_.x, vRes_.y);
    }

    publishPose( id_, position_.x, position_.y, position_.z, vRes_.x, vRes_.y, vRes_.z );


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

void SwarmControllerNode::publishPose ( int id, double x, double y, double z, double vX, double vY, double vZ )
{
    swarm_control::UavPosition msg;

    msg.id = id;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.velocity.x = vX;
    msg.velocity.y = vY;
    msg.velocity.z = vZ;

    uav_position_pub_.publish(msg);
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

void SwarmControllerNode::globalPositionCb( const sensor_msgs::NavSatFix &msg )
{
    // Quando a primeira mensagem de GPS chegar, calcular dx_ e dy_
    if ( !initialDeltasCalculated_ )
    {
        double originLat, originLon;
        ros::param::get("swarm_controller_node/origin_lat", originLat);
        ros::param::get("swarm_controller_node/origin_lon", originLon);
        double lat = msg.latitude;
        double lon = msg.longitude;
        dx_ = haversines( originLat, originLon, originLat, lon );
        dy_ = haversines( originLat, originLon, lat, originLon );
        if ( (lon - originLon) < 0 ) dx_ *= -1;
        if ( (lat - originLat) < 0 ) dy_ *= -1;
        initialDeltasCalculated_ = true;
    }
}

void SwarmControllerNode::odomCb( const nav_msgs::OdometryConstPtr &msg )
{
    position_.x = msg->pose.pose.position.x + dx_;
    position_.y = msg->pose.pose.position.y + dy_;
    position_.z = msg->pose.pose.position.z;
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
        pos.velocity.x = msg->positions[i].velocity.x;
        pos.velocity.y = msg->positions[i].velocity.y;
        pos.velocity.y = msg->positions[i].velocity.z;
        neighbors_.positions.push_back(pos);
    }
}


// REYNOLDS RULES

// Rule 1: Flocking
geometry_msgs::Point32 SwarmControllerNode::rule1()
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

    return v1;
}


// Rule 2: Collision Avoidance
geometry_msgs::Point32 SwarmControllerNode::rule2()
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

                    if ( vm < 0.01 ) vm = 0.1;

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

    return v2;
}

// Rule 3: Velocity Matching
geometry_msgs::Point32 SwarmControllerNode::rule3()
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
                v3.x += neighbors_.positions[i].velocity.x;
                v3.y += neighbors_.positions[i].velocity.y;
                v3.z += neighbors_.positions[i].velocity.z;
            }
        }

        v3.x *= ( 1 / n );
        v3.y *= ( 1 / n );
        v3.z *= ( 1 / n );

        v3.x = v3.x - vRes_.x;
        v3.y = v3.y - vRes_.y;
        v3.z = v3.z - vRes_.z;
    }

    v3.x *= r3_;
    v3.y *= r3_;
    v3.z *= r3_;

    return v3;
}


// Rule 4: Migration
geometry_msgs::Point32 SwarmControllerNode::rule4()
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

    return v4;
}


// Returns the distance in meters between 2 points given their GPS coordinates
double SwarmControllerNode::haversines( double lat1, double lon1, double lat2, double lon2 )
{
    double a, c, d, dLat, dLon;
    int r = 6371000; // raio médio da Terra em metros
    // converter os ângulos para radianos:
    double degToRad = PI / 180.0;
    lat1 *= degToRad;
    lat2 *= degToRad;
    lon1 *= degToRad;
    lon2 *= degToRad;
    dLat = lat2 - lat1;
    dLon = lon2 - lon1;

    // fórmula de haversines:
    a = sin( dLat / 2 ) * sin( dLat / 2 ) +
               cos( lat1 ) * cos( lat2 ) *
               sin( dLon / 2 ) * sin( dLon / 2 );
    c = 2 * atan2( sqrt( a ), sqrt( 1 - a ) );
    d = r * c;

    return d;
}
