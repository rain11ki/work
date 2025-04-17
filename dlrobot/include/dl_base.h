#ifndef RIKI_BASE_H
#define RIKI_BASE_H

#include <ros/ros.h>
#include <dl_msgs/Velocities.h>
#include <tf/transform_broadcaster.h>

class DlBase
{
public:
    DlBase();
    void velCallback(const dl_msgs::Velocities& vel);
private:
    ros::NodeHandle nh_;
    ros::Publisher  odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    tf::TransformBroadcaster odom_broadcaster_;

    float linear_scale_;
    float steering_angle_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
	
};

#endif
