#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_pcl_out;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;

std::unique_ptr<Eigen::Affine3d> sensor2novatel;
std::unique_ptr<Eigen::Affine3d> novatel2world;
std::unique_ptr<Eigen::Affine3d> sensor2world;

bool b_tf = false;

void LivoxMsgCbk1(const livox_ros_driver2::CustomMsgConstPtr& livox_msg_in) {

  if((sensor2novatel == nullptr)&&(novatel2world == nullptr)&&(sensor2world == nullptr)&&(b_tf == false))
  {
      return;
  }

  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> handle_cloud;
  pcl::PointCloud<PointType> out_cloud; 
  //新加
    Eigen::Affine3d z_rotation = Eigen::Affine3d::Identity();
    z_rotation.rotate(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ()));  // 顺时针90度
    //
  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      //新加
      Eigen::Vector3d original_point(livox_msg->points[i].x, livox_msg->points[i].y, livox_msg->points[i].z);
      Eigen::Vector3d rotated_point = z_rotation * original_point;
      //
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      float s = livox_msg->points[i].offset_time / (float)time_end;
      pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      handle_cloud.push_back(pt);
    }
  }
    //ROS_INFO("in__cloud points size: %ld",handle_cloud.points.size());
    
    if(handle_cloud.points.size() == 0)
    {
        return;
    }

    for (size_t i = 0; i < handle_cloud.points.size(); ++i) {
        //if(handle_cloud.points[i].x > 50) continue;
        //if(handle_cloud.points[i].y > 50) continue;
        Eigen::Vector3d trans_point(-handle_cloud.points[i].y, handle_cloud.points[i].x, handle_cloud.points[i].z);
        trans_point = (*sensor2novatel) * trans_point;
        PointType new_point;
        
        new_point.x = trans_point(0);
        new_point.y = trans_point(1);
        new_point.z = trans_point(2);
        new_point.intensity = handle_cloud.points[i].intensity;
        // new_point.intensity = 1;
        out_cloud.points.push_back(new_point);
    }
    ROS_INFO("out_cloud points size: %ld",out_cloud.points.size());
    if(out_cloud.points.size() == 0)
    {
        return;
    }
    
    sensor_msgs::PointCloud2 output_points;
    pcl::toROSMsg(out_cloud,output_points);
    output_points.header.frame_id = "livox_frame";
    output_points.header.stamp = ros::Time::now();
    pub_pcl_out.publish(output_points);
    livox_data.clear();
    b_tf = false;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
        try {
        // 从IMU获取姿态
        Eigen::Quaterniond imu_orientation(
            imu_msg->orientation.w,
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z
        );
        
        // 从参数服务器读取外参(或使用默认值)
        Eigen::Vector3d lidar_to_vehicle_translation( -0.011, -0.02329, 0.04412 ); // x,y,z -0.011, -0.02329, 0.04412 
        Eigen::Quaterniond lidar_to_vehicle_rotation = Eigen::Quaterniond::Identity();
        
        // 更新变换矩阵
        sensor2novatel.reset(new Eigen::Affine3d);
        *sensor2novatel = Eigen::Translation3d(lidar_to_vehicle_translation) * lidar_to_vehicle_rotation;
        
        novatel2world.reset(new Eigen::Affine3d);
        *novatel2world = Eigen::Translation3d(0,0,0) * imu_orientation;
        
        sensor2world.reset(new Eigen::Affine3d);
        *sensor2world = *novatel2world * (*sensor2novatel);
        
        b_tf = true;
    } catch (...) {
        ROS_WARN("Failed to process IMU message");
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub_node");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");
    
  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_pc2", 10);

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver2::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
    
   ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 10, imuCallback);
    
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
}
