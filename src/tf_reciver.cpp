/*
 * tf_sub.cpp
 *
 *  Created on: Feb 23, 2020
 *      Author: light
*       ori_Author: Hannes Keller
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <nav_msgs/Odometry.h>//里程计信息格式
#include <std_msgs/Float32.h>

#define PI 3.14159265

static ros::Publisher predict_yaw_pub;
static std_msgs::Float32 predict_yaw_msg;

class TfRemapper
{
 public:
  TfRemapper(ros::NodeHandle nh);
  ~TfRemapper() {}

 private:
  void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber tfSub_;

};

TfRemapper::TfRemapper(ros::NodeHandle nh): nh_(nh)
{
  tfSub_ = nh_.subscribe("/tf", 1000, &TfRemapper::tfCallback, this);
  predict_yaw_pub = nh.advertise<std_msgs::Float32>("/predict_yaw", 100);
}

void TfRemapper::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  double roll, pitch, yaw;//定义存储r\p\y的容器
  //x = msg->transforms[0].transform.translation.x;
  //y = msg->transforms[0].transform.translation.y;
  //z = msg->transforms[0].transform.translation.z;
  yaw = msg->transforms[0].transform.rotation.z;

  if((msg->transforms[0].header.frame_id.size())>3) //排除map -> odom的值 map -> odom=3 / odom -> base_footprint=4
  {
      tf::Quaternion quat;
      tf::quaternionMsgToTF(msg->transforms[0].transform.rotation, quat); //解四元素
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
      yaw = yaw*180/PI; //弧度转角度 yaw 范围 -180 到180
      if(yaw < 0)
      {
      	yaw = yaw+360; //把yaw角度范围改成0-360
      }
      ROS_INFO("yaw = %f",yaw);
      predict_yaw_msg.data = yaw;
      predict_yaw_pub.publish(predict_yaw_msg);
  }  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_reciver");

  ros::NodeHandle nh;

  TfRemapper tfRemapper(nh);

  ros::spin();

  return 0;
}


