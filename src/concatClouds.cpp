#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

tf::TransformListener * tfListener;
ros::Publisher pub_points_;

void concatenateClouds(std::vector<sensor_msgs::PointCloud2::ConstPtr> &msg_vector, sensor_msgs::PointCloud2 &points_msg2) {
  pcl::PointCloud<pcl::PointXYZI> cloud, curr_cloud, transformed_cloud;
  tf::StampedTransform curr_transform;

  const auto &msg_0 = *(msg_vector.at(0));
  
  pcl::fromROSMsg (msg_0, cloud); // Transform the first cloud. We'll append the rest here
  
  for (size_t i = 1; i < msg_vector.size(); i++) {  // Transform the rest of clouds and append to the out cloud
    auto &msg_i  = *(msg_vector.at(i));
    pcl::fromROSMsg (msg_i, curr_cloud);
    try{
      tfListener->lookupTransform(msg_0.header.frame_id, msg_i.header.frame_id, ros::Time(0), curr_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
  
    pcl_ros::transformPointCloud(curr_cloud, transformed_cloud,tf::Transform(curr_transform));    
    cloud = cloud + transformed_cloud;
  }
  pcl::toROSMsg(cloud, points_msg2);

  points_msg2.header.stamp = msg_0.header.stamp;
  points_msg2.header.frame_id = msg_0.header.frame_id;
}

void callback2(const sensor_msgs::PointCloud2::ConstPtr& msg_1, const sensor_msgs::PointCloud2::ConstPtr& msg_2)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  concatenateClouds(msg_vector, points_msg2);

  //Send the result to other ROS nodes
  pub_points_.publish(points_msg2);
}

void callback3(const sensor_msgs::PointCloud2::ConstPtr& msg_1, const sensor_msgs::PointCloud2::ConstPtr& msg_2, const sensor_msgs::PointCloud2::ConstPtr& msg_3)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

void callback4(const sensor_msgs::PointCloud2::ConstPtr& msg_1, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_2, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_3,
               const sensor_msgs::PointCloud2::ConstPtr& msg_4)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  msg_vector.push_back(msg_4);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

void callback5(const sensor_msgs::PointCloud2::ConstPtr& msg_1, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_2, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_3,
               const sensor_msgs::PointCloud2::ConstPtr& msg_4,
               const sensor_msgs::PointCloud2::ConstPtr& msg_5)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  msg_vector.push_back(msg_4);
  msg_vector.push_back(msg_5);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

void callback6(const sensor_msgs::PointCloud2::ConstPtr& msg_1, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_2, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_3,
               const sensor_msgs::PointCloud2::ConstPtr& msg_4,
               const sensor_msgs::PointCloud2::ConstPtr& msg_5,
               const sensor_msgs::PointCloud2::ConstPtr& msg_6)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  msg_vector.push_back(msg_4);
  msg_vector.push_back(msg_5);
  msg_vector.push_back(msg_6);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

void callback7(const sensor_msgs::PointCloud2::ConstPtr& msg_1, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_2, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_3,
               const sensor_msgs::PointCloud2::ConstPtr& msg_4,
               const sensor_msgs::PointCloud2::ConstPtr& msg_5,
               const sensor_msgs::PointCloud2::ConstPtr& msg_6,
               const sensor_msgs::PointCloud2::ConstPtr& msg_7)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  msg_vector.push_back(msg_4);
  msg_vector.push_back(msg_5);
  msg_vector.push_back(msg_6);
  msg_vector.push_back(msg_7);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

void callback8(const sensor_msgs::PointCloud2::ConstPtr& msg_1, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_2, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_3,
               const sensor_msgs::PointCloud2::ConstPtr& msg_4,
               const sensor_msgs::PointCloud2::ConstPtr& msg_5,
               const sensor_msgs::PointCloud2::ConstPtr& msg_6,
               const sensor_msgs::PointCloud2::ConstPtr& msg_7,
               const sensor_msgs::PointCloud2::ConstPtr& msg_8)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  msg_vector.push_back(msg_4);
  msg_vector.push_back(msg_5);
  msg_vector.push_back(msg_6);
  msg_vector.push_back(msg_7);
  msg_vector.push_back(msg_8);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

void callback9(const sensor_msgs::PointCloud2::ConstPtr& msg_1, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_2, 
               const sensor_msgs::PointCloud2::ConstPtr& msg_3,
               const sensor_msgs::PointCloud2::ConstPtr& msg_4,
               const sensor_msgs::PointCloud2::ConstPtr& msg_5,
               const sensor_msgs::PointCloud2::ConstPtr& msg_6,
               const sensor_msgs::PointCloud2::ConstPtr& msg_7,
               const sensor_msgs::PointCloud2::ConstPtr& msg_8,
               const sensor_msgs::PointCloud2::ConstPtr& msg_9)
{
  sensor_msgs::PointCloud2 points_msg2;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> msg_vector;
  msg_vector.push_back(msg_1);
  msg_vector.push_back(msg_2);
  msg_vector.push_back(msg_3);
  msg_vector.push_back(msg_4);
  msg_vector.push_back(msg_5);
  msg_vector.push_back(msg_6);
  msg_vector.push_back(msg_7);
  msg_vector.push_back(msg_8);
  msg_vector.push_back(msg_9);
  concatenateClouds(msg_vector, points_msg2);
  pub_points_.publish(points_msg2);
}

int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "concat_pcl");

  ros::NodeHandle nh, pnh("~");

  tfListener = new tf::TransformListener();

  
  int synch_margin_queue = 10;
  nh.getParam("sychronization_margin", synch_margin_queue);

  int n_clouds = 4;
  pnh.getParam("n_clouds", n_clouds);

  if (n_clouds < 2 || n_clouds > 9) {
    ROS_FATAL("The number of clouds should be between 2 and 9");
    exit(-1);
  }

  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("concatenatedCloud",  1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_1(nh, "/pc_1", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_2(nh, "/pc_2", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_3(nh, "/pc_3", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_4(nh, "/pc_4", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_5(nh, "/pc_5", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_6(nh, "/pc_6", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_7(nh, "/pc_7", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_8(nh, "/pc_8", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_9(nh, "/pc_9", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy2; 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy3;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2> MySyncPolicy4;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy5;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy6;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy7;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy8;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy9;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy2> *sync_2 = NULL;
  message_filters::Synchronizer<MySyncPolicy3> *sync_3 = NULL;
  message_filters::Synchronizer<MySyncPolicy4> *sync_4 = NULL;
  message_filters::Synchronizer<MySyncPolicy5> *sync_5 = NULL;
  message_filters::Synchronizer<MySyncPolicy6> *sync_6 = NULL;
  message_filters::Synchronizer<MySyncPolicy7> *sync_7 = NULL;
  message_filters::Synchronizer<MySyncPolicy8> *sync_8 = NULL;
  message_filters::Synchronizer<MySyncPolicy9> *sync_9 = NULL;
  switch(n_clouds) {
    case 2:
      sync_2 = new message_filters::Synchronizer<MySyncPolicy2>(MySyncPolicy2(synch_margin_queue), pc_sub_1, pc_sub_2);
      sync_2->registerCallback(boost::bind(&callback2, _1, _2));
      break;
    case 3:
      sync_3 = new message_filters::Synchronizer<MySyncPolicy3>(MySyncPolicy3(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3);
      sync_3->registerCallback(boost::bind(&callback3, _1, _2, _3));
      break;
    case 4:
      sync_4 = new message_filters::Synchronizer<MySyncPolicy4>(MySyncPolicy4(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3, pc_sub_4);
      sync_4->registerCallback(boost::bind(&callback4, _1, _2, _3, _4));
      break;
    case 5:
      sync_5 = new message_filters::Synchronizer<MySyncPolicy5>(MySyncPolicy5(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3, pc_sub_4, pc_sub_5);
      sync_5->registerCallback(boost::bind(&callback5, _1, _2, _3, _4, _5));
      break;
    case 6:
      sync_6 = new message_filters::Synchronizer<MySyncPolicy6>(MySyncPolicy6(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3, pc_sub_4, pc_sub_5, pc_sub_6);
      sync_6->registerCallback(boost::bind(&callback6, _1, _2, _3, _4, _5, _6));
      break;
    case 7:
      sync_7 = new message_filters::Synchronizer<MySyncPolicy7>(MySyncPolicy7(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3,
                                                             pc_sub_4, pc_sub_5, pc_sub_6, pc_sub_7);
      sync_7->registerCallback(boost::bind(&callback7, _1, _2, _3, _4, _5, _6, _7));
      break;
    case 8:
      sync_8 = new message_filters::Synchronizer<MySyncPolicy8>(MySyncPolicy8(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3,
                                                             pc_sub_4, pc_sub_5, pc_sub_6, pc_sub_7, pc_sub_8);
      sync_8->registerCallback(boost::bind(&callback8, _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    case 9:
      sync_9 = new message_filters::Synchronizer<MySyncPolicy9>(MySyncPolicy9(synch_margin_queue), pc_sub_1, pc_sub_2, pc_sub_3,
                                                             pc_sub_4, pc_sub_5, pc_sub_6, pc_sub_7, pc_sub_8, pc_sub_9);
      sync_9->registerCallback(boost::bind(&callback9, _1, _2, _3, _4, _5, _6, _7, _8, _9));
      break;
  }
  ROS_INFO("Cloud Concatenator --> Expecting %d clouds", n_clouds);
  ros::spin();

  delete sync_2, sync_3, sync_4, sync_5, sync_6, sync_7, sync_8, sync_9;
  delete tfListener;
}
