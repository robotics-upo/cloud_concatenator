#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

class AsyncCloudConcatenator
{

public:
    AsyncCloudConcatenator() : spinner_(4)
    {

        cloud_1_sub_ = pnh_.subscribe<sensor_msgs::PointCloud2>("/cloud1", 1, &AsyncCloudConcatenator::callback1, this);
        cloud_2_sub_ = pnh_.subscribe<sensor_msgs::PointCloud2>("/cloud2", 1, &AsyncCloudConcatenator::callback2, this);
        cloud_3_sub_ = pnh_.subscribe<sensor_msgs::PointCloud2>("/cloud3", 1, &AsyncCloudConcatenator::callback3, this);
        cloud_4_sub_ = pnh_.subscribe<sensor_msgs::PointCloud2>("/cloud4", 1, &AsyncCloudConcatenator::callback4, this);
        double freq;
        pnh_.param("frequency", freq, 10.0);
        time_interval_ = 1.0/freq;
        timer_ = pnh_.createTimer(ros::Duration(time_interval_), &AsyncCloudConcatenator::timerCallback, this);
        full_cloud_pub_ = pnh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/full_cloud", 1);

        pnh_.param("target_frame", target_frame_, (std::string)"base_link");

    }
    void run()
    {
        spinner_.start();
        ros::waitForShutdown();
    }
 
private:
    void timerCallback(const ros::TimerEvent& event){


        final_cloud_.points.clear();
        ROS_INFO("Number of clouds: [%ld, %ld, %ld, %ld]", cloud_1_vec_.size(),cloud_2_vec_.size(),cloud_3_vec_.size(),cloud_4_vec_.size() );
        insertPoints(cloud_1_vec_, final_cloud_);
        insertPoints(cloud_2_vec_, final_cloud_);
        insertPoints(cloud_3_vec_, final_cloud_);
        insertPoints(cloud_4_vec_, final_cloud_);
        ROS_INFO("Final number of points: [%ld]",final_cloud_.points.size() );
        pcl_conversions::toPCL(ros::Time::now(), final_cloud_.header.stamp);
        final_cloud_.header.frame_id = target_frame_;
        full_cloud_pub_.publish(final_cloud_);
        
    }
    void insertPoints(std::vector<pcl::PointCloud<pcl::PointXYZI>> &in_cloud, pcl::PointCloud<pcl::PointXYZI> &result_cloud){
        
        if(in_cloud.size() == 0)
            return;

        for(auto &it: in_cloud){
            for(auto &point_it: it){
                // std::cout << "Point" << point_it.x << point_it.y << std::endl;
                result_cloud.points.push_back(point_it);
            }
        }
        // in_cloud.clear();
    }

    void callback1(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // ROS_INFO("Callback 1");
        sensor_msgs::PointCloud2 msg_target_frame;
        pcl_ros::transformPointCloud(target_frame_, *msg, msg_target_frame, tf_listener_);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(msg_target_frame, cloud);
        cloud_1_vec_.push_back(cloud);
         
         for(int i = 0; i < cloud_1_vec_.size(); ++i){
            if(cloud_1_vec_[i].header.stamp - final_cloud_.header.stamp > 1e6 * time_interval_)
                cloud_1_vec_.erase(cloud_1_vec_.begin()+i);
        }
    }
    void callback2(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // ROS_INFO("Callback 2");

        sensor_msgs::PointCloud2 msg_target_frame;
        pcl_ros::transformPointCloud(target_frame_, *msg, msg_target_frame, tf_listener_);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(msg_target_frame, cloud);
        cloud_2_vec_.push_back(cloud);
        for(int i = 0; i < cloud_2_vec_.size(); ++i){
            if(cloud_2_vec_[i].header.stamp - final_cloud_.header.stamp > 1e6 * time_interval_)
                cloud_2_vec_.erase(cloud_2_vec_.begin()+i);
        }
    }
    void callback3(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // ROS_INFO("Callback 3");
        sensor_msgs::PointCloud2 msg_target_frame;
        pcl_ros::transformPointCloud(target_frame_, *msg, msg_target_frame, tf_listener_);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(msg_target_frame, cloud);
        cloud_3_vec_.push_back(cloud);

         for(int i = 0; i < cloud_3_vec_.size(); ++i){
            if(cloud_3_vec_[i].header.stamp - final_cloud_.header.stamp > 1e6 * time_interval_)
                cloud_3_vec_.erase(cloud_3_vec_.begin()+i);
        }
    }
    void callback4(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // ROS_INFO("Callback 4");
        
        sensor_msgs::PointCloud2 msg_target_frame;
        pcl_ros::transformPointCloud(target_frame_, *msg, msg_target_frame, tf_listener_);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(msg_target_frame, cloud);
        cloud_4_vec_.push_back(cloud);

        for(int i = 0; i < cloud_4_vec_.size(); ++i){
            if(cloud_4_vec_[i].header.stamp - final_cloud_.header.stamp > 1e6 * time_interval_)
                cloud_4_vec_.erase(cloud_4_vec_.begin()+i);
        }

    }

    ros::NodeHandle pnh_{"~"};
    tf::TransformListener tf_listener_;
    ros::Timer timer_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber cloud_1_sub_, cloud_2_sub_, cloud_3_sub_, cloud_4_sub_;
    ros::Publisher full_cloud_pub_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_1_vec_, cloud_2_vec_, cloud_3_vec_, cloud_4_vec_;
    pcl::PointCloud<pcl::PointXYZI> final_cloud_;

    double time_interval_;
    std::string target_frame_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "concat_pcl");
    AsyncCloudConcatenator concat;
    concat.run();
    return 0;
}