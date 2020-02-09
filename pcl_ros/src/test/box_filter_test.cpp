#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/crop_box.h>

ros::Publisher pub;

 void
 cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
 {

 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Do data processing here...
 pcl_conversions::toPCL(*cloud_msg, *cloud);

 pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
 //boxFilter.setNegative(true);
 boxFilter.setInputCloud(cloudPtr);
 boxFilter.setMin(Eigen::Vector4f(-2.0,-2.0, 0.0, 0));
 boxFilter.setMax(Eigen::Vector4f(2.0, 2.0, 1.0, 0));
 boxFilter.setTranslation(Eigen::Vector3f(0.0, 0.0, 0.0));
 boxFilter.filter(cloud_filtered);

 sensor_msgs::PointCloud2 output;
 pcl_conversions::fromPCL(cloud_filtered, output);
 output.header = cloud_msg->header;
/*
 for (sensor_msgs::PointCloud2ConstIterator<float> it(output, "x"); it != it.end(); ++it) {
     // TODO: do something with the values of x, y, z

     std::cout <<  it[0] << "/ ";
     std::cout <<  it[1]<< "/ ";
     std::cout <<  it[2]<< "/ ";
     std::cout << std::endl;
 }
*/
 // Publish the data.
 pub.publish (output);

 }

 int main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "CROP");
ros::NodeHandle nh;

// Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("/T01/os1_cloud_node/points", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("/Cloud_Cropped", 1);

// Spin
ros::spin ();
}
