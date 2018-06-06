#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// include PCL(point cloud library)
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
float distance = 0;
//declare point cloud pointer
PointCloudXYZ::Ptr cloud_XYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_output;

//declare publisher
ros::Publisher pub_XYZRGB;

//declare global variable
bool lock = false;
void pointcloud_processing(void);

//call back function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;
    
    pcl::fromROSMsg (*input, *cloud_XYZ); //covert from ros type to pcl type
    copyPointCloud(*cloud_XYZ, *cloud_XYZRGB); //it will include frame information

    for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
    {
      if (cloud_XYZRGB->points[i].y >0 && cloud_XYZRGB->points[i].x>0 && cloud_XYZRGB->points[i].z>0)
      {
        cloud_XYZRGB->points[i].r = 255;
        cloud_XYZRGB->points[i].g = 0;
        cloud_XYZRGB->points[i].b = 0;
      }

      if (cloud_XYZRGB->points[i].y <0 && cloud_XYZRGB->points[i].x>0 && cloud_XYZRGB->points[i].z>0)
      {
        cloud_XYZRGB->points[i].r = 0;
        cloud_XYZRGB->points[i].g = 153;
        cloud_XYZRGB->points[i].b = 0;
      }
      if (cloud_XYZRGB->points[i].y >0 && cloud_XYZRGB->points[i].x<0 && cloud_XYZRGB->points[i].z>0)
      {
        cloud_XYZRGB->points[i].r = 0;
        cloud_XYZRGB->points[i].g = 204;
        cloud_XYZRGB->points[i].b = 204;
      }
      if (cloud_XYZRGB->points[i].y >0 && cloud_XYZRGB->points[i].x>0 && cloud_XYZRGB->points[i].z<0)
      {
        cloud_XYZRGB->points[i].r = 255;
        cloud_XYZRGB->points[i].g = 51;
        cloud_XYZRGB->points[i].b = 153;
      }
      if (cloud_XYZRGB->points[i].y <0 && cloud_XYZRGB->points[i].x<0 && cloud_XYZRGB->points[i].z>0)
      {
        cloud_XYZRGB->points[i].r = 255;
        cloud_XYZRGB->points[i].g = 153;
        cloud_XYZRGB->points[i].b = 51;
      }
      if (cloud_XYZRGB->points[i].y <0 && cloud_XYZRGB->points[i].x>0 && cloud_XYZRGB->points[i].z<0)
      {
        cloud_XYZRGB->points[i].r = 178;
        cloud_XYZRGB->points[i].g = 102;
        cloud_XYZRGB->points[i].b = 255;
      }
      if (cloud_XYZRGB->points[i].y >0 && cloud_XYZRGB->points[i].x<0 && cloud_XYZRGB->points[i].z<0)
      {
        cloud_XYZRGB->points[i].r = 102;
        cloud_XYZRGB->points[i].g = 102;
        cloud_XYZRGB->points[i].b = 255;
      }
      if (cloud_XYZRGB->points[i].y <0 && cloud_XYZRGB->points[i].x<0 && cloud_XYZRGB->points[i].z<0)
      {
        cloud_XYZRGB->points[i].r = 255;
        cloud_XYZRGB->points[i].g = 255;
        cloud_XYZRGB->points[i].b = 255;
      }
      }

    pointcloud_processing();

  }
  else
  {
    std::cout << "lock" << std::endl;
  }
}
void getdistance()
{
  for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
  {
    float X = cloud_XYZRGB->points[i].x;
    float Y = cloud_XYZRGB->points[i].y;
    float Z = cloud_XYZRGB->points[i].z;
    distance = sqrt(X*X + Y*Y);
    if (distance < 1)
    {
      cloud_XYZRGB->points[i].r = 255;
      cloud_XYZRGB->points[i].g = 0;
      cloud_XYZRGB->points[i].b = 0;
    }
  }
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void pointcloud_processing()
{
  getdistance();
  ROS_INFO("Success output");
//  for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
//  {
//    distance =
//  }
  pub_XYZRGB.publish(*cloud_XYZRGB);

  lock = false;
}

int main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "ros_pcl");
     ros::NodeHandle nh;

     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);

     // Create a ROS publisher for the output point cloud
     pub_XYZRGB = nh.advertise<PointCloudXYZRGB> ("/output_pcl", 1);

     // Spin
     ros::spin ();
}
