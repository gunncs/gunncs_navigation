/**
 * 
 *
 *
 *
 *
 *
 */
// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Rviz related
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <iterator>

using namespace std;

//Typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

CloudT cloudFull;

ros::Publisher rotatedCloudPub; 

void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    static tf::TransformBroadcaster br;

    tf::Transform transform;

    // Precomputed for efficiency
    double theta = -0.3f;
    const float cosTheta = cos(theta);
    const float sinTheta = sin(theta);


    CloudT rotatedCloud;

    pcl::fromROSMsg(*cloud, cloudFull);


    for (CloudT::const_iterator it = cloudFull.begin(); it != cloudFull.end(); ++it)
    {
        // Rotate about the x-axis to align floor with xz-plane
        float x = it->x;
        float y = (it->y)*cosTheta - (it->z)*sinTheta;
        float z = (it->y)*sinTheta + (it->z)*cosTheta;

        PointT point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.rgb = it->rgb;

        rotatedCloud.push_back(point);
    }


    sensor_msgs::PointCloud2 rotatedCloudMsg;

    pcl::toROSMsg(rotatedCloud, rotatedCloudMsg);

    transform.setOrigin(tf::Vector3(10.0,0.0,0.0));

    tf::StampedTransform trans =
        tf::StampedTransform(transform, ros::Time::now(),
                "/base_footprint",
                "/openni_camera");

    br.sendTransform(trans);

    rotatedCloudMsg.header.frame_id = "/camera_rgb_optical_frame";
    rotatedCloudMsg.header.stamp = ros::Time::now();

    rotatedCloudPub.publish(rotatedCloudMsg);


}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pclGroundDetect");
    ros::NodeHandle nh;
    ros::Subscriber kinectSub = nh.subscribe("/camera/rgb/points", 1, CloudCallback);
    rotatedCloudPub = nh.advertise<sensor_msgs::PointCloud2>("rotated_cloud", 1);
    ros::spin();

    return 0;
}

