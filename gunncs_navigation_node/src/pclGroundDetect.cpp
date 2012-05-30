/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pointcloud_online_viewer.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

//Segmentation
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

//extraction
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

//convex hull
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"

//passthrough filter
#include "pcl/filters/passthrough.h"

//tf
#include <tf/transform_listener.h>


#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <string>

using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> CloudT;
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
boost::mutex m;

double getAngle(){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_link", "/camera_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    return 0;

}


/**
 * Flattens cloud to be parallel to XZ axis (red/blue axis)
 */
CloudT::Ptr flattenCloud(CloudT::Ptr original){

    CloudT::Ptr flattenedCloud (new CloudT());


    for( CloudT::const_iterator it = original->begin(); it != original->end(); ++it){

        Point point;
        point.x = it -> x;
        point.y = 0;
        point.z = it -> z;
        //point.rgb = it->rgb;

        flattenedCloud->push_back(point);


    }
    //CloudT::Ptr retu (new CloudT(rotatedCloud));
    return flattenedCloud;

}



/**
 * Rectifies cloud-- rotates it to 0 degrees
 */
CloudT::Ptr rectifyCloud(CloudT::Ptr original){
    double theta = -0.3f;
    const float cosTheta = cos(theta);
    const float sinTheta = sin(theta);

    CloudT::Ptr rotatedCloud (new CloudT());


    for( CloudT::const_iterator it = original->begin(); it != original->end(); ++it){
        // Rotate about the x-axis to align floor with xz-plane
        float x = it->x;
        float y = (it->y)*cosTheta - (it->z)*sinTheta;
        float z = (it->y)*sinTheta + (it->z)*cosTheta;
        //double y = it->y;
        //double z = it->z;

        Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        //point.rgb = it->rgb;

        rotatedCloud->push_back(point);


    }
    //CloudT::Ptr retu (new CloudT(rotatedCloud));
    return rotatedCloud;

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud) {
    ROS_INFO ("PointCloud with %d data points (%s), stamp %f,and frame %s.", 
            cloud->width * cloud->height, 
            pcl::getFieldsList (*cloud).c_str (), 
            cloud->header.stamp.toSec (), 
            cloud->header.frame_id.c_str ()); 
    m.lock ();
    cloud_ = cloud;
    m.unlock ();
}

/* ---[ */
int main (int argc, char** argv) {
    ros::init (argc, argv, "pointcloud_online_viewer");
    ros::NodeHandle nh;

    // Get the queue size from the command line
    int queue_size = 1;
    pcl::console::parse_argument (argc, argv, "-qsize", queue_size);
    pcl::console::print_highlight ("Using a queue size of %d\n", queue_size);

    // Get the number of clouds to keep on screen
    int nr_clouds = 1;
    pcl::console::parse_argument (argc, argv, "-nclouds", nr_clouds);

    // Create a ROS subscriber
    ros::Subscriber sub = nh.subscribe ("/cloud_throttled", queue_size, cloud_cb);

    pcl_visualization::PCLVisualizer vis_orig(argc, argv, "Original");
    ColorHandlerPtr color_handler;

    double psize = 0;
    while (nh.ok ()){
        // Spin
        ros::spinOnce ();
        ros::Duration (0.001).sleep ();
        vis_orig.spinOnce (10);

        // If no cloud received yet, 
        if (!cloud_){
            continue;
        }
        if (cloud_ == cloud_old_){
            continue;
        }

        // Save the last point size used
        //vis_orig.getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud_original");
        vis_orig.removePointCloud ("cloud_original");

        //vis_orig.getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "psegment");
        vis_orig.removePointCloud ("psegment");

        //vis_orig.getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "hull");
        vis_orig.removePointCloud ("hull");
        vis_orig.addCoordinateSystem();
        // Convert to PointCloud<T>
        m.lock ();

        /*****************
         * PROCESSING LOOP
         *****************
         */
        {

            CloudT cloud_raw;
            //(const sensor_msgs::PointCloud2 &msg, pcl::PointCloud< PointT > &cloud)
            pcl::fromROSMsg (*cloud_, cloud_raw);

            CloudT::Ptr cloud_original (new CloudT(cloud_raw));
            CloudT::Ptr cloud_filtered (new CloudT());
            CloudT::Ptr cloud_rotated(new CloudT());
            CloudT::Ptr cloud_psegment (new CloudT());
            CloudT::Ptr cloud_projected (new CloudT());
            CloudT::Ptr cloud_hull (new CloudT());

            /*
             * PASSTHROUGH FILTER-- remove NaNs
             */

            /*
               pcl::PassThrough<Point> pass;
               pass.setInputCloud (cloud_original);
               pass.setFilterFieldName ("z");
               pass.setFilterLimits(0, 1.1);
               pass.filter(*cloud_filtered);

*/
            ROS_INFO ("original: %d data points.", 
                    cloud_original->width * cloud_original->height);

            /*
             * DOWNSAMPLE WITH VOXEL FILTER
             */

            pcl::VoxelGrid<Point> sor;
            sor.setInputCloud (cloud_original);
            sor.setLeafSize (0.01, 0.01, 0.01);
            sor.filter (*cloud_filtered);
            ROS_INFO ("downsampled:: %d data points.", 
                    cloud_filtered->width * cloud_filtered->height);

            /*
             * ROTATE
             */
            cloud_rotated = rectifyCloud(cloud_filtered);


            /*
             * VISUALIZING FILTERED
             */

            pcl_visualization::PointCloudColorHandlerCustom<Point> white_color_handler 
                (*cloud_rotated, 255, 255, 255);
            vis_orig.addPointCloud (*cloud_rotated, white_color_handler, "cloud_original");

            // Set the point size
            vis_orig.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud_original");



            /*
             * PLANAR SEGMENTATION
             */

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
            // Create the segmentation object
            pcl::SACSegmentation<Point> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);

            seg.setInputCloud (cloud_rotated);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0){ 
                ROS_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }   

            /*
             * EXTRACTION
             */

            pcl::ExtractIndices<Point> extract;
            // Extract the inliers
            extract.setInputCloud (cloud_rotated);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_psegment);
            ROS_INFO ("planar segment: %d data points.", 
                    cloud_psegment->width * cloud_psegment->height);

            /*
               std::stringstream ss;
               ss << "table_scene_lms400_plane_" << i << ".pcd";
               writer.write<pcl::PointXYZ> (ss.str (), *cloud_psegment, false);
               */

            // Create the filtering object
            //extract.setNegative (true);
            //extract.filter (*cloudptr);
            //

            /*
             * GROUND PLANE FLATTENING
             */
            cloud_psegment = flattenCloud(cloud_psegment);


            /*
             * GROUND PLANE VISUALIZATION
             */ 

            pcl_visualization::PointCloudColorHandlerCustom<Point> psegment_color
                (*cloud_psegment, 0, 255, 0);

            vis_orig.addPointCloud (*cloud_psegment, psegment_color, "psegment");
            vis_orig.setPointCloudRenderingProperties 
                (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "psegment");


            /*
             * CONVEX HULL
             */

            // Create a Convex Hull representation of the projected inliers
            pcl::ConvexHull<Point> chull;
            chull.setInputCloud (cloud_psegment);
            chull.reconstruct (*cloud_hull);

            //ROS_INFO ("Convex hull has: %zu data points.", cloud_hull->points.size ());
            ROS_INFO ("convex hull: %d data points.", 
                    cloud_hull->width * cloud_hull->height);

            //downsample convex hull
            pcl::VoxelGrid<Point> hull_dsample;
            hull_dsample.setInputCloud (cloud_hull);
            hull_dsample.setLeafSize (0.4, 0.4, 0.4);
            hull_dsample.filter (*cloud_hull);
            ROS_INFO ("downsampled convex hull: %d data points.", 
                    cloud_hull->width * cloud_hull->height);



            /*
             * CONVEX HULL VISUALIZATION
             */
            pcl_visualization::PointCloudColorHandlerCustom<Point> convexHull_color 
                (*cloud_hull, 255, 0, 0);

            vis_orig.addPointCloud (*cloud_hull, convexHull_color, "hull");
            vis_orig.setPointCloudRenderingProperties 
                (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 5, "hull");


            /*
             * LINE DRAWING
             */



            cloud_old_ = cloud_;
        }
        m.unlock ();
    }

    return (0);
}
/* ]--- */
