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

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"


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
    CloudT cloud_xyz;
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
        vis_orig.getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud_original");

        vis_orig.removePointCloud ("cloud_original");

        vis_orig.getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "filtered");

        vis_orig.removePointCloud ("filtered");
        // Convert to PointCloud<T>
        m.lock ();

        /*****************
         * PROCESSING LOOP
         *****************
         */
        {
            //(const sensor_msgs::PointCloud2 &msg, pcl::PointCloud< PointT > &cloud)
            pcl::fromROSMsg (*cloud_, cloud_xyz);


            /*
             * VISUALIZING ORIGINAL
             */

            
            pcl_visualization::PointCloudColorHandlerCustom<Point> white_color_handler 
                (cloud_xyz, 255, 255, 255);
            vis_orig.addPointCloud (cloud_xyz, white_color_handler, "cloud_original");

            // Set the point size
            vis_orig.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud_original");

            

            /*
             * PLANAR SEGMENTATION
             */

            pcl::ModelCoefficients coefficients;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.1);



            CloudT::Ptr cloudptr (new CloudT (cloud_xyz));
            seg.setInputCloud (cloudptr);
            seg.segment (*inliers, coefficients);

            if (inliers->indices.size () == 0){ 
                ROS_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }   

            /*
             * EXTRACTION
             */
            CloudT::Ptr cloud_p (new CloudT());

            pcl::ExtractIndices<Point> extract;
            // Extract the inliers
            extract.setInputCloud (cloudptr);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            ROS_INFO ("PointCloud representing the planar component: %d data points.", 
                    cloud_p->width * cloud_p->height);

            /*
            std::stringstream ss;
            ss << "table_scene_lms400_plane_" << i << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
            */

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloudptr);

            
            /*
             * GROUND PLANE VISUALIZATION
             */ 

            pcl_visualization::PointCloudColorHandlerCustom<Point> single_color
                (*cloud_p, 0, 255, 0);
            
            vis_orig.addPointCloud (*cloud_p, single_color, "filtered");
            vis_orig.setPointCloudRenderingProperties 
                (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "filtered");
            


            cloud_old_ = cloud_;
        }
        m.unlock ();
    }

    return (0);
}
/* ]--- */
