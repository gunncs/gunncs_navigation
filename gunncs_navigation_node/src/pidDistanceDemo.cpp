/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>

#include <math.h>

using namespace cv;
using namespace std;

cv::Mat_<float> getImage(const sensor_msgs::ImageConstPtr& msg);
double readDistance(cv::Mat depth, int x_pos, int y_pos); cv::Mat flattenRawImage(cv::Mat original); 
void loop(cv::Mat original); 
void addText(Mat image, string text, double data, int x, int y);


int x = 320;
int y = 240;

ros::Publisher distance_pub;


sensor_msgs::CvBridge img_bridge_;
cv::Mat orig;

void kinectCallBack(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat original = getImage(msg);
    orig = original;
    loop(original);

} 

void loop(cv::Mat original){

    cv::Mat floatImage = flattenRawImage(original);
    cv::cvtColor(floatImage, floatImage, CV_GRAY2BGR);

    cv::circle(floatImage, cv::Point(x,y), 3, cv::Scalar(0,255,0), -1);
    double dist = readDistance(original, x, y);
    addText(floatImage, "", dist, x, y);

    
    std_msgs::Float32 msg;
    msg.data = dist;
    distance_pub.publish(msg);

    cv::imshow("Image", floatImage);
    cv::waitKey(1);
}

void onMouse( int event, int x_pos, int y_pos, int flags, void* param){
    //if(flags == 1){//primary mouse down
        x = x_pos;
        y = y_pos;
    //}
    stringstream str;
    str << "mouse event:   x" << x_pos << "\ty:" << y << "\tevent: " << event<< endl;
    //ROS_INFO(str.str());
    cout << str.str() << endl;
}



/** 
 * Flattens a monochrome image for viewability
 */
cv::Mat flattenRawImage(cv::Mat original){
    cv::Mat_<float> float_image = original.clone();

    float max_val = 0;

    for(int i = 0; i < float_image.rows; ++i){
        for(int j = 0; j < float_image.cols; ++j){
            max_val = std::max(max_val, float_image(i,j));
        }   
    }   

    if(max_val > 0){ 
        float_image /= max_val;
    }   
    return float_image;

}   


double readDistance(cv::Mat depth, int x_pos, int y_pos){
    return depth.at<float>(y_pos, x_pos);
}   


//converts a ROS Image to an opencv float matrix
cv::Mat_<float> getImage(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat orig =  img_bridge_.imgMsgToCv(msg,"mono16");
    cv::Mat_<float> retu = orig;
    return retu;
}   

void addText(Mat image, string text, double data, int x, int y){
    stringstream ss;
    ss << text << data;
    putText(image, ss.str(), cvPoint(x, y), FONT_HERSHEY_PLAIN, 0.7, cvScalar(255,0, 0), 1, CV_AA);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundTracker");

    ros::NodeHandle n;
    cv::namedWindow("Image");
    ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 1, kinectCallBack);      
    //cv::setMouseCallback("Image", onMouse, 0);

    distance_pub = n.advertise<std_msgs::Float32>("//distance", 1);
    //pub_ = nh_.advertise<std_msgs::Float32>("tracking/normal_distance", 1);

    //while (original.isEmpty()){
    //ROS_INFO("Waiting for update...");
    //ros::spinOnce();
    //}
    
    ros::spin();

    return 0;
}

