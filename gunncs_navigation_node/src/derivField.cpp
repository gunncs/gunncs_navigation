
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
int divisor = 1;

ros::Publisher distance_pub;


sensor_msgs::CvBridge img_bridge_;
cv::Mat orig;

void kinectCallBack(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat original = getImage(msg);
    orig = original;
    loop(original);

} 

Mat sobel(Mat original){

    Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    GaussianBlur( original, original, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Create window
    //namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( original, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( original, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( original, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( original, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    return grad;
}


void loop(cv::Mat original){


    Mat floatImage = flattenRawImage(original);
    //cv::cvtColor(floatImage, floatImage, CV_GRAY2);
    //Canny(floatImage, floatImage, 10, 100, 3);
    //cvtColor(floatImage, floatImage, CV_GRAY2BGR);
    //cv::Canny(floatImage, floatImage, 50, 150, 3);


    /*
    std_msgs::Float32 msg;
    msg.data = dist;
    distance_pub.publish(msg);
    */
    //Mat final = flattenRawImage(grad);

    Mat grad = sobel(original);
    Mat grad2 = sobel(grad);

    grad = divisor*grad;
    grad2 = divisor*grad2;
    cv::imshow("Gradient", grad );
    cv::imshow("Gradient2", grad2 );
    cv::imshow("Depth", floatImage);
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

void addText(Mat image, string text, double data, int x, int y){ stringstream ss;
    ss << text << data;
    putText(image, ss.str(), cvPoint(x, y), FONT_HERSHEY_PLAIN, 0.7, cvScalar(255,0, 0), 1, CV_AA);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "derivField");

    ros::NodeHandle n;
    cv::namedWindow("Depth");
    cv::namedWindow("Gradient");
    cv::namedWindow("Gradient2");
    ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 1, kinectCallBack);      
    //cv::setMouseCallback("Image", onMouse, 0);

    distance_pub = n.advertise<std_msgs::Float32>("/distance", 1);
    //pub_ = nh_.advertise<std_msgs::Float32>("tracking/normal_distance", 1);
    cv::namedWindow("hull points");

    cv::createTrackbar("point1", "hull points", &divisor, 10, [] (int x, void*){divisor= x;});


    //while (original.isEmpty()){
    //ROS_INFO("Waiting for update...");
    //ros::spinOnce();
    //}

    ros::spin();

    return 0;
}

