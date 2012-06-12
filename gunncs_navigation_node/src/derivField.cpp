
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


#define ORIGINAL 1
#define XGRAD 0
#define YGRAD 0
#define GRAD 1
#define THRESHOLD 1
#define DILATED 1

using namespace cv;
using namespace std;

cv::Mat_<float> getImage(const sensor_msgs::ImageConstPtr& msg);
double readDistance(cv::Mat depth, int x_pos, int y_pos); cv::Mat flattenRawImage(cv::Mat original); 
void loop(cv::Mat original); 
void addText(Mat image, string text, double data, int x, int y);


int x = 320;
int y = 240;
int divisor = 1;
int ir_threshold = 99;

ros::Publisher distance_pub;


sensor_msgs::CvBridge img_bridge_;
cv::Mat orig(640, 480, CV_8UC1);

void kinectCallBack(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat original = getImage(msg);
    orig = original;
    //loop(original);

} 

Mat sobel(Mat& original){

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
    Scharr( original, grad_x, ddepth, 2, 0, scale, delta, BORDER_DEFAULT );
    //Sobel( original, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Scharr( original, grad_y, ddepth, 0, 2, scale, delta, BORDER_DEFAULT );
    //Sobel( original, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    return grad;
}

Mat getThresholded(Mat& img){
    Mat ret;
    //GaussianBlur(img, img, Size(5, 5), 1, 1, 1); 
    threshold(img, ret, (float)(ir_threshold/100.0), 1, CV_THRESH_BINARY);
    return ret;
}   

Mat getDilatedImage(Mat& img){
    Mat ret;
    //dilate(InputArray src, OutputArray dst, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue() )
    dilate(img, ret, Mat(), Point(-1, -1), 10 );
    return ret;
}


vector<Point> floodFill(Mat& original){


    int channels = original.channels();

    int nRows = original.rows * channels;
    int nCols = original.cols;
    Mat retu = original.clone();

    if (original.isContinuous()){   
        nCols *= nRows;
        nRows = 1;
    }   

    int i,j;
    float* originalP;
    float* retuP;
    /*
    originalP = original.ptr<float>(0);
    //uint8_t* originalP = original.data;
    uint8_t* retuP = retu.data;
    for(int i = 0; i< nRows; i++){
        for(int j = 0; j<nCols; j++){
            //pixel =// 
            //
            originalP[i*retu.cols + j] = 0;

        }
    }
    */

    for( i = 0; i < nRows; ++i){   
        originalP = original.ptr<float>(i);
        retuP = retu.ptr<float>(i);

        for ( j = 0; j < nCols; ++j){   
            retuP[j] = 0;
            int col = j % original.cols;
        }   
    }
    //because continuous, just 1x(640x480 pixels)
    
    imshow("Test", retu);

    return vector<Point>();
}


void loop(Mat original){

    Mat floatImage = flattenRawImage(original);
    //cv::cvtColor(floatImage, floatImage, CV_GRAY2);
    //Canny(floatImage, floatImage, 10, 100, 3);
    //cvtColor(floatImage, floatImage, CV_GRAY2BGR);
    //cv::Canny(floatImage, floatImage, 50, 150, 3);

    Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    Mat blurred;
    GaussianBlur( original, blurred, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Create window
    //namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Scharr( blurred, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Scharr( blurred, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    //Sobel( blurred, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    //Mat final = flattenRawImage(grad);

    Mat flat_orig = flattenRawImage(blurred);
    Mat flat_grad= flattenRawImage(grad);
    Mat threshold = getThresholded(flat_grad);
    Mat dilated = getDilatedImage(threshold);
    Mat flat_dilated = flattenRawImage(dilated);

    Mat display;
    cvtColor(flat_dilated, display, CV_GRAY2BGR);
    //cvtColor(dilated, dilated, CV_GRAY2BGR);
    circle(display, Point(x, y), 3, Scalar(0, 255, 0), -1);
    addText(display, "", readDistance(dilated, x, y), x, y);
    //cvtColor(flat_dilated, display, CV_GRAY2BGR);

    Mat flooded = dilated.clone();
    floodFill(flooded, Point(320, 479), Scalar(1));
    flooded = flooded - dilated;
    //flooded = dilated - flooded;
    //floodFill(dilated);

#if ORIGINAL
    imshow("Original", flat_orig);
#endif
#if GRAD 
    imshow("Grad", flat_grad);
#endif
#if XGRAD 
    imshow("xGrad", flattenRawImage(abs_grad_x));
#endif
#if YGRAD 
    imshow("yGrad", flattenRawImage(abs_grad_y));
#endif
#if THRESHOLD 
    imshow("Threshold", flattenRawImage(threshold));
#endif
#if DILATED
    imshow("Dilated", display);
#endif
    imshow("Test", flooded);
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
    return depth.at<float>(Point(x_pos, y_pos));
}   


//converts a ROS Image to an opencv float matrix
cv::Mat_<float> getImage(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat orig =  img_bridge_.imgMsgToCv(msg,"mono16");
    cv::Mat_<float> retu = orig;
    return retu;
}   

void addText(Mat image, string text, double data, int x, int y){ stringstream ss;
    ss << text << data; putText(image, ss.str(), cvPoint(x, y), FONT_HERSHEY_PLAIN, 0.7, cvScalar(255,0, 0), 1, CV_AA);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "derivField");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 1, kinectCallBack);      

    distance_pub = n.advertise<std_msgs::Float32>("/distance", 1);
    //pub_ = nh_.advertise<std_msgs::Float32>("tracking/normal_distance", 1);

#if ORIGINAL
    namedWindow("Original");
#endif
#if GRAD 
    namedWindow("Grad");
#endif
#if XGRAD 
    namedWindow("xGrad");
#endif
#if XGRAD 
    namedWindow("yGrad");
#endif
#if THRESHOLD 
    namedWindow("Threshold");
    createTrackbar("ir", "Threshold", &ir_threshold, 255, [](int x, void*){ ir_threshold = x; });

#endif
#if DILATED
    namedWindow("Dilated");
    setMouseCallback("Dilated", onMouse, 0);
#endif
    namedWindow("Test");

    while(n.ok ()){
        //cout << orig.isEmpty() << endl;
        ros::spinOnce();
        ros::Duration(0.001).sleep();
        loop(orig);


    }



    return 0;
}

