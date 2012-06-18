
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
#define FLOOD_FILL 1

using namespace cv;
using namespace std;

cv::Mat_<float> getImage(const sensor_msgs::ImageConstPtr& msg);
double readDistance(cv::Mat depth, int x_pos, int y_pos); cv::Mat flattenRawImage(cv::Mat original); 
void loop(cv::Mat original); 
void addText(Mat image, string text, double data, int x, int y);


int x = 320;
int y = 240;
int divisor = 1;
//int ir_threshold = 99;
float ir_threshold = 0.9999999;
int hullThreshold = 21;
int dilateIterations = 10;
int houghRho = 1;
int houghTheta = 1; //in degrees
int houghThreshold = 68;
int houghMinLineLength= 19;
int houghMaxLineGap= 73;

int houghThickness= 1;

int houghFilteredThreshold = 68;
int houghFilteredMinLineLength= 19;
int houghFilteredMaxLineGap= 73;

int erode_iterations= 1;

int angleTolerance = 1;
int interceptTolerance= 1;

ros::Publisher distance_pub;


sensor_msgs::CvBridge img_bridge_;
cv::Mat orig(640, 480, CV_8UC1);

struct Feature {
    Point p1;
    Point p2;
    double slope;
    double distance;
    double intercept;
};

void kinectCallBack(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat original = getImage(msg);
    orig = original;
    //loop(original);

} 

Point cartesianPoint(const Point& screenPoint){
    return Point((screenPoint.x - 320), (480 - screenPoint.y));
}
Point screenPoint(const Point& cartesianPoint){
    return Point((cartesianPoint.x + 320), (480 - cartesianPoint.y));
}
Mat sobel(const Mat& original){
    Mat blurred = original.clone();

    Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    GaussianBlur( original, blurred, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Create window
    //namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Scharr( blurred, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    //Sobel( blurred, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Scharr( blurred, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    //Sobel( blurred, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    return grad;
}

Mat getThresholded(const Mat& img){
    Mat ret;
    //GaussianBlur(img, img, Size(5, 5), 1, 1, 1); 
    threshold(img, ret, ir_threshold, 1, CV_THRESH_BINARY);
    return ret;
}   

Mat getDilatedImage(const Mat& img){
    Mat ret;
    //dilate(InputArray src, OutputArray dst, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue() )
    dilate(img, ret, Mat(), Point(-1, -1), dilateIterations);
    return ret;
}
    
Point getGroundPoint(const Mat& dilated_binary){
    return Point();
}

int getSemicircleHeight(double radius, int x){
    return sqrt(radius * radius - x * x);
}


void loop(Mat original){

    Mat floatImage = flattenRawImage(original);
    //cv::cvtColor(floatImage, floatImage, CV_GRAY2);
    //Canny(floatImage, floatImage, 10, 100, 3);
    //cvtColor(floatImage, floatImage, CV_GRAY2BGR);
    //cv::Canny(floatImage, floatImage, 50, 150, 3);


    Mat grad = sobel(original);
    Mat flat_orig = flattenRawImage(original);
    Mat flat_grad= flattenRawImage(grad);
    Mat threshold = getThresholded(flat_grad);
    Mat dilated = getDilatedImage(threshold);
    Mat flat_dilated = flattenRawImage(dilated);

    Mat display;
    cvtColor(flat_dilated, display, CV_GRAY2BGR);
    //cvtColor(dilated, dilated, CV_GRAY2BGR);

    Point cartesian = cartesianPoint(Point(x,y));
    Point toDisplay = screenPoint(Point(cartesian.x, getSemicircleHeight(320,cartesian.x)));
    circle(display, toDisplay, 3, Scalar(0, 255, 0), -1);
    //addText(display, "", readDistance(dilated, x, y), x, y);
    //addText(display, "", readDistance(dilated, x, y), x, y);
    stringstream ss;
    ss << "( " << cartesian.x << " , " << getSemicircleHeight(320, cartesian.x)<< ")"<< 
        "\t ( " << toDisplay.x << " , " << toDisplay.y << ")" << endl;
    cout <<  ss.str() << endl;
    putText(display, ss.str(), cvPoint(toDisplay.x, toDisplay.y), FONT_HERSHEY_PLAIN, 0.7, cvScalar(255,0, 0), 1, CV_AA);
    //cvtColor(flat_dilated, display, CV_GRAY2BGR);

    //floodfill and isolation 
    Mat flooded = dilated.clone();
    floodFill(flooded, Point(320, 240), Scalar(1));
    //the convex hull we want is just the part that got flooded
    flooded = flooded - dilated;


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
#if FLOOD_FILL
    imshow("Flood Fill", flooded);
#endif
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
    //cout << str.str() << endl;
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

void addText(Mat image, string text, double data, int x, int y){ 
    stringstream ss;
    ss << text << data; 
    putText(image, ss.str(), cvPoint(x, y), FONT_HERSHEY_PLAIN, 0.7, cvScalar(255,0, 0), 1, CV_AA);
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
    createTrackbar("dilateIterations", "Threshold", &dilateIterations, 255, [](int x, void*){ dilateIterations= x; });

#endif
#if DILATED
    namedWindow("Dilated");
    setMouseCallback("Dilated", onMouse, 0);
#endif
#if FLOOD_FILL
    namedWindow("Flood Fill");
#endif

    while(n.ok ()){
        //cout << orig.isEmpty() << endl;
        ros::spinOnce();
        ros::Duration(0.001).sleep();
        loop(orig);


    }



    return 0;
}

