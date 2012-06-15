
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
#define FLOOD_FILL_EDGES 1
#define LINES 1
#define LINES_FILTERED 1

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

int slopeTolerance = 1;
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

Mat removeNoise(Mat& image){
    Mat src ;
    image.convertTo(src, CV_8UC1);
    Mat ret = src;
    vector<vector<Point> > contours;
    ret *=100;

    findContours(src, contours, CV_RETR_LIST, CV_CHAIN_CODE, Point(0, 0));
    for( int i = 0; i< contours.size(); i++ ){
        float area = contourArea(contours[i]);
        //removes shapes with an area less than hullThreshold
        if(fabs(area) < hullThreshold){
            //draws black over small things
            drawContours(ret, contours, i, Scalar(0,0,0), CV_FILLED);
        } else {
            //   vector<Rect> br = boundingRect(contours[i]);
            vector<Point> hull;
            convexHull(contours[i], hull, CV_CLOCKWISE);
            vector<vector<Point> > hullcontours;
            hullcontours.push_back(hull);
            drawContours(ret, hullcontours, -1, Scalar(255,255,255), CV_FILLED);
            //drawContours(ret, contours, i, Scalar(255,255,255));
        }   
    }   

    return ret;
}   


Mat sobel(Mat& original){
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

Mat getThresholded(Mat& img){
    Mat ret;
    //GaussianBlur(img, img, Size(5, 5), 1, 1, 1); 
    threshold(img, ret, ir_threshold, 1, CV_THRESH_BINARY);
    return ret;
}   

Mat getDilatedImage(Mat& img){
    Mat ret;
    //dilate(InputArray src, OutputArray dst, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue() )
    dilate(img, ret, Mat(), Point(-1, -1), dilateIterations);
    return ret;
}

vector<Vec4i> doHough(Mat& input,int rho, int numDegrees, int threshold, int minLineLength, int maxLineGap){
    vector<Vec4i> lines;
    HoughLinesP( input, lines, rho, CV_PI/180 * numDegrees, threshold, minLineLength, maxLineGap);
    return lines;
}

Mat showHoughLines(vector<Vec4i> lines, int thickness){

    //Mat lineResult(480, 640, CV_8UC1);
    Mat lineResult = Mat::zeros(480, 640, CV_8UC1);
    //cvtColor(lineResult, lineResult, CV_GRAY2BGR);
    for( size_t i = 0; i < lines.size(); i++ ){
        Point a = Point(lines[i][0], lines[i][1]);
        Point b = Point(lines[i][2], lines[i][3]);
        //(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        line( lineResult,a,b, Scalar(255,255,255), thickness, 8 );
    }
    
    return lineResult;
}

bool areFeaturesSame(const Feature& f1, const Feature& f2){
    return  abs (f1.slope - f2.slope) 
        < (10.0 / (double)slopeTolerance);
        /*
        &&
            abs (f1.intercept - f2.intercept) 
            < (10.0 / (double)interceptTolerance);

            */
}

bool isFeatureUnique(const vector<Feature>& features, const Feature& f){

    for(int i = 0; i< features.size(); i++){
        if(areFeaturesSame(features[i], f)){
            return false;
        }
    }
    return true;

}

vector<Feature> getFeatures(const vector<Vec4i>& lines){
    vector<Feature> features;


    for(size_t i = 0; i<lines.size(); i++){
        Feature f;

        Point p1 = Point(lines[i][0], lines[i][1]);
        Point p2 = Point(lines[i][2], lines[i][3]);
        double slope = (double) (p2.y - p1.y) / (double)(p2.x - p1.x);
        //(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        double b = (double)p1.y - slope * (double)p1.x;
        double distance = abs(b) / sqrt(slope * slope + 1);

        f.p1 = p1;
        f.p2 = p2;
        f.slope = slope;
        f.intercept = b;
        f.distance = distance;

        cout << "slope: " << slope << "\tintecept: " << b<< endl;

        if (isFeatureUnique(features, f)){
            features.push_back(f);
        }

    }
    cout << "DONE!" << endl;
    return features;
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
    circle(display, Point(x, y), 3, Scalar(0, 255, 0), -1);
    addText(display, "", readDistance(dilated, x, y), x, y);
    //cvtColor(flat_dilated, display, CV_GRAY2BGR);

    //floodfill and isolation 
    Mat flooded = dilated.clone();
    floodFill(flooded, Point(320, 469), Scalar(1));
    //the convex hull we want is just the part that got flooded
    flooded = flooded - dilated;
    dilate(flooded, flooded, Mat(), Point(-1, -1), hullThreshold);

    Mat edges = sobel(flooded)*100;

    vector<Vec4i> lines = 
        doHough(edges,
                houghRho, 
                houghTheta, 
                houghThreshold, 
                houghMinLineLength, 
                houghMaxLineGap);
    Mat lineResult = showHoughLines(lines, houghThickness);

    Vector<Feature> features = getFeatures(lines);

    /*
    vector<Vec4i> filteredLines = 
        doHough(lineResult, 
                houghRho, 
                houghTheta, 
                houghFilteredThreshold, 
                houghFilteredMinLineLength, 
                houghFilteredMaxLineGap);
    Mat filteredLineResult = showHoughLines(filteredLines, 1);
    */

    addText(lineResult, "Num_lines:", lines.size(), 10, 10);
    addText(lineResult, "Num_lines:", features.size(), 10, 30);
    //addText(filteredLineResult, "Num_lines:", filteredLines.size(), 10, 10);

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
#if FLOOD_FILL_EDGES
    imshow("Flood Fill Edges", edges );
#endif
#if LINES 
    imshow("Lines", lineResult);
#endif
#if LINES_FILTERED
    //imshow("Filtered Lines", filteredLineResult);
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
    createTrackbar("hullThreshold", "Threshold", &hullThreshold, 255, [](int x, void*){ hullThreshold= x; });
    createTrackbar("dilateIterations", "Threshold", &dilateIterations, 255, [](int x, void*){ dilateIterations= x; });

    createTrackbar("houghRho", "Threshold", &houghRho, 255, [](int x, void*){houghRho = x; });
    createTrackbar("houghTheta", "Threshold", &houghTheta, 255, [](int x, void*){houghTheta= x; });

    createTrackbar("houghThreshold", "Threshold", &houghThreshold, 255, [](int x, void*){ houghThreshold= x; });
    createTrackbar("houghMinLineLength", "Threshold", &houghMinLineLength, 255, [](int x, void*){houghMinLineLength = x; });
    createTrackbar("houghMaxLineGap", "Threshold", &houghMaxLineGap, 255, [](int x, void*){ houghMaxLineGap= x; });

    createTrackbar("houghThickness", "Threshold", &houghThickness, 255, [](int x, void*){ houghThickness= x; });

    createTrackbar("houghFilteredThreshold", "Threshold", &houghFilteredThreshold, 255, [](int x, void*){ houghFilteredThreshold= x; });
    createTrackbar("houghFilteredMinLineLength", "Threshold", &houghFilteredMinLineLength, 255, [](int x, void*){houghFilteredMinLineLength = x; });
    createTrackbar("houghFilteredMaxLineGap", "Threshold", &houghFilteredMaxLineGap, 255, [](int x, void*){ houghFilteredMaxLineGap= x; });
    createTrackbar("erode_iterations", "Threshold", &erode_iterations, 255, [](int x, void*){ erode_iterations= x; });

    createTrackbar("slopeTolerance", "Threshold", &slopeTolerance, 255, [](int x, void*){ slopeTolerance= x; });
    createTrackbar("interceptTolerance", "Threshold", &interceptTolerance, 255, [](int x, void*){ interceptTolerance= x; });

#endif
#if DILATED
    namedWindow("Dilated");
    setMouseCallback("Dilated", onMouse, 0);
#endif
#if FLOOD_FILL
    namedWindow("Flood Fill");
#endif
#if FLOOD_FILL
    namedWindow("Flood Fill Edges");
#endif
#if LINES 
    namedWindow("Lines");
#endif
#if LINES_FILTERED
    namedWindow("Filtered Lines");
#endif

    while(n.ok ()){
        //cout << orig.isEmpty() << endl;
        ros::spinOnce();
        ros::Duration(0.001).sleep();
        loop(orig);


    }



    return 0;
}

