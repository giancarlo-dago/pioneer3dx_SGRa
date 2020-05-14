#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d.hpp>

#include "std_msgs/Float32.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";



struct kp_info {
    float size;
    float centroid_abscissa;
};

bool compareByLength(const kp_info &a, const kp_info &b) {
    return a.size > b.size;                                                            // (se necessario rimettere) +10 per mitigare le oscillazioni quando non sa bene qual è il blob più grande
}




class IMAGECONVERTER {
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
        ros::Publisher centering_error_pub;
        ros::Publisher blobsize_pub;

    public:
        IMAGECONVERTER() : it_(nh_) {
            image_sub_ = it_.subscribe("/image", 1, &IMAGECONVERTER::imageCb, this);            // Subscribe to input video feed and publish output video feed
            image_pub_ = it_.advertise("/image_converter/output_video", 1);
            centering_error_pub = nh_.advertise<std_msgs::Float32> ("/centering_error", 0);
            blobsize_pub = nh_.advertise<std_msgs::Float32> ("/blob_size", 0);
            cv::namedWindow(OPENCV_WINDOW);
        }

        ~IMAGECONVERTER() {
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void imageCb(const sensor_msgs::ImageConstPtr&);   
};




void IMAGECONVERTER::imageCb(const sensor_msgs::ImageConstPtr& msg) {

    // Trasformazione da image_msg a cv_ptr tramite cv_bridge 
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);                      // trasformo image_msgs di ros in un cv_ptr per essere utilizzato in opencv

    // Costruzione della mask per fare la detection dei colori
    Mat im = cv_ptr->image;                                                                     // il campo image di cv_ptr contiene la matrice dei dati dell'immagine
    Mat im_HSV;
    Mat lower_red_mask;
    Mat upper_red_mask;
    Mat blue_mask;
    Mat green_mask;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);                                                        // trasformo la mia immagine da BGR a HSV perchè mi serve per fare il detect dei colori
    cv::inRange(im_HSV, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_mask);     // creo la maschera del colore rosso
    cv::inRange(im_HSV, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_mask);  // il rosso ha due range
    cv::inRange(im_HSV, cv::Scalar(100,150,0), cv::Scalar(140,255,255), blue_mask);             // creo la maschera del colore blu
    cv::inRange(im_HSV, cv::Scalar(40,100,100), cv::Scalar(80,255,255), green_mask);            // creo la maschera del colore verde

    // detection del blob di colore
    SimpleBlobDetector::Params params;                                                          // Setup SimpleBlobDetector parameters.
    params.minThreshold = 40;                                                                   // Change thresholds
    params.maxThreshold = 255;
    params.filterByArea = true;                                                                 // Filter by Area.
    params.minArea = 3.1415 * 20.0 * 20.0;                                                      // blob con raggio di 20
    params.maxArea = 3.1415 * 100.0 * 100.0;                                                    // blob con raggio di 100
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByCircularity = false;
    std::vector<KeyPoint> keypoints;                                                            // Storage for blobs
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    detector->detect(lower_red_mask, keypoints);

    // Visualizzazione a schermo delle immagini 
    Mat im_with_keypoints;
    drawKeypoints(lower_red_mask, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow(OPENCV_WINDOW, im);                                          
    imshow("image_hsv", im_HSV); 
    imshow("lower_red_mask", lower_red_mask);
    // imshow("upper_red_mask", upper_red_mask); 
    // imshow("blue_mask", blue_mask);      
    // imshow("green_mask", green_mask);            
    imshow("keypoints", im_with_keypoints);
    waitKey(3);

    // Invio dei dati 
    std_msgs::Float32 center;                                                                   // l'immagine è di 256 pixel, il centro è a 128
    std_msgs::Float32 centering_error;
    std_msgs::Float32 size;
    center.data = 128.0;
    centering_error.data = 0.0;
    int index_biggest_blob = 0;
    
    if(!keypoints.empty()) {
        vector <kp_info> kp_info_vector;
        kp_info_vector.resize(keypoints.size());
        for (int i=0; i<keypoints.size(); i++) {
            kp_info_vector[i].size = keypoints[i].size;  //?
            kp_info_vector[i].centroid_abscissa = keypoints[i].pt.x;
        }
        sort(kp_info_vector.begin(), kp_info_vector.end(), compareByLength);
        // for (int i=0; i<keypoints.size(); i++) {
        //     cout << kp_info_vector[i].size << "   " ;
        //     cout << kp_info_vector[i].centroid_abscissa << "    ";
        // }    
        // cout << endl;
        centering_error.data = center.data - kp_info_vector[0].centroid_abscissa;
        size.data = kp_info_vector[0].size;
    }
    else {
        centering_error.data = 0.0;
        size.data = 0.0;
    }
    centering_error_pub.publish(centering_error);
    blobsize_pub.publish(size);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    IMAGECONVERTER ic;
    ros::spin();
    return 0;
}