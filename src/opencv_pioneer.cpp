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
        // image_transport::Publisher image_pub_;
        ros::Publisher red_centering_error_pub;
        ros::Publisher red_blobsize_pub;
        ros::Publisher blue_centering_error_pub;
        ros::Publisher blue_blobsize_pub;
        ros::Publisher green_centering_error_pub;
        ros::Publisher green_blobsize_pub;
    public:
        IMAGECONVERTER() : it_(nh_) {
            image_sub_ = it_.subscribe("/image", 1, &IMAGECONVERTER::imageCb, this);            // Subscribe to input video feed and publish output video feed
            // image_pub_ = it_.advertise("/image_converter/output_video", 1);
            red_centering_error_pub = nh_.advertise<std_msgs::Float32> ("/centering_error_red", 0);
            red_blobsize_pub = nh_.advertise<std_msgs::Float32> ("/blob_size_red", 0);
            blue_centering_error_pub = nh_.advertise<std_msgs::Float32> ("/centering_error_blue", 0);
            blue_blobsize_pub = nh_.advertise<std_msgs::Float32> ("/blob_size_blue", 0);
            green_centering_error_pub = nh_.advertise<std_msgs::Float32> ("/centering_error_green", 0);
            green_blobsize_pub = nh_.advertise<std_msgs::Float32> ("/blob_size_green", 0);
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
    cv::inRange(im_HSV, cv::Scalar(35,100,100), cv::Scalar(85,255,255), green_mask);            // creo la maschera del colore verde

    // detection del blob di colore rosso
    SimpleBlobDetector::Params params_red;                                                          // Setup SimpleBlobDetector parameters.
    params_red.minThreshold = 40;                                                                   // Change thresholds
    params_red.maxThreshold = 255;
    params_red.filterByArea = true;                                                                 // Filter by Area.
    params_red.minArea = 3.1415 * 20.0 * 20.0;                                                      // blob con raggio di 20
    params_red.maxArea = 3.1415 * 100.0 * 100.0;                                                    // blob con raggio di 100
    params_red.filterByColor = true;
    params_red.blobColor = 255;
    params_red.filterByInertia = false;
    params_red.filterByConvexity = false;
    params_red.filterByCircularity = false;
    std::vector<KeyPoint> keypoints_red;                                                            // Storage for blobs
    Ptr<SimpleBlobDetector> detector_red = SimpleBlobDetector::create(params_red);
    detector_red->detect(lower_red_mask, keypoints_red);

    // detection del blob di colore verde
    std::vector<KeyPoint> keypoints_green;                                                            // Storage for blobs
    Ptr<SimpleBlobDetector> detector_green = SimpleBlobDetector::create(params_red);
    detector_green->detect(green_mask, keypoints_green);

    // detection del blob di colore blu
    SimpleBlobDetector::Params params_blue;                                                          // Setup SimpleBlobDetector parameters.
    params_blue.minThreshold = 40;                                                                   // Change thresholds
    params_blue.maxThreshold = 255;
    params_blue.filterByArea = true;                                                                 // Filter by Area.
    params_blue.minArea = 3.1415 * 35.0 * 35.0;                                                      // blob con raggio di 40
    params_blue.maxArea = 3.1415 * 100.0 * 100.0;                                                    // blob con raggio di 100
    params_blue.filterByColor = true;
    params_blue.blobColor = 255;
    params_blue.filterByInertia = false;
    params_blue.filterByConvexity = false;
    params_blue.filterByCircularity = false;
    std::vector<KeyPoint> keypoints_blue;                                                            // Storage for blobs
    Ptr<SimpleBlobDetector> detector_blue = SimpleBlobDetector::create(params_blue);
    detector_blue->detect(blue_mask, keypoints_blue);

    // Visualizzazione a schermo delle immagini 
    Mat im_with_keypoints_red;
    Mat im_with_keypoints_blue;
    Mat im_with_keypoints_green;
    drawKeypoints(lower_red_mask, keypoints_red, im_with_keypoints_red, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(blue_mask, keypoints_blue, im_with_keypoints_blue, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(green_mask, keypoints_green, im_with_keypoints_green, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS); 
    imshow("image window", im);                                          
    // imshow("image_hsv", im_HSV); 
    // imshow("lower_red_mask", lower_red_mask);
    // imshow("blue_mask", blue_mask);     
    // imshow("upper_red_mask", upper_red_mask);  
    // imshow("green_mask", green_mask);            
    imshow("keypoints_red", im_with_keypoints_red);
    imshow("keypoints_blue", im_with_keypoints_blue);
    imshow("keypoints_green", im_with_keypoints_green);
    waitKey(3);

    // Invio dei dati riguardanti la dimensione del blob di colore rosso e il centroide del più grande blob rosso presente nel campo visivo
    std_msgs::Float32 center;                                                                   // l'immagine è di 256 pixel, il centro è a 128
    std_msgs::Float32 red_centering_error;
    std_msgs::Float32 red_size;
    center.data = 128.0;
    red_centering_error.data = 0.0;
    if(!keypoints_red.empty()) {
        vector <kp_info> kp_info_vector;
        kp_info_vector.resize(keypoints_red.size());
        for (int i=0; i<keypoints_red.size(); i++) {
            kp_info_vector[i].size = keypoints_red[i].size;  //?
            kp_info_vector[i].centroid_abscissa = keypoints_red[i].pt.x;
        }
        sort(kp_info_vector.begin(), kp_info_vector.end(), compareByLength);                    // calcolo del più grande blob di colore rosso
        red_centering_error.data = center.data - kp_info_vector[0].centroid_abscissa;
        red_size.data = kp_info_vector[0].size;
    }
    else {
        red_centering_error.data = 0.0;
        red_size.data = 0.0;
    }
    red_centering_error_pub.publish(red_centering_error);
    red_blobsize_pub.publish(red_size);

    // Invio dei dati riguardanti il blob di colore blu
    std_msgs::Float32 blue_size;
    std_msgs::Float32 blue_centering_error;
    blue_centering_error.data = 0.0;
    if(!keypoints_blue.empty()) {
        blue_centering_error.data = center.data - keypoints_blue[0].pt.x;
        blue_size.data = keypoints_blue[0].size;
    }
    else {
        blue_centering_error.data = 0.0;
        blue_size.data = 0.0;
    }
    blue_centering_error_pub.publish(blue_centering_error);
    blue_blobsize_pub.publish(blue_size);

    // Invio dei dati riguardanti il blob di colore verde
    std_msgs::Float32 green_size;
    std_msgs::Float32 green_centering_error;
    green_centering_error.data = 0.0;
    if(!keypoints_green.empty()) {
        green_centering_error.data = center.data - keypoints_green[0].pt.x;
        green_size.data = keypoints_green[0].size;
    }
    else {
        green_centering_error.data = 0.0;
        green_size.data = 0.0;
    }
    green_centering_error_pub.publish(green_centering_error);
    green_blobsize_pub.publish(green_size);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    IMAGECONVERTER ic;
    ros::spin();
    return 0;
}