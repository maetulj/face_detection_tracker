/**
 * FILENAME:    face_detection_tracker.h
 *
 * DESCRIPTION:
 *
 * This file includes the definition of the FaceDetectionTracker class.
 * This class firstly converts the sensor_msgs::Image to cv::Mat, then
 * it detects the faces in the given image and tracks the faces.
 *
 *
 * AUTHOR:  maetulj
 *
 * START DATE: 28.11.2016
 *
 */
#ifndef FACE_DETECTION_TRACKER_H
#define FACE_DETECTION_TRACKER_H


//C++ related includes.
#include <cstdio>
#include <cmath>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <memory>

// ROS related includes.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

// OpenCV related includes.
#include <cv_bridge/cv_bridge.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>


// Third party includes for tracking.
#include "../cf_libs/kcf/kcf_tracker.hpp"
#include <perception_msgs/Rect.h>

// Self defined includes.
#include <perception_msgs/Rect.h>

#include <faces_storage.hpp>

// Debug defines.
// Include this if you want to have visual output.
#define DEBUG


using namespace cv;


const std::string PATH = "/home/maetulj/tiago_ws/src/face_detection_tracker/";

const int HEIGHT = 48;
const int WIDTH = 48;

const int FRAMESRECORDED = 50;


/**
 * @brief      Class for face detection and tracking.
 */
class FaceDetectionTracker
{
public:

    /**
     * @brief      Constructor for the class.
     */
    FaceDetectionTracker(bool train, int faceId = 0);

    /**
     * @brief      Destructor.
     */
    ~FaceDetectionTracker();

    /**
     * @brief      Function for detecting and displaying the faces.
     *
     * @param[in]  frame  The frame
     */
    void detectAndDisplay(cv::Mat frame);

    /**
     * @brief      Track the object.
     */
    void track();

    void readCSV(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';');

    void readCSVLegend(const string& filename, char separator = ';');

    /**
     * @brief       Train the face detector.
     */
    void trainDetector();

    /**
     * @brief       Recognize the faces on the video.
     */
    void recognizeFace();

    /**
     * @brief      Save face as a separate jpg image.
     */
    void saveFaceAsJPG(cv::Mat frame, cv::Point p1, int height, int width);

private:
    // Global variables.
    static bool m_newImage_static;
    static bool m_newBB_static;

    // Storage for face width and height.
    // The images for Fischer recognition have to be the same size.
    static int m_faceHeight;
    static int m_faceWidth;

    // Face counter.
    static int m_faceCounter;

    // Person id.
    static int m_personId;

    static bool savedImages;

    // The ros node handle.
    ros::NodeHandle m_node;

    std::string m_windowName{"Face detection"};
    std::string m_directory{"/home/maetulj/tiago_ws/src/face_detection_tracker/"};

    // Buffer for publishers, subscibers.
    int m_queuesize = 2;

    ////////////////////////////
    /// Face detection part. ///
    ////////////////////////////

    // Helper member variables for image transformation.
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_imageSub;
    image_transport::Publisher m_imagePub;

    // Publisher.
    ros::Publisher m_perceptPub;

    // POinter to the cv image.
    cv_bridge::CvImagePtr m_cvPtr;

    // Name the haarcascades for frontal and profile face detection.
    std::string m_frontalFaceCascadeName{"haarcascade_frontalface_alt.xml"};
    std::string m_profilefaceCascadeName{"haarcascade_profileface.xml"};

    // Cascade classifiers.
    cv::CascadeClassifier m_frontalfaceCascade;
    cv::CascadeClassifier m_profilefaceCascade;

    // Variable to hold the detection rectangle.
    perception_msgs::Rect m_msgRect;

    /**
     * @brief      Callback for the sensor_msgs::Image.
     *
     * @param[in]  msg   The image in a form of a sensor_msgs::Image.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    // Detected faces.

    std::vector<Rect> m_faces;

    // TO BE IMPLEMENTED!
    // Save detected faces in a storage class.
    // std::vector<FacesStorage> m_storeFaces;

    // Point in the upper left corner.
    cv::Point m_p1;

    // Point in the lower right corner.
    cv::Point m_p2;

    // Height and width of the bounding box.
    int m_width;
    int m_height;

    //////////////////////
    /// Tracking part. ///
    //////////////////////

    // Cv Bridge variables for transforming sensor_msgs::Image into cv::Mat
    cv_bridge::CvImagePtr m_inImg;

    perception_msgs::Rect m_inBb;
    perception_msgs::Rect m_outBb;

    // local variables
    cv::Mat img;
    cv::Rect bb;

    // Detected face number to track.
    int i = 0;

    // Declare and initialize subscribers, rgb image and the 2D region to track.
    ros::Subscriber rgbimgSub;

    // Face detection subscriber.
    ros::Subscriber bbSub;

    //Declare and initialize publishers, 2D region tracked region.
    ros::Publisher bbPub;

    // Tracker parameters.
    cf_tracking::KcfParameters m_paras;

    std::vector<cf_tracking::KcfTracker*> vKCF;

    // Declare tracker.
    cf_tracking::KcfTracker *cKCF;

    // The tracker is running.
    bool tracking = false;

    // If the tracker is on frame.
    bool targetOnFrame = false;

    /**
     * @brief      Callback function for gettint sensor image.
     *
     * @param[in]  _img  The image
     */
    void callbackimage(const sensor_msgs::ImageConstPtr &_img);

    /**
     * @brief      Callback for perception rectangle.
     *
     * @param[in]  _bb   Bounding box.
     */
    void callbackbb(const perception_msgs::RectConstPtr &_bb);

    /////////////////////////
    /// Recognizing part. ///
    /////////////////////////

    // Define if training or recognizing.
    bool m_train;

    // These vectors hold the images and corresponding labels:
    vector<Mat> m_images;
    vector<int> m_labels;

    // Hold the legend for labels.
    std::map<int, std::string> m_labelLegend;

    int m_imWidth;
    int m_imHeight;

    Ptr<FaceRecognizer> m_model;

    // The tracked person label.
    int m_trackedPersonId;
    int m_trackedPerson;
};

#endif // FACE_DETECTION_TRACKER_H
