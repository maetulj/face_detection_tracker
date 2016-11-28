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

// ROS related includes.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

// OpenCV related includes.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Self defined includes.
#include <perception_msgs/Rect.h>


// Debug defines.
// Include this if you want to have visual output.
// #define DEBUG


using namespace cv;


/**
 * @brief      Class for face detection and tracking.
 */
class FaceDetectionTracker
{
public:

    /**
     * @brief      Constructor for the class.
     */
    FaceDetection();

    /**
     * @brief      Destructor.
     */
    ~FaceDetection();


    /**
     * @brief      Callback for the sensor_msgs::Image.
     *
     * @param[in]  msg   The image in a form of a sensor_msgs::Image.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);


    /**
     * @brief      Function for detecting and displaying the faces.
     *
     * @param[in]  frame  The frame
     */
    void detectAndDisplay(cv::Mat frame);

private:
    // The ros node handle.
    ros::NodeHandle m_node;

    std::string m_windowName{"Face detection"};
    std::string m_directory{"/home/maetulj/tiago_ws/src/face_detection/"};

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
};

#endif // FACE_DETECTION_TRACKER_H
