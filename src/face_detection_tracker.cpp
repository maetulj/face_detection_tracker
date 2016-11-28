#include "face_detection.h"
#include <sensor_msgs/JointState.h>

/**
 * @brief      Constructor for the class.
 */
FaceDetectionTracker::FaceDetectionTracker() :
        m_it(m_node)
{
    // Subscribe to input video feed and publish output video feed.
    m_imageSub = m_it.subscribe("/kinect2/qhd/image_color_rect", 1, &FaceDetectionTracker::imageCallback, this);

    // Advertise the rectangle with information about the detected face.
    m_perceptPub = m_node.advertise<perception_msgs::Rect>("/face_detection/bb", 1);

    // Load the cascades.
    // // Frontal face.
    if(!m_frontalfaceCascade.load(m_directory + m_frontalFaceCascadeName))
    {
        ROS_ERROR("Error loading frontal face cascade!");
        return;
    }

    // Profile face.
    if(!m_profilefaceCascade.load(m_directory + m_profilefaceCascadeName))
    {
        ROS_ERROR("Error loading profile face cascade!");
        return;
    }
}

/**
 * @brief      Destructor.
 */
FaceDetectionTracker::~FaceDetectionTracker()
{}

/**
 * @brief      Callback for the sensor_msgs::Image.
 *
 * @param[in]  msg   The image in a form of a sensor_msgs::Image.
 */
void FaceDetectionTracker::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // Try to convert the message to cv image.
    try
    {
        m_cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Resize the image to half its size.
        cv::resize(m_cvPtr->image, m_cvPtr->image, cv::Size(m_cvPtr->image.cols / 2, m_cvPtr->image.rows / 2));
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Apply the classifiers to the frame.
    if(m_cvPtr)
    {
        // ROS_INFO("Captured a frame!");
        detectAndDisplay(m_cvPtr->image);
    }
    else
    {
        ROS_INFO("No captured frame!");
    }

    // Output modified video stream.
    // m_imagePub.publish(m_cvPtr->toImageMsg());
}

/**
 * @brief      Function for detecting and displaying the faces.
 *
 * @param[in]  frame  The frame
 */
void FaceDetectionTracker::detectAndDisplay(cv::Mat frame)
{
    std::vector<Rect> faces;
    Mat frameGray;

    cv::cvtColor(frame, frameGray, CV_BGR2GRAY);
    cv::equalizeHist(frameGray, frameGray);

    // Detect faces.
    m_frontalfaceCascade.detectMultiScale(frameGray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
    m_profilefaceCascade.detectMultiScale(frameGray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

    for( size_t i = 0; i < faces.size(); i++ )
    {
        // Center point
        // cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);

        // Point in the upper left corner.
        cv::Point p1(faces[i].x, faces[i].y);

        // Point in the lower right corner.
        cv::Point p2(faces[i].x + faces[i].width, faces[i].y + faces[i].height);

        /*
        cv::ellipse(frame, center, Size( faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        */
        // Draw the rectangle on the frame.
        cv::rectangle(frame, p1, p2, Scalar(0, 0, 255), 4, 8, 0);

        // Create the header.
        m_msgRect.header = m_cvPtr->header;
        m_msgRect.id = i;
        m_msgRect.x = faces[i].x;
        m_msgRect.y = faces[i].y;
        m_msgRect.height = faces[i].height;
        m_msgRect.width = faces[i].width;

        // Output perception_msgs.
        m_perceptPub.publish(m_msgRect);
    }

#ifdef DEBUG // Enable/Disable in the header.
    // Visualize the image with the fame.
    cv::imshow( m_windowName, m_cvPtr->image );
    cv::waitKey(3);
#endif
}
