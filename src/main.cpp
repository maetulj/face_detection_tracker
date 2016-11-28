/**
 * FILENAME:    main.cpp
 *
 * DESCRIPTION:
 *
 * This is the main file for the face detection tracker.
 * It includes the main function which initializes the face detection and
 * tracker and starts it.
 *
 *
 * AUTHOR:  maetulj
 *
 * START DATE: 28.11.2016
 *
 */
#include "face_detection_tracker.h"


/**
 * @brief      Main function to execute the program.
 *
 * @param      argc  Argument for the main function.
 * @param      argv  Argument for the main function.
 *
 * @return     { description_of_the_return_value }
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_detection");

    /** @face_detection_tracker */
    FaceDetectionTracker fd;

    ROS_INFO("initialized the class");

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
