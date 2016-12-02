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
    ros::init(argc, argv, "face_detection_tracker");

    /** @face_detection_tracker */
    // FaceDetectionTracker fd;
    std::shared_ptr<FaceDetectionTracker> fdPtr;

    ROS_INFO("initialized the class");

    std::cout << "Do you want to train? yes/no" << std::endl;

    std::string train;

    std::cin >> train;

    std::cout << std::endl;

    if (train == "yes")
    {
        std::cout << "Input id of the face to learn: ";

        int faceId;

        std::cin >> faceId;

        std::cout << std::endl << "Training face: " << faceId << std::endl;

        // Create FaceDetectionTracker.
        fdPtr = std::make_shared<FaceDetectionTracker>(true, faceId);

        // fdPtr->trainDetector();
    }
    else
    {
        // Create FaceDetectionTracker.
        fdPtr = std::make_shared<FaceDetectionTracker>(false);

        std::cout << "Recognizing.." << std::endl;
    }

    while (ros::ok())
    {
        ros::spinOnce();

        if (train != "yes")
        {


            fdPtr->track();
        }
    }


    return 0;
}
