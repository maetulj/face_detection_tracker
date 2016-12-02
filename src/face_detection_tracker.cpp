#include <face_detection_tracker.h>
#include <sensor_msgs/JointState.h>

// Initialize static members.
bool FaceDetectionTracker::m_newImage_static = false;
bool FaceDetectionTracker::m_newBB_static = false;

int FaceDetectionTracker::m_faceHeight = 0;
int FaceDetectionTracker::m_faceWidth = 0;

int FaceDetectionTracker::m_faceCounter = 0;

int FaceDetectionTracker::m_personId = 0;

/**
 * @brief      Constructor for the class.
 */
FaceDetectionTracker::FaceDetectionTracker()
 : m_it(m_node)
 , m_trackedPerson(-1)
{
    ///////////////////////
    /// Detection part. ///
    ///////////////////////

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


    /////////////////////
    /// Tracker part. ///
    /////////////////////

    // rgbimgSub = m_node.subscribe<sensor_msgs::Image>("kinect2/qhd/image_color_rect", m_queuesize, &FaceDetectionTracker::callbackimage, this);

    // bbSub = m_node.subscribe<perception_msgs::Rect>("face_detection/bb", m_queuesize, &FaceDetectionTracker::callbackbb, this);

    bbPub = m_node.advertise<perception_msgs::Rect>("tracker/bb", m_queuesize);

    m_paras.enableTrackingLossDetection = true;
    // paras.psrThreshold = 10; // lower more flexible
    m_paras.psrThreshold = 13.5; // higher more restricted to changes
    m_paras.psrPeakDel = 2; // 1;
}

/**
 * @brief      Destructor.
 */
FaceDetectionTracker::~FaceDetectionTracker()
{}

///////////////////////
/// Detection part. ///
///////////////////////


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
    //
    // Signal new image.
    m_newImage_static = true;
}

/**
 * @brief      Function for detecting and displaying the faces.
 *
 * @param[in]  frame  The frame
 */
void FaceDetectionTracker::detectAndDisplay(cv::Mat frame)
{
    Mat frameGray;

    cv::cvtColor(frame, frameGray, CV_BGR2GRAY);
    cv::equalizeHist(frameGray, frameGray);

    // Detect faces.
    m_frontalfaceCascade.detectMultiScale(frameGray, m_faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

    // Problems with profile face?
    // m_profilefaceCascade.detectMultiScale(frameGray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

    for( size_t i = 0; i < m_faces.size(); i++ )
    {
        // Center point
        // cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);

        // Point in the upper left corner.
        m_p1 = cv::Point(m_faces[i].x, m_faces[i].y);

        // Point in the lower right corner.
        m_p2 = cv::Point(m_faces[i].x + m_faces[i].width, m_faces[i].y + m_faces[i].height);

        m_width = m_faces[i].width;
        m_height = m_faces[i].height;

        /*
        cv::ellipse(frame, center, Size( faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        */

        // Copy frame to local variable to not mingle with the tracking part.
        cv::Mat localFrame = frame;

        // Draw the rectangle on the frame.
        cv::rectangle(localFrame, m_p1, m_p2, Scalar(0, 0, 255), 4, 8, 0);

        // Create the header.
        m_msgRect.header = m_cvPtr->header;
        m_msgRect.id = i;
        m_msgRect.x = m_faces[i].x;
        m_msgRect.y = m_faces[i].y;
        m_msgRect.height = m_faces[i].height;
        m_msgRect.width = m_faces[i].height;

        // Output perception_msgs.
        m_perceptPub.publish(m_msgRect);

        // Signal a new bounding box.
        m_newBB_static = true;

        if (m_train)
        {
            m_personId = i;

            saveFaceAsJPG(frame, m_p1, m_width, m_height);
        }
    }

    if (!m_train)
    {
        // Recognize the faces.
        recognizeFace();
    }


#ifdef DEBUG // Enable/Disable in the header.
    // Visualize the image with the frame.
    cv::imshow( m_windowName, m_cvPtr->image );
    cv::waitKey(3);
#endif
}

/////////////////////
/// Tracker part. ///
/////////////////////

/**
 * @brief      Read the camera image.
 *
 * @param[in]  _img  The image
 */
void FaceDetectionTracker::callbackimage(const sensor_msgs::ImageConstPtr &_img)
{
    //TODO
    //use cv_bridge to copy the data from the sensor message. use the the sensor_msgs::image_encodings to define the type of image.
    //in_img = ;
    //newImage = true;
    m_inImg = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);

    m_newImage_static = true;
}

/**
 * @brief      Callback to get the bounding box.
 *
 * @param[in]  _bb   { parameter_description }
 */
void FaceDetectionTracker::callbackbb(const perception_msgs::RectConstPtr &_bb)
{
    m_inBb = perception_msgs::Rect(*_bb);
    m_newBB_static = true;
}

/**
 * @brief      Track the face.
 */
void FaceDetectionTracker::track()
{
    // If we have a new image.
    if (m_newImage_static)
    {
        // Resize the image done in detecting part.

        // If new bounding box arrived (detected face) && we are not yet tracking anything.
        if (m_newBB_static && !tracking)
        {
            ROS_INFO("New bounding box!");
            // Create new tracker!
            cKCF = new cf_tracking::KcfTracker(m_paras);

            // Save the incoming bounding box to a private member.
            bb.x = m_p1.x; // m_inBb.x;
            bb.y = m_p1.y; // m_inBb.y;
            bb.height = m_height; // m_inBb.height;
            bb.width = m_width; //m_inBb.width;

            // Reinitialize the tracker.
            if (cKCF->reinit(m_cvPtr->image, bb)) // KcfTracker->reinit(cv::Mat, cv::Rect)
            {
                // This means that it is correctly initalized.
                tracking = true;
                targetOnFrame = true;
            }
            else
            {
                // The tracker initialization has failed.
                delete cKCF;
                tracking = false;
                targetOnFrame = false;
            }
        }

        // If the target is on frame.
        if (targetOnFrame)
        {
            // Save the incoming bounding box to a private member.
            bb.x = m_p1.x; // m_inBb.x;
            bb.y = m_p2.y; // m_inBb.y;
            bb.width = m_width; // m_inBb.width;
            bb.height = m_height; // m_inBb.height;

            // Update the current tracker (if we have one)!
            targetOnFrame = cKCF->update(m_cvPtr->image, bb); //vKCF[i]->update(img, bb); // KcfTracker->reinit(cv::Mat, cv::Rect)

            // If the tracking has been lost or the bounding box is out of limits.
            if (!targetOnFrame)
            {
                // We are not tracking anymore.
                delete cKCF;
                tracking = false;
            }
        }

        // If we are tracking, then publish the bounding box.
        if (tracking)
        {
            m_outBb.x = bb.x;
            m_outBb.y = bb.y;
            m_outBb.width = bb.width;
            m_outBb.height = bb.height;

            bbPub.publish(m_outBb);
        }


#ifdef DEBUG // Enable/Disable in the header.
        cv::Mat out_img;
        cv::cvtColor(m_cvPtr->image, out_img, CV_BGR2GRAY);// Convert to gray scale
        cv::cvtColor(out_img, out_img, CV_GRAY2BGR); //Convert from 1 color channel to 3 (trick)

        //Draw a rectangle on the out_img using the tracked bounding box.
        if (targetOnFrame)
        {
            cv::rectangle(out_img, cv::Point(bb.x, bb.y), cv::Point(bb.x + bb.width, bb.y + bb.height), cv::Scalar(255, 255, 255));

            if (m_trackedPerson != -1)
            {
                // Create the text we will annotate the box with:
                // std::string box_text = format("Prediction = %d", prediction);
                // Calculate the position for annotated text (make sure we don't
                // put illegal values in there):
                int pos_x = std::max(m_faces[m_trackedPersonId].tl().x - 10, 0);
                int pos_y = std::max(m_faces[m_trackedPersonId].tl().y + m_faces[m_trackedPersonId].height + 10, 0);

                // Display the information about the tracked person.
                cv::putText(out_img, m_labelLegend[m_trackedPerson], Point(pos_x, pos_y), CV_AA, 0.5, CV_RGB(255,0,0), 2.0);
            }
        }

        cv::imshow("Tracked object", out_img);
        cv::waitKey(1);
#endif

        // Signal that the image and bounding box are not new.
        m_newImage_static = false;
        m_newBB_static = false;
    } //end if new image
}

void FaceDetectionTracker::readCSV(const string& filename, vector<Mat>& images, vector<int>& labels, char separator)
{
    std::ifstream file(filename.c_str(), std::ifstream::in);

    if (!file)
    {
        std::string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }

    std::string line, path, classlabel;

    while (getline(file, line))
    {
        std::stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);

        if (!path.empty() && !classlabel.empty())
        {
            images.push_back(cv::imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}

void FaceDetectionTracker::readCSVLegend(const string& filename, char separator)
{
    std::ifstream file(filename.c_str(), std::ifstream::in);

    if (!file)
    {
        std::string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }

    std::string line, name, classlabel;

    while (getline(file, line))
    {
        std::stringstream liness(line);
        getline(liness, classlabel, separator);
        getline(liness, name);

        if (!name.empty() && !classlabel.empty())
        {
            int label = atoi(classlabel.c_str());
            m_labelLegend.insert(std::pair<int, std::string>(label, name));
        }
    }
}

void FaceDetectionTracker::trainDetector()
{
    if (m_train)
    {
        return;
    }

    std::string fn_csv = "/home/maetulj/tiago_ws/src/face_detection_tracker/face_images/face_images.csv";

    // Legend.
    std::string legend_csv = "/home/maetulj/tiago_ws/src/face_detection_tracker/face_images/face_legend.csv";

    // Read in the data (fails if no valid input filename is given, but you'll get an error message):
    try
    {
        readCSV(fn_csv, m_images, m_labels);

        readCSVLegend(legend_csv);
    }
    catch (cv::Exception& e)
    {
        std::cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << std::endl;
        // nothing more we can do
        exit(1);
    }

    // Get the height from the first image. We'll need this
    // later in code to reshape the images to their original
    // size AND we need to reshape incoming faces to this size:
    m_imWidth = m_images[0].cols;
    m_imHeight = m_images[0].rows;
    // Create a FaceRecognizer and train it on the given images:
    m_model = cv::createFisherFaceRecognizer();
    m_model->train(m_images, m_labels);

    ROS_INFO("The face has been remembered :)");
}

void FaceDetectionTracker::recognizeFace()
{
    // Clone the current frame:
    cv::Mat original = m_cvPtr->image;

    if (m_newBB_static)
    {
        Mat gray;
        cv::cvtColor(original, gray, CV_BGR2GRAY);

        for (int i = 0; i < m_faces.size(); i++)
        {
            // Process face by face:
            cv::Rect face_i = m_faces[i];

            // Crop the face from the image. So simple with OpenCV C++:
            cv::Mat face = gray(face_i);

            // Resizing the face is necessary for Eigenfaces and Fisherfaces. You can easily
            // verify this, by reading through the face recognition tutorial coming with OpenCV.
            // Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
            // input data really depends on the algorithm used.
            //
            // I strongly encourage you to play around with the algorithms. See which work best
            // in your scenario, LBPH should always be a contender for robust face recognition.
            //
            // Since I am showing the Fisherfaces algorithm here, I also show how to resize the
            // face you have just found:
            cv::Mat face_resized;

            cv::resize(face, face_resized, Size(m_imWidth, m_imHeight), 1.0, 1.0, INTER_CUBIC);

            // Now perform the prediction, see how easy that is:
            int prediction = -1;
            prediction = m_model->predict(face_resized);

            if (prediction != -1)
            {
                m_trackedPersonId = i;
                m_trackedPerson = prediction;
            }

            // And finally write all we've found out to the original image!
            // First of all draw a green rectangle around the detected face:
            cv::rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            // Create the text we will annotate the box with:
            // std::string box_text = format("Prediction = %d", prediction);
            // Calculate the position for annotated text (make sure we don't
            // put illegal values in there):
            // int pos_x = std::max(face_i.tl().x - 10, 0);
            // int pos_y = std::max(face_i.tl().y + face_i.height + 10, 0);
            // And now put it into the image:
            // putText(original, m_labelLegend[prediction], Point(pos_x, pos_y), CV_AA, 0.5, CV_RGB(255,0,0), 2.0);
        }
    }

#ifdef DEBUG
    // Show the result:
    imshow("face_recognizer", original);
    // And display it:
    char key = (char) waitKey(20);
#endif

}

/**
 * @brief      Save face as a separate jpg image.
 */
void FaceDetectionTracker::saveFaceAsJPG(cv::Mat frame, cv::Point p1, int height, int width)
{
    // Csv file to write image info.
    ofstream imagesCSV("/home/maetulj/tiago_ws/src/face_detection_tracker/face_images/face_images.csv");

    // Calculate second point based on the saved information.
    if (m_faceHeight && m_faceWidth)
    {
        height = m_faceHeight;
        width = m_faceWidth;
    }
    else if (!m_faceHeight && !m_faceWidth)
    {
        m_faceHeight = height;
        m_faceWidth = width;
    }

    // Second point.
    cv::Point p2(p1.x + width, p1.y + height);

    // Get the rectangle.
    cv::Rect face(p1, p2);

    // Crop the face from the image.
    cv::Mat faceImg = frame(face);

    // If we did not yet do 100 images.
    if (m_faceCounter < 100)
    {
        std::string imgId = std::to_string(m_faceCounter);
        std::string personId = std::to_string(m_personId);

        cv::imwrite("/home/maetulj/tiago_ws/src/face_detection_tracker/face_images/images/person_" + personId + "_" + imgId + ".jpg", faceImg);

        imagesCSV << "/home/maetulj/tiago_ws/src/face_detection_tracker/face_images/images/person_" + personId + "_" + imgId + ".jpg;" + personId << std::endl;

        ROS_INFO("Saved the image.");

        m_faceCounter++;
    }

    imagesCSV.close();
}
