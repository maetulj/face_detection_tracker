ROS Package for detecting, tracking and recognizing faces.
It depends greatly on the teaching faces.

To teach from video:
- start the program with:
        rosrun face_detection_tracker face_detection tracker

- the program asks if you want to start teaching or not. To teach write:
        yes

- IMPORTANT!! Before starting teaching be sure to delete (or empty) the file "face_images/face_images.csv"

- the program now waits either for "/kinect2/qhd/image_color_rect" or "/pseye_camera/image_raw"
  and then learns from the topic (first 50 frames with detected face)

- input the id with which you want to save the detected face

- you can save the names for persons in the file "face_images/face_legend.csv"

- after the first run the program will crash, which is perfectly okay! Because there was only one person in the video,
  but the Fischer algorithm needs at least two of them. Start the program again and input an id (different from previous one)
  and play the video with a different person.

- The program should now have enough data to recognize faces.
  If not, repeat the previous step with either the same video or a third one.

- When the program outputs: "The face has been remembered :)" then the faces have been (more or less) successfully saved
  and you can close the program (ctrl + c). The teaching has been done now.


To have the best results have good videos (face not to far away)


To run the program:
- IMPORTANT!! You have to have "face_images/face_legend.csv", "face_images/face_images.csv" and "face_images/fischer_faces.yml" files

- start the program with:
        rosrun face_detection_tracker face_detection tracker

- the program asks if you want to start teaching or not. To teach write:
        no

- the program now waits either for "/kinect2/qhd/image_color_rect" or "/pseye_camera/image_raw"
  and then searches for faces from them. Who was recognized can be seen in the displayed window.
  The quality of recognition greatly depends on the teaching!
