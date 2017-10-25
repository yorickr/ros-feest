#include "opencv/includes.h"

#include "opencv/VideoFaceDetector.h"

CascadeClassifier face_cascade;

std::vector<std::pair<int, Rect>> detectFace(Mat frame) {

    std::vector<pair<int, Rect>> retFaces;
    std::vector<Rect> faces;
    Mat frame_gray;
    int minNeighbors = 2;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);   // Convert to gray
    equalizeHist(frame_gray, frame_gray);          // Equalize histogram

    // Detect face with open source cascade
    face_cascade.detectMultiScale(frame_gray, faces,
                                  1.1, minNeighbors,
                                  0 | CASCADE_SCALE_IMAGE, Size(30, 30));


    // if (faces.size()>0){ROS_INFO("Face detected!");}
    // else{ROS_INFO("No face detected!");}

    for (size_t i = 0; i < faces.size(); i++) {
        Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
        retFaces.push_back(pair<int, Rect>(i, faces[i]));
    }
    return retFaces;
}

int main(int argc, char **argv) {
    cout << "Running opencv package" << endl;

    VideoCapture cap = VideoCapture(1);

    init(argc, argv, "opencv_camera_head_tracking");
    NodeHandle nh;

    Publisher pub = nh.advertise<priorityhandler::PrioMsg>("Prio/cmd_vel", 100);

		if (!cap.isOpened())
	 	{
	 			cout << "Cannot open the video cam" << endl;
	 			return -1;
	 	}

    VideoFaceDetector detector("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml", cap);
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	 	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    face_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml"); // load face classifiers

    Mat frame;
		vector<pair<int, Rect>> points;
    Rate rate(10);
    cout << "Starting loop" << endl;
    while(ok()) {
        //bool suc = cap.read(frame);
        //if (!suc) {
            //cout << "Can't read frame from camera" << endl;
						//ROS_FATAL("Can't read frame from camera!");
            //return -1;
        //}
				//points = detectFace(frame);
				//cout << "Found faces" << points.size() << endl;
        detector >> frame;
        geometry_msgs::Twist msg;
        if (detector.isFaceFound()) {
            Point faceCenter = detector.facePosition();
             //if in right side of screen
            int halfWidth = dWidth/2;
            int sensitivity = 50;
            float turnConstant = 0.25f;
            if (faceCenter.x > halfWidth + sensitivity) {
                 //robot should turn right
                msg.angular.z = -turnConstant;
            } else if (faceCenter.x < halfWidth - sensitivity) {
                 //robot should turn left
                msg.angular.z = turnConstant;
            } else {
                msg.linear.x = 0.5f;
            }
            priorityhandler::PrioMsg prio_msg;
            prio_msg.priority = 1;
            prio_msg.cmd = msg;
            pub.publish(prio_msg);

            rectangle(frame, detector.face(), Scalar(255, 0, 0));
            //circle(frame, detector.facePosition(), Scalar(0, 255, 0));
            //ellipse(frame, faceCenter, Size(points[0].second.width/2, points[0].second.height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
        }
        imshow("Image", frame);

        //spinOnce();
        if (waitKey(1) == 27) {
            break;
        }
        rate.sleep();
    }
    return 0;
}

