#include "opencv/includes.h"

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

    for (size_t i = 0; i < faces.size(); i++) {
        Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
        retFaces.push_back(pair<int, Rect>(i, faces[i]));
    }
    return retFaces;
}

int main(int argc, char **argv) {
    cout << "Running opencv package" << endl;

    VideoCapture cap = VideoCapture(0);

    init(argc, argv, "opencv_camera_head_tracking");
    NodeHandle nh;

    Publisher pub = nh.advertise<priorityhandler::PrioMsg>("Prio/cmd_vel", 100);

		if (!cap.isOpened())
	 	{
	 			cout << "Cannot open the video cam" << endl;
	 			return -1;
	 	}
		double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	 	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    Point priorCenter(0, 0);

    face_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"); // load face classifiers

    Mat frame;
		vector<pair<int, Rect>> points;
    Rate rate(10);
    cout << "Starting loop" << endl;
    while(ok()) {
        bool suc = cap.read(frame);
        if (!suc) {
            cout << "Can't read frame from camera" << endl;
            return -1;
        }
				points = detectFace(frame);
				//cout << "Found faces" << points.size() << endl;
        geometry_msgs::Twist msg;
        for ( size_t i = 0; i < points.size(); i++) {
            //cout << "Drawing point " << points[i].first << endl;
            Point faceCenter;
            faceCenter.x = points[i].second.x + points[i].second.width/2;
            faceCenter.y = points[i].second.y + points[i].second.height/2;
            // if in right side of screen
            int halfWidth = dWidth/2;
            int sensitivity = 50;
            float turnConstant = 0.25f;
            if (faceCenter.x > halfWidth + sensitivity) {
                // robot should turn right
                msg.angular.z = -turnConstant;
            } else if (faceCenter.x < halfWidth - sensitivity) {
                // robot should turn left
                msg.angular.z = turnConstant;
            } else {
                msg.linear.x = -4;
            }
            ellipse(frame, faceCenter, Size(points[i].second.width/2, points[i].second.height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
        }
        imshow("Image", frame);
        //geometry_msgs::Twist msg;
        //msg.linear.x = 4;
        //msg.angular.z = 4;
        priorityhandler::PrioMsg prio_msg;
        prio_msg.priority = 1;
        prio_msg.cmd = msg;
        pub.publish(prio_msg);
        //spinOnce();
        if (waitKey(1) == 27) {
            break;
        }
        rate.sleep();
    }
    return 0;
}

