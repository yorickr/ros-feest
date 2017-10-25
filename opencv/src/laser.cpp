#include "opencv/includes.h"

// Function and values to obtain Mat from camera
Mat getFrameFromCam(int index);
VideoCapture cap;
bool camInit = false;
int frameHalfWidth;
int frameHeight;

std::pair<int, int> detectLaser(Mat frame)
{
    // Convert frame to HSV image
    Mat hsv, mask, closed, bin, bin16, labels;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // Red (0, 255, 191)
    int const hue = 160;
    int const lowerMargin = 15;
    int const upperMargin = 15;
    int lowerLimit = hue - lowerMargin;
    int upperLimit = hue + upperMargin;
    if (lowerLimit < 0)
        lowerLimit = 0;
    if (upperLimit > 179)
        upperLimit = 179;
    Scalar lower(lowerLimit, 50, 200);
    Scalar upper(upperLimit, 255, 255);
    inRange(hsv, lower, upper, mask);

    // Fill detected dot
    Mat const element = getStructuringElement(MORPH_ELLIPSE, Size(10, 10), Point());
    morphologyEx(mask, closed, MORPH_DILATE, element);

    SimpleBlobDetector::Params params;
    params.minThreshold = 1;
    params.maxThreshold = 255;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    //Flip image
    bitwise_not(closed, labels);

    // Detect blobs.
    vector<KeyPoint> keypoints;
    detector->detect(labels, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints(closed, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // // Show blobs
    imshow("keypoints", im_with_keypoints);

    // Only Laser detected
    if (keypoints.size() == 1)
        return make_pair(keypoints[0].pt.x, keypoints[0].pt.y);
    else
        return make_pair(0, 0);
}

int main(int argc, char **argv)
{
    init(argc, argv, "opencv_camera_laser_tracking");
    NodeHandle nh;
    Publisher pub = nh.advertise<priorityhandler::PrioMsg>("Prio/cmd_vel", 100);
    // Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
    // Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    Rate rate(10);

    double sensitivity = 75.0;

    float minHorSpeed = 0.25f;
    float horDif = 0.25f;

    float minVerSpeed = 0.15f;
    float verDif = 0.25f;

    while (ok())
    {
        geometry_msgs::Twist msg;
        Mat frame = getFrameFromCam(0);
        imshow("webcam", frame);
        pair<int, int> location = detectLaser(frame);

        //If laserpointer is detected
        if (location.first > 0 || location.second > 0)
        {
            ROS_INFO("Laser gedetecteerd");
            // Naar rechts
            if (location.first > frameHalfWidth + sensitivity)
            {
                msg.angular.z = ((location.first - (320.0 + sensitivity)) / (320.0 - sensitivity) * horDif + minHorSpeed) * -1.0;
            }
            // Naar links
            if (location.first < frameHalfWidth - sensitivity)
            {
                msg.angular.z = (1.0 - (location.first / (320.0 - sensitivity))) * horDif + minHorSpeed;
            }

            msg.linear.x = (1.0 - location.second / 480.0) * verDif + minVerSpeed;
            priorityhandler::PrioMsg prio_msg;
            prio_msg.priority = 3;
            prio_msg.cmd = msg;
            pub.publish(prio_msg);
        }

        waitKey(1);
        rate.sleep();
    }
    return 0;
}

Mat getFrameFromCam(int index)
{
    Mat frame;
    if (!camInit)
    {
        cap = VideoCapture(index);
        if (!cap.isOpened())
        {
            ROS_FATAL("Can't read frame from camera!");
            exit(1);
        }
        double camWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
        double camheight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        cout << "Frame size: " << camWidth << " x " << camheight << endl;
        frameHalfWidth = camWidth / 2;
        frameHeight = camheight;
        camInit = true;
    }
    bool succesfulRead = cap.read(frame);
    if (!succesfulRead)
    {
        ROS_FATAL("Can't read frame from camera!");
        exit(1);
    }
    return frame;
}
