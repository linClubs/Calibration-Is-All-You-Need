#include "image_procession.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "image_procession_node");
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);

    cv::Mat frame;
    std::shared_ptr<lidar2cam_calibration::ImageProcession> img1 = std::make_shared<lidar2cam_calibration::ImageProcession>();
    if (!cap.isOpened())
    {
        std::cout << "\n--Please check out the video capture...\n" << std::endl;
    }

    while (1)
    {
        cap >> frame;
        img1->getImage(frame);
        cv::Mat img = img1->detectMarkers();
        cv::imshow("img", img);
        if(cv::waitKey(100) == 'q')
        {
            break;
        }
    }
    return 0;
}
