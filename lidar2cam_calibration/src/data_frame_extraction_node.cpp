#include "data_frame_extraction.h"

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "data_frame_extraction_node");
    std::shared_ptr<lidar2cam_calibration::FrameExtraction> frame_extraction = std::make_shared<lidar2cam_calibration::FrameExtraction>();
    ros::spin();

    return 0;
}
