#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "AprilTags/TagFamily.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag36h11_other.h"
#include "perception/Observation.h"
#include "perception/perception.h"
using namespace std;
Perception::
Perception() : tag_detector(AprilTags::tagCodes36h11),
                   tagsize(0.175),
                   observations(){
}
Perception::
~Perception(){
}
void Perception::
    handleRGBImage(const sensor_msgs::Image::ConstPtr &msg)
{
    cout << "got rgb image" << endl;
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::
                                                                      image_encodings::BGR8);
    cv::Mat cv_image_gray(cv_image_ptr->image.rows, cv_image_ptr->image.cols,
                          CV_8UC1);
    cv::cvtColor(cv_image_ptr->image, cv_image_gray, CV_BGR2GRAY);
    vector<AprilTags::TagDetection> detections = tag_detector.extractTags(cv_image_gray);
    cout << "found " << detections.size() << " detections" << endl;
    observations.observations.clear();
    for (unsigned int i = 0; i < detections.size(); i++)
    {
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        // parameters read from camera/rgb/camera_info
        detections[i].getRelativeTranslationRotation(tagsize, 525.0, 525.0,319.5,239.5,translation, rotation);
        cout << " found detection " << detections[i].id << endl;
        cout << " translation:" << translation << endl;
        observations.observations.push_back(perception::Observation());
        observations.observations.back().range = sqrt(translation(0) *
                                                          translation(0) +
                                                      translation(1) * translation(1));
        observations.observations.back().bearing = atan2(translation(1),
                                                         translation(0));
        observations.observations.back().signature = detections[i].id;
        cout << " (range,bearing,signature) = (" << observations.observations.back().range << "," << observations.observations.back().bearing << ","
             << observations.observations.back().signature << ")" << endl;
    }
    //cv::imshow("RGB Image", cv_image_ptr->image);
    cv::waitKey(3);
    return;
}
void Perception::
    handleDepthImage(const sensor_msgs::Image::ConstPtr &msg)
{
    cout << "got depth image" << endl;
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::
                                                                  image_encodings::TYPE_16UC1);
    cv::imshow("Depth Image", cv_image->image);
    cv::waitKey(3);
    return;
}
ostream &
operator<<(ostream &out,
           const Perception &other)
{
    return out;
}
