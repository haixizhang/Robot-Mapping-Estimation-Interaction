#ifndef PERCEPTION_H
#define PERCEPTION_H
#include <iostream>
#include <deque>
#include <opencv2/imgproc/imgproc.hpp>
#include "AprilTags/TagDetection.h"
#include "AprilTags/TagDetector.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "perception/Observations.h"
class Perception {
  public:
	Perception();
	virtual ~Perception();
	void handleRGBImage( const sensor_msgs::Image::ConstPtr& msg );
	void handleDepthImage( const sensor_msgs::Image::ConstPtr& msg );
	AprilTags::TagDetector tag_detector;
	double tagsize;
	perception::Observations observations;
	};
std::ostream& operator<<( std::ostream& out, const Perception& other );
#endif /* PERCEPTION_H */

