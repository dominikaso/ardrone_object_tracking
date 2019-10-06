#ifndef CHECKCOLOR_HPP
#define CHECKCOLOR_HPP

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "objectColor/ColorMsg.h"
#include "bounding_box.hpp"


class checkColor
{
public:
    checkColor(ros::NodeHandle &n);
    virtual ~checkColor();

    //gettery
    int8_t getNumberOfObjects();
    std::vector<boundingBox> getBoundingBox();
    cv::Mat getDetectionImage();

    void publishColor(objectColor::ColorMsg color);

    std::vector<cv::Mat> cropToObject(cv::Mat& img, std::vector<boundingBox>& bbox);
    std::string findDominantColor(cv::Mat& img);


    void init();

private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;

    ros::Subscriber _numberOfObjectSub;
    ros::Subscriber _boundingBoxSub;
    image_transport::Subscriber _detectionImageSub;

    ros::Publisher _colorPub;

    void numberOfObjectsCb(const std_msgs::Int8::ConstPtr msg);
    void boundingBoxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr msg);
    void detectionImageCb(const sensor_msgs::ImageConstPtr& msg);

    int8_t _numberOfObjects=0;
    std::vector<boundingBox> _boundingBoxes;
    cv::Mat _detectionImage;

    std::string _adjustColor(std::vector<int> channel,std::vector<double> maxVal, int sizeObject);
};



#endif // CHECKCOLOR_HPP
