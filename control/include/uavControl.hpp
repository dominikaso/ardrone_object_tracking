#ifndef UAVCONTROL_HPP
#define UAVCONTROL_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "keyboard/Key.h"
#include <vector>
#include <math.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "control/ColorMsg.h"
#include "sensor_msgs/CameraInfo.h"
#include "ardrone_autonomy/Navdata.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "bounding_box.hpp"
#include "camera.hpp"

const uint32_t WAIT_FOR_START=0;
const uint32_t TAKE_OFF=1;
const uint32_t GO_TO_POSITION=2;
const uint32_t LAND=3;


class uavControl
{
public:
    uavControl(ros::NodeHandle &n);
    virtual ~uavControl();

    //gettery
    int8_t getNumberOfObjects();
    std::vector<boundingBox> getBoundingBox();
    std::vector<std::string> getObjectColor();
    cv::Mat getDetectionImage();
    float getBatteryPercent();
    uint32_t getState();
    int32_t getAltitude();
    int getKeyDown();
    unsigned int getCameraWidth();
    unsigned int getCameraHeight();

    //publishery
    void takeOff();
    void land();
    void reset();
    void flyTo(double x, double y, double z, double turn);

    void sendObjectPosition(double D, double x, double yaw);

    // function to calculate information from camera
    double calculateAngle(boundingBox bbox); // return value in radian (-pi/2 - pi/2)
    double calculateDistanceWFromCamera(boundingBox bbox, double realObjectWidth);
    double calculateDistanceHFromCamera(boundingBox bbox, double realObjectHeight);
    double calculateXpossition(boundingBox bbox, double distanceFromCamera);
    double calculateYpossition(boundingBox bbox, double distanceFromCamera);

    // filtered data with camera
    std::vector<double> filterData(std::vector<double> objectPos, unsigned int numberOfSamples, const double DistanceFromObject);

private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;

    void init();

    ros::Subscriber _numberOfObjectSub;
    ros::Subscriber _boundingBoxSub;
    ros::Subscriber _objectColorSub;
    ros::Subscriber _cameraInfoSub;
    ros::Subscriber _keyboardSub;
    image_transport::Subscriber _detectionImageSub;

    //ardrone subscriber
    ros::Subscriber _infoWithUavSub;

    //ardrone publisher
    ros::Publisher _takeOffPub;
    ros::Publisher _landPub;
    ros::Publisher _resetPub;
    ros::Publisher _uavControlPub;

    //object position publisher
    ros::Publisher _objectPositionPub;

    void numberOfObjectsCb(const std_msgs::Int8::ConstPtr msg);
    void boundingBoxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr msg);
    void objectColorCb(const control::ColorMsg::ConstPtr msg);
    void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr msg);
    void detectionImageCb(const sensor_msgs::ImageConstPtr& msg);
    void keyboardCb(const keyboard::Key::ConstPtr msg);

    void infoWithUavCb(const ardrone_autonomy::Navdata::ConstPtr msg);

    int8_t _numberOfObjects=0;
    std::vector<boundingBox> _boundingBoxes;
    std::vector <std::string> _detectColor;
    camera _cameraInfo;
    cv::Mat _detectionImage;
    std::vector<std::vector<double>> _filteredData;

    int _keyDown=0;

    //drone information
    float _batteryPercent=0;
    uint32_t _state=0; //1:init, 2:landed. 3:flying, 4:hovering, 5: test,
                    //6: taking_off, 7:goto fix point, 8:landing, 9:loooping
    int32_t _altitude=0; // estimated altitude (cm)

    std_msgs::Empty _emp_msg;
    geometry_msgs::Twist msg_vel;
    geometry_msgs::Point _objectPos;
};

#endif // UAVCONTROL_HPP
