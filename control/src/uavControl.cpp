#include "../include/uavControl.hpp"


uavControl::uavControl(ros::NodeHandle &n): _nh(n), _it(_nh)
{
    _filteredData.clear();
    init();
}

uavControl::~uavControl(){

}

void uavControl::init(){
    // camera subscriber
    _numberOfObjectSub=_nh.subscribe("darknet_ros/found_object",100, &uavControl::numberOfObjectsCb,this);
    _boundingBoxSub=_nh.subscribe("darknet_ros/bounding_boxes",100,&uavControl::boundingBoxCb,this);
    _objectColorSub=_nh.subscribe("checkColor/detectColor",100,&uavControl::objectColorCb,this);
    _cameraInfoSub=_nh.subscribe("/ardrone/front/camera_info",100, &uavControl::cameraInfoCb,this);
    _keyboardSub=_nh.subscribe("keyboard/keydown",100,&uavControl::keyboardCb,this);
    _detectionImageSub=_it.subscribe("ardrone/front/image_raw",100,&uavControl::detectionImageCb,this);

    // drone information subscriber
    _infoWithUavSub=_nh.subscribe("ardrone/navdata",100,&uavControl::infoWithUavCb,this);;

    // drone publisher
    _takeOffPub=_nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
    _landPub=_nh.advertise<std_msgs::Empty>("ardrone/land",1);
    _resetPub=_nh.advertise<std_msgs::Empty>("ardrone/reset",1);
    _uavControlPub=_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

    // object position publisher
    _objectPositionPub=_nh.advertise<geometry_msgs::Point>("objectPosition",1);
}

void uavControl::numberOfObjectsCb(const std_msgs::Int8::ConstPtr msg){
    _numberOfObjects=msg->data;
}

void uavControl::boundingBoxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr msg){
    _boundingBoxes.clear();
    for(unsigned int i=0; i<static_cast<unsigned int>(_numberOfObjects); ++i){
        _boundingBoxes.insert(_boundingBoxes.begin() + i,{msg->bounding_boxes[i].Class,
                                                          msg->bounding_boxes[i].probability,
                                                          msg->bounding_boxes[i].xmin,
                                                          msg->bounding_boxes[i].ymin,
                                                          msg->bounding_boxes[i].xmax,
                                                          msg->bounding_boxes[i].ymax} );
    }

}

void uavControl::detectionImageCb(const sensor_msgs::ImageConstPtr& msg){
    try {
        _detectionImage=cv_bridge::toCvCopy(msg,"bgr8")->image;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void uavControl::objectColorCb(const control::ColorMsg::ConstPtr msg){
    _detectColor=msg->detectColor;
}

void uavControl::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr msg){
    _cameraInfo.width=msg->width;
    _cameraInfo.height=msg->height;
    _cameraInfo.fx=msg->K[0];
    _cameraInfo.fy=msg->K[4];
    _cameraInfo.cx=msg->K[2];
    _cameraInfo.cy=msg->K[5];
}

void uavControl::infoWithUavCb(const ardrone_autonomy::Navdata::ConstPtr msg){
    _batteryPercent=msg->batteryPercent;
    _state=msg->state;
    _altitude=msg->altd;
}

void uavControl::keyboardCb(const keyboard::Key::ConstPtr msg){
    _keyDown=msg->code;
}
//gettery
int8_t uavControl::getNumberOfObjects(){
    return  _numberOfObjects;
}
std::vector<boundingBox> uavControl::getBoundingBox(){
    return _boundingBoxes;
}
std::vector<std::string> uavControl::getObjectColor(){
    return _detectColor;
}

cv::Mat uavControl::getDetectionImage(){
    return _detectionImage;
}

float uavControl::getBatteryPercent(){
    return _batteryPercent;
}

uint32_t uavControl::getState(){
    return _state;
}

int32_t uavControl::getAltitude(){
    return _altitude;
}

int uavControl::getKeyDown(){
    return _keyDown;
}

unsigned int uavControl::getCameraWidth(){
   return _cameraInfo.width;
}

unsigned int uavControl::getCameraHeight(){
    return _cameraInfo.height;
}

//publisher
void uavControl::takeOff(){
    _takeOffPub.publish(_emp_msg);
    ROS_INFO("ARdrone take off");

}

void uavControl::land(){
    _landPub.publish(_emp_msg);
    ROS_INFO("ARdrone land");
}

void uavControl::reset(){
    _resetPub.publish(_emp_msg);
    ROS_INFO("ARdrone reset");
}

void uavControl::flyTo(double x, double y, double z, double turn){
    msg_vel.angular.x=0;
    msg_vel.angular.y=0;
    msg_vel.angular.z=turn;
    msg_vel.linear.x=x;
    msg_vel.linear.y=y;
    msg_vel.linear.z=z;

    ROS_INFO("ARdrone go to position");
    _uavControlPub.publish(msg_vel);
}

void uavControl::sendObjectPosition(double D, double x, double yaw){
    _objectPos.x=D;
    _objectPos.y=x;
    _objectPos.z=yaw;

    _objectPositionPub.publish(_objectPos);
}

//function to calculate information from camera
double uavControl::calculateAngle(boundingBox bbox){
    return atan((0.5*(bbox.xmax-bbox.xmin)+bbox.xmin-_cameraInfo.cx)/_cameraInfo.fx);
}

double uavControl::calculateDistanceWFromCamera(boundingBox bbox, double realObjectWidth){
    return (_cameraInfo.fx*realObjectWidth)/(bbox.xmax-bbox.xmin);
}

double uavControl::calculateDistanceHFromCamera(boundingBox bbox, double realObjectHeight){
    return (_cameraInfo.fy*realObjectHeight)/(bbox.ymax-bbox.ymin);
}

double uavControl::calculateXpossition(boundingBox bbox,  double distanceFromCamera){
    return (0.5*(bbox.xmax-bbox.xmin)+bbox.xmin-_cameraInfo.cx)*(distanceFromCamera/_cameraInfo.fx);
}

double uavControl::calculateYpossition(boundingBox bbox,  double distanceFromCamera){
    return (0.5*(bbox.ymax-bbox.ymin)+bbox.ymin-_cameraInfo.cy)*(distanceFromCamera/_cameraInfo.fy);
}

// filtered data about object position
std::vector<double> uavControl::filterData(std::vector<double> objectPos, unsigned int numberOfSamples, const double DistanceFromObject){
    std::vector <double> dataAfterFiltration={0,0,0,0};
    double sum_D=0, sum_x=0, sum_y=0, sum_fi=0;

    _filteredData.push_back(objectPos);

    while (_filteredData.size()<numberOfSamples) _filteredData.push_back({DistanceFromObject,0,0,0});

    if (_filteredData.size()>numberOfSamples)  _filteredData.erase(_filteredData.begin());

    for (auto &i:_filteredData){
        sum_D+=i[0];
        sum_x+=i[1];
        sum_y+=i[2];
        sum_fi+=i[3];
    }
    dataAfterFiltration[0]=sum_D/_filteredData.size();
    dataAfterFiltration[1]=sum_x/_filteredData.size();
    dataAfterFiltration[2]=sum_y/_filteredData.size();
    dataAfterFiltration[3]=sum_fi/_filteredData.size();

    return dataAfterFiltration;
}
