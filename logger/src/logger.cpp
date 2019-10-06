#include "../include/logger.hpp"

logger::logger(ros::NodeHandle &n,std::string &fileName): _nh(n), _file(fileName) {
    init();
}

logger::~logger(){
    _file.close();
}

void logger::init(){
    _uavOdometrySub=_nh.subscribe("ardrone/odometry",100,&logger::_uavOdometryCb, this);
    _objectPositionSub=_nh.subscribe("objectPosition",100,&logger::_objectPositionCb, this);
}

void logger::_uavOdometryCb(const nav_msgs::Odometry::ConstPtr& msg){
    _data.x=msg->pose.pose.position.x;
    _data.y=msg->pose.pose.position.y;
    _data.z=msg->pose.pose.position.z;
    _data.roll=msg->pose.pose.orientation.x;
    _data.pitch=msg->pose.pose.orientation.y;
    _data.yaw=msg->pose.pose.orientation.z;
}

void  logger::_objectPositionCb(const geometry_msgs::Point::ConstPtr& msg){
    _data.D=msg->x;
    _data.xo=msg->y;
    _data.fi=msg->z;
}

dataToSave logger::getDataToSave(){
    return _data;
}

std::ostream& operator << (std::ostream& stream, const dataToSave& data){
    stream << data.x << '\t' << data.y << '\t' << data.z << '\t' << data.roll << '\t' << data.pitch << '\t' << data.yaw
           << '\t' << data.D << '\t' << data.xo << '\t' << data.fi << std::endl;
    return stream;
}

void logger::saveData(const dataToSave &data){
    if (_file.is_open())  _file << data;
}
