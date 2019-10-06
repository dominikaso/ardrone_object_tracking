#ifndef LOGGER_H
#define LOGGER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <fstream>
#include <string>
#include "datatosave.hpp"


class logger {
public:
    logger(ros::NodeHandle &n, std::string &fileName);
    virtual ~logger();

    dataToSave getDataToSave();
    void saveData(const dataToSave &data);
private:
    ros::NodeHandle _nh;

    void init();

    ros::Subscriber _uavOdometrySub;
    ros::Subscriber _objectPositionSub;

    void _uavOdometryCb(const nav_msgs::Odometry::ConstPtr& msg);
    void _objectPositionCb(const geometry_msgs::Point::ConstPtr& msg);

    std::ofstream _file;
    dataToSave _data;

    friend std::ostream& operator << (std::ostream& stream, const dataToSave& data);
};


#endif // LOGGER_H
