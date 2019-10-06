#include "ros/ros.h"
#include "../include/checkColor.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "objectColor");
    ros::NodeHandle n;

    checkColor check(n);

    ros::Rate loop_rate(30);


    std::vector<cv::Mat> objectImg;
    cv::Mat newImg;
    std::vector<boundingBox> bbox;
    cv::Mat detectImg;
    unsigned int num;
    std::string color;

    objectColor::ColorMsg msg;

    while (ros::ok())
    {
        msg.detectColor.clear();
        objectImg.clear();

        num=static_cast<unsigned int>(check.getNumberOfObjects());

        if(num>0){
            bbox=check.getBoundingBox();
            detectImg=check.getDetectionImage();
            objectImg=check.cropToObject(detectImg,bbox);

            for(auto &it:objectImg){
                color=check.findDominantColor(it);
                msg.detectColor.push_back(color);
            }
        }

        check.publishColor(msg);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
