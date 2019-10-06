#include "../include/checkColor.hpp"


checkColor::checkColor(ros::NodeHandle &n): _nh(n), _it(_nh)
{
    init();
}

checkColor::~checkColor(){
}

void checkColor::init(){
    _numberOfObjectSub=_nh.subscribe("darknet_ros/found_object",100, &checkColor::numberOfObjectsCb,this);
    _boundingBoxSub=_nh.subscribe("darknet_ros/bounding_boxes",100,&checkColor::boundingBoxCb,this);
    _detectionImageSub=_it.subscribe("ardrone/front/image_raw",100,&checkColor::detectionImageCb,this);

    _colorPub=_nh.advertise<objectColor::ColorMsg>("checkColor/detectColor",100);
}

void checkColor::numberOfObjectsCb(const std_msgs::Int8::ConstPtr msg){
    _numberOfObjects=msg->data;
}

void checkColor::boundingBoxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr msg){
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

void checkColor::detectionImageCb(const sensor_msgs::ImageConstPtr& msg){
    try {
        _detectionImage=cv_bridge::toCvCopy(msg,"bgr8")->image;

    }
    catch (cv_bridge::Exception&)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

std::vector<cv::Mat> checkColor::cropToObject(cv::Mat& img, std::vector<boundingBox>& bbox){
    std::vector<cv::Mat> objectImg;

    for(unsigned int i=0; i<bbox.size(); ++i)
    {
        cv::Rect roi;

        roi.x=static_cast<int>(bbox[i].xmin);
        roi.y=static_cast<int>(bbox[i].ymin);
        roi.width=static_cast<int>(bbox[i].xmax-bbox[i].xmin);
        roi.height=static_cast<int>(bbox[i].ymax-bbox[i].ymin);

        cv::Mat ROI(img,roi);

        objectImg.push_back(ROI);
    }

    return objectImg;
}

std::string checkColor::findDominantColor(cv::Mat& img){

    cv::Mat hsv_img;
    cvtColor(img,hsv_img,CV_BGR2HSV);

    std::vector<cv::Mat> hsv_channels;

    //seperate the image in 3 channels (H,S,V)
    split(hsv_img, hsv_channels);

    cv::Mat h_hist, s_hist, v_hist;
    // histogram parametrs
    int h_bins=36;  int sv_bins=8; // quantize the hue to 30lvl and the saturation to 32lvl
    int histSize[]={h_bins,sv_bins};
    float h_range[]={0, 180};   float sv_range[]={0, 256};
    const float* range[]={h_range,sv_range};
    bool uniform=true;
    bool accumulate=false;
    int channels[]={0};

    // compute the histograms
    calcHist(&hsv_channels[0], 1, channels, cv::Mat(), h_hist, 1, &histSize[0], &range[0], uniform, accumulate);
    calcHist(&hsv_channels[1], 1, channels, cv::Mat(), s_hist, 1, &histSize[1], &range[1], uniform, accumulate);
    calcHist(&hsv_channels[2], 1, channels, cv::Mat(), v_hist, 1, &histSize[1], &range[1], uniform, accumulate);

    //calculate max value of histograms
    std::vector <double> maxVal={0,0,0};
    std::vector<cv::Point> maxPoint={{0,0},{0,0},{0,0}};
    std::vector<int> channel={0,0,0};

    minMaxLoc(h_hist, NULL, &maxVal[0], NULL, &maxPoint[0]);
    minMaxLoc(s_hist, NULL, &maxVal[1], NULL, &maxPoint[1]);
    minMaxLoc(v_hist, NULL, &maxVal[2], NULL, &maxPoint[2]);

    channel[0]=maxPoint[0].y;
    channel[1]=maxPoint[1].y;
    channel[2]=maxPoint[2].y;

    double sizeObjectProcent=0.5; // the percentage ratio of a colored object to a slice of the image

    int sizeObject=static_cast<int>(sizeObjectProcent*img.cols*img.rows);

    return _adjustColor(channel, maxVal, sizeObject);
}

std::string checkColor::_adjustColor(std::vector<int> channel, std::vector<double> maxVal, int sizeObject){

    std::string color="";


    if (channel[2]<=1 && maxVal[2]>sizeObject) color="black";
    else if (channel[1]<=1 && maxVal[1]>sizeObject)
    {
        if (channel[2]>=6 && maxVal[2]>sizeObject) color="white";
        else color="gray";

    }
    else
    {
        switch(channel[0]){
        case 33: case 34: case 35:  case 0: case 1:
            color="red";
            break;
        case 2: case 3:
            color="orange";
            break;
        case 4: case 5: case 6: case 7:
            color="yellow";
            break;
        case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15:
            color="green";
            break;
        case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25:
            color="blue";
            break;
        case 26: case 27: case 28:
            color="violet";
            break;
        case 29: case 30: case 31: case 32:
            color="pink";
        }
    }

    return color;
}

void checkColor::publishColor(objectColor::ColorMsg color){
    _colorPub.publish(color);
}

//gettery
int8_t checkColor::getNumberOfObjects(){
    return  _numberOfObjects;
}
std::vector<boundingBox> checkColor::getBoundingBox(){
    return _boundingBoxes;
}
cv::Mat checkColor::getDetectionImage(){
    return _detectionImage;
}

