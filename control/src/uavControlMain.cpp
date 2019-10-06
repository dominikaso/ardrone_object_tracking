#include "../include/uavControl.hpp"

const std::string DefinedClass="person";
const std::string DefinedColor="red";
const double ObjectRealWidth=500;
const double ObjectRealHeight=1820;
const double Velocity=0.05;

const int START_KEY=277;
const int STOP_KEY=19;

const double DistanceFromObject=3000.0; // desirable distance in meters

uint32_t missionState=0;


void mission(uavControl control,std::vector<double> uavVel);
std::vector<double> camera(uavControl control);
std::vector<double> calculateUavVel(uavControl control);

int main(int argc, char **argv)
{
    std::vector<double> uavVel={0,0,0,0};

    ros::init(argc, argv, "uavControl");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);

    uavControl control(n);

    sleep(5);

    cv::namedWindow("detekcja", CV_WINDOW_AUTOSIZE);
    cv::startWindowThread();


    while (ros::ok())
    {
        uavVel=calculateUavVel(control);

        mission(control,uavVel);

        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyWindow("detekcja");
    return 0;
}




void mission(uavControl control,std::vector<double> uavVel){
    int pressedKey=control.getKeyDown();
    uint32_t uavState=control.getState();

    switch(missionState){
        case WAIT_FOR_START:
            if (uavState==0) control.reset();
            if (pressedKey==START_KEY && uavState==2) missionState=TAKE_OFF;
        break;
        case TAKE_OFF:
            control.takeOff();
            if (uavState==4) missionState=GO_TO_POSITION;
            if (pressedKey==STOP_KEY) missionState=LAND;
        break;
        case GO_TO_POSITION:
            control.flyTo(uavVel[0],uavVel[1],0,0);
            if (pressedKey==STOP_KEY) missionState=LAND;
        break;
        case LAND:
            control.land();
            if (uavState==2) missionState=WAIT_FOR_START;
        break;
    }

}

std::vector<double> camera(uavControl control){
    std::vector<boundingBox> bbox;
    unsigned int num;
    std::vector<std::string> color;
    cv::Mat detectImg;
    cv::Mat rectangleImg;

    std::vector<double> objectPos={DistanceFromObject,0,0,0};

    num=static_cast<unsigned int>(control.getNumberOfObjects());

    if (num>0){
        bbox.clear();
        color.clear();

        bbox=control.getBoundingBox();
        color=control.getObjectColor();

        detectImg=control.getDetectionImage();

        for (unsigned int i=0; i<num; ++i){
            if (bbox[i].classOfObject==DefinedClass && bbox[i].probability>=0.6){
                //double object_proportion=(bbox[i].xmax-bbox[i].xmin)/(bbox[i].ymax-bbox[i].ymin);

                    if (bbox[i].ymin==0 || bbox[i].ymax==control.getCameraHeight()){
                        if (bbox[i].xmin==0) {
                            if (bbox[i].xmax==control.getCameraWidth()) {
                                // fly in backward
                                objectPos[0]=DistanceFromObject-150;
                                objectPos[1]=0;
                                objectPos[2]=0;
                                objectPos[3]=0;
                                std::cout<<"FLY IN BACKWARD"<<std::endl;
                            } else {
                                // fly in left
                                objectPos[0]=DistanceFromObject;
                                objectPos[1]=-150;
                                objectPos[2]=0;
                                objectPos[3]=-0.17; //10 degree
                                std::cout<<"FLY IN LEFT"<<std::endl;
                            }
                        } else {
                            if (bbox[i].xmax==control.getCameraWidth()) {
                                // fly in right
                                objectPos[0]=DistanceFromObject;
                                objectPos[1]=150;
                                objectPos[2]=0;
                                objectPos[3]=0.17; //10 degree
                                std::cout<<"FLY IN RIGHT"<<std::endl;
                            } else {
                                //if (object_proportion>=(0.8*ObjectRealWidth/ObjectRealHeight)
                                  //     && object_proportion<=(1.2*ObjectRealWidth/ObjectRealHeight)) {
                                    // calculate distance with width
                                    objectPos[0]=control.calculateDistanceWFromCamera(bbox[i],ObjectRealWidth);
                                    objectPos[1]=control.calculateXpossition(bbox[i], objectPos[0]);
                                    objectPos[2]=control.calculateYpossition(bbox[i], objectPos[0]);
                                    objectPos[3]=control.calculateAngle(bbox[i]);

                                //}
                            }
                        }
                    }
                    else {
                        //if (object_proportion>=(0.8*ObjectRealWidth/ObjectRealHeight)
                          //      && object_proportion<=(1.2*ObjectRealWidth/ObjectRealHeight)) {

                            // calculate distance with height
                            objectPos[0]=control.calculateDistanceHFromCamera(bbox[i],ObjectRealHeight);
                            objectPos[1]=control.calculateXpossition(bbox[i], objectPos[0]);
                            objectPos[2]=control.calculateYpossition(bbox[i], objectPos[0]);
                            objectPos[3]=control.calculateAngle(bbox[i]);
                        //}
                    }

                    cv::Rect rect(static_cast<int>(bbox[i].xmin), static_cast<int>(bbox[i].ymin),
                                 static_cast<int>(bbox[i].xmax-bbox[i].xmin), static_cast<int>(bbox[i].ymax-bbox[i].ymin));
                    cv::rectangle(detectImg,rect,cv::Scalar(0,255,0),3,8,0);
                    cv::imshow("detekcja", detectImg);
                    cv::waitKey(1);
                //}

                break;
            }
        }
     }
    objectPos=control.filterData(objectPos,5,DistanceFromObject);
    control.sendObjectPosition(objectPos[0],objectPos[1],objectPos[3]);
    std::cout<<objectPos[0] <<" ;" <<objectPos[1] <<" ;"<<objectPos[2] << " ;" << objectPos[3] << std::endl;
    return objectPos;
}

std::vector<double> calculateUavVel(uavControl control){
    std::vector<double> objectPos;
    std::vector<double> uavVelocity={0,0,0,0};

    objectPos=camera(control);

    if ((objectPos[0]-DistanceFromObject)>75) uavVelocity[0]=Velocity; //+x - move forward
    else if ((objectPos[0]-DistanceFromObject)<-75) uavVelocity[0]=-Velocity; //-x - move backward

    if (objectPos[1]>75) uavVelocity[1]=-Velocity; // -y - move right
    else if (objectPos[1]<-75) uavVelocity[1]=Velocity; // +y - move left

    if (objectPos[2]>75) uavVelocity[2]=Velocity; //+z - move up
    else if (objectPos[2]<-75) uavVelocity[2]=-Velocity; //-z - move down

    if (objectPos[3]>0.01) uavVelocity[3]=-Velocity; //yaw - turn right
    else if (objectPos[3]<-0.01) uavVelocity[3]=Velocity; //-yaw - turn left
    return uavVelocity;
}
