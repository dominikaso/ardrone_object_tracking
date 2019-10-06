#include "../include/logger.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "logger");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);

    std::string fileName;

    char bufor[100];
    time_t timeData;
    time(&timeData);
    tm timeTM = * localtime(&timeData);

    strftime(bufor, sizeof(bufor), "/home/dominika/Pulpit/magisterka/pomiary/pomiary%H:%M:%S.txt",&timeTM);
    fileName=+bufor;


    logger save(n,fileName);
    dataToSave data;

    sleep(5);

    while (ros::ok())
    {
        data=save.getDataToSave();
        save.saveData(data);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
