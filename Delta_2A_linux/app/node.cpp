/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/

#include <iostream>
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)


typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;

using namespace std;
using namespace everest::hwdrivers;


void mymin(double *mn, double dis) {
    for (int i = 0; i < 3; ++i)
        if (dis < mn[i]) {
            for (int j = i; j < 2; ++j) mn[j+1] = mn[j];
            mn[i] = dis;
            break;
        }
}

int main(int argc, char * argv[])
{
	int    opt_com_baudrate = 230400;
    string opt_com_path = "/dev/ttyUSB0";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serail port sucessful!\n");
        printf("baud rate:%d\n",serial_connect.getBaud());
    }
    else
    {
        printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path.c_str());
        return -1;
    }

    printf("C3iroboticslidar connected\n");

    robotics_lidar.initilize(&serial_connect);


    while (1)
    {
		TLidarGrabResult result = robotics_lidar.getScanData();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
                TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();
                std::vector<RslidarDataComplete> send_lidar_scan_data;
                send_lidar_scan_data.resize(lidar_scan_size);
                RslidarDataComplete one_lidar_data;
                for(size_t i = 0; i < lidar_scan_size; i++)
                {
                    one_lidar_data.signal = lidar_scan.signal[i];
                    one_lidar_data.angle = lidar_scan.angle[i];
                    one_lidar_data.distance = lidar_scan.distance[i];
                    send_lidar_scan_data[i] = one_lidar_data;
                }

                // printf("Lidar count %d!\n", lidar_scan_size);
                double mn0[3], mn1[3], mn2[3], mn3[3], mn4[3];
                for (int i = 0; i < 3; ++i) mn0[i] = mn1[i] = mn2[i] = mn3[i] = mn4[i] = 100.0;
                for (int i = 0; i<lidar_scan_size; i++) if (lidar_scan.distance[i]!=0) {
                    double dis = lidar_scan.distance[i], angle = lidar_scan.angle[i];
                    if (angle>315 || angle<30) mymin(mn0, dis);
                    else if (angle>30 && angle<90) mymin(mn1, dis);
                    else if (angle>90 && angle<150) mymin(mn2, dis);
                    else if (angle>150 && angle<225) mymin(mn3, dis);
                    else mymin(mn4, dis);
                }
                double res0 = (mn0[0]+mn0[1]+mn0[2])/3.0;
                double res1 = (mn1[0]+mn1[1]+mn1[2])/3.0;
                double res2 = (mn2[0]+mn2[1]+mn2[2])/3.0;
                double res3 = (mn3[0]+mn3[1]+mn3[2])/3.0;
                double res4 = (mn4[0]+mn4[1]+mn4[2])/3.0;
                cout<<"sec0_"<<res0<<"_sec1_"<<res1<<"_sec2_"<<res2<<"_sec3_"<<res3<<"_sec4_"<<res4<<endl;
                // cout<<lidar_scan_size<<endl;

                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        //usleep(50);
    }

    return 0;
}
