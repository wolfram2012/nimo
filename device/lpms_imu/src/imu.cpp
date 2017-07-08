#include <cstdio>
#ifdef _WIN32
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#endif
#ifdef __GNUC__
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#endif
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

sensor_msgs::Imu Imusensor_data;
ros::Publisher imudata_pub;

int main(int argc, char** argv)
{
    ImuData d;
    ros::init(argc,argv,"imu_data");
    ros::NodeHandle n;
    imudata_pub=n.advertise<sensor_msgs::Imu>("imu_sensor",100);    

    LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
    LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_U2, "/dev/ttyUSB0");
   
     ros::Rate loop(400);
     // char a[50];
     // // lpms->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);
     // while(1)
     // {
     //    if(lpms->setConfigurationPrm(PRM_SAMPLING_RATE, 200) != true
     //        && lpms->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_115200) != true
     //        && lpms->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_125KBPS) != true)
     //        ;
     //    else
     //        break;
     // }
    // ROS_INFO("result: %s",a);

     ros::Duration(5).sleep();
     while(ros::ok())
    {		 
        
        if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
            lpms->hasImuData()) 
        {            
            d = lpms->getCurrentData();
            Imusensor_data.header.stamp=ros::Time::now();
            Imusensor_data.header.frame_id="Imu_sensor_data";
            Imusensor_data.orientation.x=d.q[0];
	       Imusensor_data.orientation.y=d.q[1];
            Imusensor_data.orientation.z=d.q[2];
            Imusensor_data.orientation.w=d.q[3];
            Imusensor_data.angular_velocity.x=d.g[0];
            Imusensor_data.angular_velocity.y=d.g[1];
            Imusensor_data.angular_velocity.z=d.g[2];
            Imusensor_data.linear_acceleration.x=d.a[0];
            Imusensor_data.linear_acceleration.y=d.a[1];
            Imusensor_data.linear_acceleration.z=d.a[2];
            Imusensor_data.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            Imusensor_data.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            Imusensor_data.linear_acceleration_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            imudata_pub.publish(Imusensor_data);

         }
        loop.sleep();
    }

   manager->removeSensor(lpms);
    delete manager;
    return 0;
}

