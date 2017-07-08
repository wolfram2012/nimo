#ifndef COMS_FILTER_H
#define COMS_FILTER_H

#include <stdio.h> 
#include "ros/ros.h"

#include "coms_msgs/CanMessage.h"
#include "coms_msgs/CanMessageStamped.h"
#include "coms_msgs/VehicleMessageStamp.h"
#include "coms_msgs/ComsCommand.h"
#include "coms_msgs/LWimuID70.h"
#include "coms_msgs/LWimuID71.h"
#include "coms_msgs/LWimuID72.h"
#include "coms_msgs/LWimuID75.h"
#include "coms_msgs/LWimuID76.h"
#include "coms_msgs/LWimuID78.h"
#include "coms_msgs/LWimuID79.h"

#include "coms_protocol.h"


namespace coms
{
    	
    struct coms_msg
    {
	ros::Publisher coms_vehicle_msg;
    };

    struct LWimu_msg
    {
	ros::Publisher LWimu_ID70;
	ros::Publisher LWimu_ID71;
	ros::Publisher LWimu_ID72;
	ros::Publisher LWimu_ID75;
	ros::Publisher LWimu_ID76;
	ros::Publisher LWimu_ID78;
	ros::Publisher LWimu_ID79;
    };
    
    struct pub_set
    {
	struct coms_msg coms_msg;
	struct LWimu_msg LWimu_msg;
    } canpub_set;

    ros::Publisher pub_cmd;

    //Message from Vehicle
    void vehicleMessage(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);

    //Message from LWimu
    void LwImuID70(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);
    void LwImuID71(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);
    void LwImuID72(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);
    void LwImuID75(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);
    void LwImuID76(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);
    void LwImuID78(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);
    void LwImuID79(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame);

    //CAN-Filter
    // void ros_publish(struct pub_set set, coms_msgs::CanMessageStamped::ConstPtr &can_frame);
}


#endif
