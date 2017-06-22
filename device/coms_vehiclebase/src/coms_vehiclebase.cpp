#include "ros/ros.h"
#include "coms_filter.h"


void CanFilterCallback(const coms_msgs::CanMessageStamped& data)
{
	coms_msgs::CanMessage can_frame;
	for(int i=0; i<7; i++) can_frame.data[i] = data.msg.data[i];
	can_frame.id = data.msg.id;
	can_frame.dlc = data.msg.dlc;;
	can_frame.extended = data.msg.extended;
	comsfilter::ros_publish(comsfilter::canpub_set,&can_frame);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coms_vehiclebase");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("can_rx",1000, CanFilterCallback);
	ros::Publisher vehicle_message = n.advertise<coms_msgs::VehicleMessageStamp>("vehiclemessage",2);	
	ros::Publisher LW_imu_ID70 = n.advertise<coms_msgs::LWimuID70>("LW_Imu/ID70",2);
	ros::Publisher LW_imu_ID71 = n.advertise<coms_msgs::LWimuID71>("LW_Imu/ID71",2);
	ros::Publisher LW_imu_ID72 = n.advertise<coms_msgs::LWimuID72>("LW_Imu/ID72",2);
	ros::Publisher LW_imu_ID75 = n.advertise<coms_msgs::LWimuID75>("LW_Imu/ID75",2);
	ros::Publisher LW_imu_ID76 = n.advertise<coms_msgs::LWimuID76>("LW_Imu/ID76",2);
	ros::Publisher LW_imu_ID78 = n.advertise<coms_msgs::LWimuID78>("LW_Imu/ID78",2);
	ros::Publisher LW_imu_ID79 = n.advertise<coms_msgs::LWimuID79>("LW_Imu/ID79",2);
	coms_msgs::CanMessage can_frame;

	comsfilter::canpub_set = {
								vehicle_message,
								LW_imu_ID70, 
							   	LW_imu_ID71, 
							   	LW_imu_ID72, 
			   					LW_imu_ID75, 
			   					LW_imu_ID76, 
			   					LW_imu_ID78, 
			   					LW_imu_ID79 
			   				};
	ros::spin();
}