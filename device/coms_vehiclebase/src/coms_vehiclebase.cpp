#include "ros/ros.h"
#include "coms_filter.h"
#include "coms_protocol.h"


void CanFilterCallback(const coms_msgs::CanMessageStamped::ConstPtr &data)
{
	coms_msgs::CanMessage *can_frame;
	for(int i=0; i<7; i++) 
		can_frame->data[i] = data->msg.data[i];
	can_frame->id = data->msg.id;
	can_frame->dlc = data->msg.dlc;;
	can_frame->extended = data->msg.extended;

	switch (data->msg.id) 
	{
    	case coms::ID_COMS_REORT:
    		coms::vehicleMessage(coms::canpub_set.coms_msg.coms_vehicle_msg,can_frame);
    		break;
    	case coms::ID_LW_IMU70_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID70,can_frame);
    		break;
    	case coms::ID_LW_IMU71_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID71,can_frame);
    		break;
    	case coms::ID_LW_IMU72_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID72,can_frame);
    		break;
    	case coms::ID_LW_IMU75_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID75,can_frame);
    		break;
    	case coms::ID_LW_IMU76_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID76,can_frame);
    		break;
    	case coms::ID_LW_IMU78_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID78,can_frame);
    		break;
    	case coms::ID_LW_IMU79_REPORT:
    		coms::LwImuID70(coms::canpub_set.LWimu_msg.LWimu_ID79,can_frame);
    		break;
    	default:
      		ROS_WARN("Unknowed CAN ID: %d . Ignore",data->msg.id);
  	}
}

void ComsCommandCallback(const coms_msgs::ComsCommand::ConstPtr &data)
{
	coms_msgs::CanMessage out;
	out.id = coms::ID_COMS_CMD;
	out.extended = true;
	out.dlc = sizeof(coms::ComsCmd); 

	coms::ComsCmd *ptr = (coms::ComsCmd*)out.data.elems;
	memset(ptr, 0x00, sizeof(*ptr));

	ptr->steering_cmd =  (uint8_t)((data->steering+1)*255);
	ptr->throttle_cmd =  (uint8_t)(data->throttle*255);
	ptr->brake_cmd    =  (uint8_t)(data->brake*255);
	ptr->gear_cmd     =  (uint8_t)(data->gear);
	ptr->lamps_cmd    =  (uint8_t)(data->lamps);

  	coms::pub_cmd.publish(out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coms_vehiclebase");
	ros::NodeHandle n;

	//can subcriber
	ros::Subscriber sub = n.subscribe("/can_rx",1000, CanFilterCallback);

	//command subcriber
	ros::Subscriber sub_cmd = n.subscribe("/coms_cmd",1000, ComsCommandCallback);
	coms::pub_cmd = n.advertise<coms_msgs::CanMessage>("/can_tx",2);

	coms_msgs::CanMessage can_frame;
	
	//repair publisher handles
	coms::canpub_set = {
								n.advertise<coms_msgs::VehicleMessageStamp>("vehicle_message",2),
								n.advertise<coms_msgs::LWimuID70>("LW_Imu/ID70",2), 
							   	n.advertise<coms_msgs::LWimuID71>("LW_Imu/ID71",2), 
							   	n.advertise<coms_msgs::LWimuID72>("LW_Imu/ID72",2), 
			   					n.advertise<coms_msgs::LWimuID75>("LW_Imu/ID75",2), 
			   					n.advertise<coms_msgs::LWimuID76>("LW_Imu/ID76",2), 
			   					n.advertise<coms_msgs::LWimuID78>("LW_Imu/ID78",2), 
			   					n.advertise<coms_msgs::LWimuID79>("LW_Imu/ID79",2) 
			   				};
	ros::spin();
}