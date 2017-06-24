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
    	case comsfilter::ID_COMS_REORT:
    		comsfilter::vehicleMessage(comsfilter::canpub_set.coms_msg.coms_vehicle_msg,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU70_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID70,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU71_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID71,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU72_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID72,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU75_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID75,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU76_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID76,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU78_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID78,can_frame);
    		break;
    	case comsfilter::ID_LW_IMU79_REPORT:
    		comsfilter::LwImuID70(comsfilter::canpub_set.LWimu_msg.LWimu_ID79,can_frame);
    		break;
    	default:
      		ROS_WARN("Unknowed CAN ID: %d . Ignore",data->msg.id);
  	}
}

void ComsCommandCallback(const coms_msgs::ComsCommand::ConstPtr &data)
{
	coms_msgs::CanMessage out;
	out.id = comsfilter::ID_COMS_CMD;
	out.extended = true;
	out.dlc = sizeof(comsfilter::ComsCmd); 

	comsfilter::ComsCmd *ptr = (comsfilter::ComsCmd*)out.data.elems;
	memset(ptr, 0x00, sizeof(*ptr));

	ptr->steering_cmd =  (uint8_t)(data->steering);
	ptr->throttle_cmd =  (uint8_t)(data->throttle);
	ptr->brake_cmd    =  (uint8_t)(data->brake);
	ptr->gear_cmd     =  (uint8_t)(data->gear);
	ptr->lamps_cmd    =  (uint8_t)(data->lamps);

  	comsfilter::pub_cmd.publish(out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coms_vehiclebase");
	ros::NodeHandle n;

	//can subcriber
	ros::Subscriber sub = n.subscribe("/can_rx",1000, CanFilterCallback);

	//command subcriber
	ros::Subscriber sub_cmd = n.subscribe("/coms_cmd",1000, ComsCommandCallback);
	comsfilter::pub_cmd = n.advertise<coms_msgs::CanMessage>("/can_tx",2);

	coms_msgs::CanMessage can_frame;
	
	//repair publisher handles
	comsfilter::canpub_set = {
								n.advertise<coms_msgs::VehicleMessageStamp>("vehiclemessage",2),
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