#include "coms_filter.h"

namespace coms
{

    /*Message from Vehicle*/

    //coms: steering_angle and speed
    void vehicleMessage(ros::Publisher pub_handle, coms_msgs::CanMessage *msg)
    {
    	const ComsReport *ptr = (const ComsReport*)msg->data.elems;
  		float speed = ((float)ptr->cur_speed_h*256 + ptr->cur_speed_l) / 65536.0 * 10;
  		float steering = ((float)ptr->cur_steering_h*256 + ptr->cur_steering_l) / 65535.0 * 70 - 35;

  		// ROS_INFO("test1 = %d",ptr->test1);
  		coms_msgs::VehicleMessageStamp data;
  		data.header.frame_id = "base_link";
  		data.header.stamp = ros::Time::now();
  		data.speed = speed;
  		data.steering = steering;
  
		pub_handle.publish(data);
    }

    /*Message from Vehicle END*/



    /*Message from LW_IMU*/

    //ID70: AngleX, AngleY, AngleZ, T
    void LwImuID70(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {

		coms_msgs::LWimuID70 data;
		//TODO
		if (can_frame->data[0]>127)
		{
			can_frame->data[0]=(double)can_frame->data[0]-128;
			data.anglex = (double)can_frame->data[0]*256+(double)can_frame->data[1];
			data.anglex =(data.anglex-32768)/100;
		}
		else 
			data.anglex = ((double)can_frame->data[0]*256+(double)can_frame->data[1])/100;
	
		if (can_frame->data[2]>127)
		{
			can_frame->data[2]=(double)can_frame->data[2]-128;
			data.angley = (double)can_frame->data[2]*256+(double)can_frame->data[3];
			data.angley =(data.angley-32768)/100;
		}
		else 
			data.angley = ((double)can_frame->data[2]*256+(double)can_frame->data[3])/100;
	
		if (can_frame->data[4]>127)
		{
			can_frame->data[4]=(double)can_frame->data[4]-128;
			data.anglez = (double)can_frame->data[4]*256+(double)can_frame->data[5];
			data.anglez =(data.anglez-32768)/100;
		}
		else 
			data.anglez = ((double)can_frame->data[4]*256+(double)can_frame->data[5])/100;
		
		if (can_frame->data[6]>127)
		{
			can_frame->data[6]=(double)can_frame->data[6]-128;
			data.T = (double)can_frame->data[6]*256+(double)can_frame->data[7];
			data.T =-(data.T-32768)/100;
		}
		else 
			data.T = ((double)can_frame->data[6]*256+(double)can_frame->data[7])/100;
		//TODO END

		pub_handle.publish(data);
	}

    //ID71: AccelX, AccelY, AccelZ
    void LwImuID71(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {
		coms_msgs::LWimuID71 data;
		//TODO
		if (can_frame->data[0]>127)
		{
			can_frame->data[0]=(double)can_frame->data[0]-128;
			data.accelx = (double)can_frame->data[0]*256+(double)can_frame->data[1];
			data.accelx =(data.accelx-32768)/100;
		}
		else 
			data.accelx = ((double)can_frame->data[0]*256+(double)can_frame->data[1])/100;
		
		if (can_frame->data[2]>127)
		{
			can_frame->data[2]=(double)can_frame->data[2]-128;
			data.accely = (double)can_frame->data[2]*256+(double)can_frame->data[3];
			data.accely=(data.accely-32768)/100;
		}
		else 
			data.accely = ((double)can_frame->data[2]*256+(double)can_frame->data[3])/100;
		
		if (can_frame->data[4]>127)
		{
			can_frame->data[4]=(double)can_frame->data[4]-128;
			data.accelz = (double)can_frame->data[4]*256+(double)can_frame->data[5];
			data.accelz =(data.accelz-32768)/100;
		}
		else 
			data.accelz = ((double)can_frame->data[4]*256+(double)can_frame->data[5])/100;
		//TODO END

		pub_handle.publish(data);
    }

    //ID72: CompassX, CompassY, CompassZ
    void LwImuID72(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {
		coms_msgs::LWimuID72 data;
		//TODO
		if (can_frame->data[0]>127)
		{
			can_frame->data[0]=(double)can_frame->data[0]-128;
			data.compassx = (double)can_frame->data[0]*256+(double)can_frame->data[1];
			data.compassx =(data.compassx-32768)/100;
		}
		else 
			data.compassx = ((double)can_frame->data[0]*256+(double)can_frame->data[1])/100;
		
		if (can_frame->data[2]>127)
		{
			can_frame->data[2]=(double)can_frame->data[2]-128;
			data.compassy = (double)can_frame->data[2]*256+(double)can_frame->data[3];
			data.compassy=(data.compassy-32768)/100;
		}
		else 
			data.compassy = ((double)can_frame->data[2]*256+(double)can_frame->data[3])/100;
		
		if (can_frame->data[4]>127)
		{
			can_frame->data[4]=(double)can_frame->data[4]-128;
			data.compassz = (double)can_frame->data[4]*256+(double)can_frame->data[5];
			data.compassz =(data.compassz-32768)/100;
		}
		else 
			data.compassz = ((double)can_frame->data[4]*256+(double)can_frame->data[5])/100;
		//TODO END

		pub_handle.publish(data);
    }

    //ID75: GPSmode, StarNum, HDOP, High, UTC:h/m/s/ms
    void LwImuID75(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {
		coms_msgs::LWimuID75 data;
		//TODO
		data.GPSmode = (int)can_frame->data[0];
		data.starNum = (int)can_frame->data[1];
		data.HDOP = (int)can_frame->data[2];
		data.hight =(int)can_frame->data[3];
		data.UTCh = (int)can_frame->data[4];
		data.UTCm = (int)can_frame->data[5];
		data.UTCs = (int)can_frame->data[6];
		data.UTCms = (int)can_frame->data[7];
		//TODO END

		pub_handle.publish(data);
    }

    //ID76: GPSL, GPSR
    void LwImuID76(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {
		coms_msgs::LWimuID76 data;
		//TODO
		
		//TODO END
		pub_handle.publish(data);
    }

    //ID78: Roll, Pitch, Yaw, PZ
    void LwImuID78(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {
		coms_msgs::LWimuID78 data;
		//TODO
		if (can_frame->data[0]>127)
		{
			can_frame->data[0]=(double)can_frame->data[0]-128;
			data.roll = (double)can_frame->data[0]*256+(double)can_frame->data[1];
			data.roll =(data.roll-32768)/100;
		}
		else 
			data.roll = ((double)can_frame->data[0]*256+(double)can_frame->data[1])/100;
		
		if (can_frame->data[2]>127)
		{
			can_frame->data[2]=(double)can_frame->data[2]-128;
			data.pitch = (double)can_frame->data[2]*256+(double)can_frame->data[3];
			data.pitch =(data.pitch-32768)/100;
		}
		else 
			data.pitch = ((double)can_frame->data[2]*256+(double)can_frame->data[3])/100;
		
		if (can_frame->data[4]>127)
		{
			can_frame->data[4]=(double)can_frame->data[4]-128;
			data.yaw = (double)can_frame->data[4]*256+(double)can_frame->data[5];
			data.yaw =(data.yaw-32768)/100;
		}
		else 
			data.yaw = ((double)can_frame->data[4]*256+(double)can_frame->data[5])/100;
		
		if (can_frame->data[6]>127)
		{
			can_frame->data[6]=(double)can_frame->data[6]-128;
			data.pz = (double)can_frame->data[6]*256+(double)can_frame->data[7];
			data.pz =(data.pz-32768)/100;}
		else 
			data.pz = ((double)can_frame->data[6]*256+(double)can_frame->data[7])/100;
		//TODO END

		pub_handle.publish(data);
    }

    //ID79: PositionX, PositionY
    void LwImuID79(ros::Publisher pub_handle, coms_msgs::CanMessage *can_frame)
    {
		coms_msgs::LWimuID79 data;
		//TODO
		data.positionx = 0;
		data.positiony = 0;
		//TODO END

	//pub_handle.publish(data);
    }
    /*Message from LW_IMU END*/


 //    /*CAN-Filter*/
//      void ros_publish(struct pub_set set, const coms_msgs::CanMessageStamped::ConstPtr &data)
//     {
//     	coms_msgs::CanMessage can_frame;
// 		for(int i=0; i<7; i++) 
// 			can_frame.data[i] = data->msg.data[i];
// 		can_frame.id = data->msg.id;
// 		can_frame.dlc = data->msg.dlc;;
// 		can_frame.extended = data->msg.extended;

// 		switch (data->msg.id) 
// 		{
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		vehicleMessage(set.coms_msg.coms_vehicle_msg,data);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID70,can_frame);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID71,can_frame);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID72,can_frame);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID75,can_frame);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID76,can_frame);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID78,can_frame);
// 	    		break;
// 	    	case comsfilter::ID_COMS_REORT:
// 	    		LwImuID70(set.LWimu_msg.LWimu_ID79,can_frame);
// 	    		break;
// 	    	default:
// 	      		ROS_WARN("Unknowed CAN ID: %d . Ignore",data->msg.id);
//     }
//     /*CAN-Filter END*/

}
