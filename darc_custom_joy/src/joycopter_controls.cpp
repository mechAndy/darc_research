/* This file has been created to try and fly the quadcopter using the joystic */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <roscopter/RC.h>
#include <roscopter/APMCommand.h>

int joy_p_,joy_r_,joy_y_,joy_t_;
int minRange = 1100,
midRange = 1500,
maxRange = 1900,
thrust = minRange,
toggle[2] = {1500,1099};

int joy_a_,joy_b_,joy_lb_, joy_rb_, joy_xbox_;

bool armed = false;

roscopter::RC rc_out;


void joy_callback(const sensor_msgs::Joy& joy_msg_in) //this also scales the joystick reading
{
	joy_p_ = 2000.0 - 500.0*(1.0 + joy_msg_in.axes[1]);		//Pitch -- left stick up-down
	joy_r_ = 2000.0 - 500.0*(1.0 + joy_msg_in.axes[0]);		//Roll -- left stick left-right
	joy_y_ = 2000.0 - 500.0*(1.0 + joy_msg_in.axes[3]);		//Yaw -- right stick left-right
	//joy_t_ = 5.0*joy_msg_in.axes[4];		//Throttle -- right stick up-down
	joy_t_ = 1000.0 + 1000.0*(joy_msg_in.axes[4]*0.75);
	joy_a_ = joy_msg_in.buttons[0]; 	//arms the robot
	joy_b_ = joy_msg_in.buttons[1]; 	//disarms the robot
	joy_lb_ = joy_msg_in.buttons[4]; 	//lb and rb together shutdown it
	joy_rb_ = joy_msg_in.buttons[5];
	joy_xbox_ = toggle[ joy_msg_in.buttons[8]]; 	//toggle flight mode
}

enum Commands
{
  CMD_LAUNCH = 1,
  CMD_LAND = 2,
  CMD_ARM = 3,
  CMD_DISARM = 4,
  CMD_SET_STABILIZE = 5,
  CMD_SET_ALT_HOLD = 6,
  CMD_SET_AUTO = 7,
  CMD_SET_LOITER = 8,
  CMD_SET_LAND = 9,
  RETURN_RC_CONTROL = 10
};


void resetController(ros::Publisher rc_pub)
{
	rc_out.channel[0] = midRange;	//roll set back to midpoint	
	rc_out.channel[1] = midRange;	//pitc set back to midpoint
	rc_out.channel[2] = minRange; //thrust to min
	rc_out.channel[3] = midRange;	//yaw back to midpoint
	rc_out.channel[4] = midRange;	//setting the mode to stabelize         
	rc_pub.publish(rc_out);
	thrust = minRange;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "joycopter_controls");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Subscriber joy_sub;
	joy_sub = node.subscribe("joy",1,joy_callback);

	ros::Publisher rc_pub;
	rc_pub = node.advertise<roscopter::RC>("apm/send_rc",1);

	ros::ServiceClient client = node.serviceClient<roscopter::APMCommand>("apm/command");
	roscopter::APMCommand srv;



	float dead_zone = .15;
	
	while(ros::ok())
	{
	ros::spinOnce();

		if(joy_a_ == true && !armed)
		{
			resetController(rc_pub);
			//ROS_INFO("Arming system");
			srv.request.command = CMD_ARM;
			if(client.call(srv)){;}
			//ROS_INFO("%d",srv.response.result);
			armed = !armed;
		}
		else if(joy_b_ == true && armed)
		{
			resetController(rc_pub);
			//ROS_INFO("Disarming system");
			srv.request.command = CMD_DISARM;
			if(client.call(srv)){;}
			//ROS_INFO("%d",srv.response.result);
			armed = !armed;
		}
		else if(joy_rb_ == true && joy_lb_ == true)
		{
			resetController(rc_pub);
			ROS_ERROR("Resetting");
		}
		else
		{
			thrust = joy_t_;
			//thrust command
			/*thrust += joy_t_;
			if ( thrust > maxRange)
				thrust = maxRange;
			else if(thrust < minRange)
				thrust = minRange;*/
			rc_out.channel[0] = joy_r_;
			rc_out.channel[1] = joy_p_;
			rc_out.channel[2] = thrust;
			rc_out.channel[3] =  joy_y_;
			rc_out.channel[4] = joy_xbox_;
//			ROS_INFO("%d\t%d\t%d\t%d",joy_p_,joy_r_,joy_y_,thrust);
	}

	rc_pub.publish(rc_out);
	loop_rate.sleep();
	}
	return 0;
}


