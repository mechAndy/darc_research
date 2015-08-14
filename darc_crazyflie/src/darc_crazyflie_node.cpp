
/*
	Andrew Pamp
	DARC Lab @ University of Utah
	
	This node is meant to be a connection between the mocap and the crazyflie.
	It takes a 'new_u' twist message in and ouputs a twist message 'cmd_vel'
	
*/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

/* For the crazyflie twist message (cmd_vel):
linear.x - pitch
linear.y - roll
linear.z - thrust
angular.z - yaw

for new_u in
angular.x - roll about robot x
angular.y - pitch about robot y
angular.z - yaw rate about z
linear.z - thrust

*/

bool thrustFlag = false;
bool killSwitch = false;

int joy_back, joy_start;
//back is arm, start is disarm

geometry_msgs::Twist vel_out;
//These values come from the crazyfie_ros demo. I need to spend some time looking further into their values. 
double x_max = 30.0,
		y_max = -30.0,
		z_max = 60000.0,
		yaw_max = -200.0;
		
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
    //right_button = joy_msg_in.buttons[5];
    //joy_a = joy_msg_in.buttons[0];
    //joy_b = joy_msg_in.buttons[1];
    joy_back = joy_msg_in.buttons[6];
    joy_start = joy_msg_in.buttons[7];
    if (joy_back)
    {
        killSwitch = false;
    }
    if (joy_start)
    {
        killSwitch = true;
    }

}


bool thrustOn_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //ROS_INFO("************************************************************Turning thrust on!");
    thrustFlag = true;
    //ROS_INFO("thrustFlag = %d", thrustFlag);
    return true;
}

bool thrustOff_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //ROS_INFO("**Turning thrust off**********************************************************");
    thrustFlag = false;
    //ROS_INFO("thrustFlag = %d", thrustFlag);
    return true;
}


void twist_callback(const geometry_msgs::Twist& twist_msg_in)
{
//this will conver the input to the proper cotnrols for the crazyflie

    vel_out.linear.x  = twist_msg_in.angular.y * x_max;
    vel_out.linear.y  = twist_msg_in.angular.x * y_max;
    vel_out.angular.z = twist_msg_in.angular.z * y_max;
    if (!killSwitch) //if killSwitch is false it flies
    {
        if (thrustFlag)
        {
            vel_out.linear.z = ((1.0+twist_msg_in.linear.z) * z_max/2.0);
        }
        else 
        {
            vel_out.linear.z = twist_msg_in.linear.z*z_max;
        }
        if (vel_out.linear.z > z_max)
        {
            vel_out.linear.z = z_max;
        }
    }
    else // killswitch is true so we want zero thrust
    {
        vel_out.linear.z = 0.0;
    }
    
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "darc_crazyflie_node");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

    //node.getParam("")
	
	ros::Subscriber twist_sub;
	twist_sub = node.subscribe("new_u",1,twist_callback);
    //ros::Subscriber twist_sub;
    //twist_sub = node.subscribe("newCMD",1,twist_callback);

    ros::Subscriber joy_sub;
    joy_sub = node.subscribe("joy",1,joy_callback);
	
	ros::Publisher twist_out;
	twist_out = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
	
	
    ros::ServiceServer thrustOn_srv = node.advertiseService("thrustOn",thrustOn_callback);
	ros::ServiceServer thrustOff_srv = node.advertiseService("thrustOff",thrustOff_callback);
	
	
	while(ros::ok())
	{
		ros::spinOnce();
        
		twist_out.publish(vel_out);
		
		loop_rate.sleep();
		
	}
	return 0;	
}
