/*
	Andrew Pamp
	DARC Lab @ University of Utah
	
	This node is meant to be a connection between the mocap and the crazyflie.
	It takes a 'new_u' twist message in and ouputs a twist message 'cmd_vel'
	
*/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

//#include <sensor_msgs/Joy.h>
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

geometry_msgs::Twist vel_out;
//These values come from the crazyfie_ros demo. I need to spend some time looking further into their values. 
double x_max = 30.0,
		y_max = -30.0,
		z_max = 60000.0,
		yaw_max = -200.0;
		
bool thrust = false;
void twist_callback(const geometry_msgs::Twist& twist_msg_in){
//this will conver the input to the proper cotnrols for the crazyflie

	    vel_out.linear.x = twist_msg_in.angular.y * x_max;
	    vel_out.linear.y = twist_msg_in.angular.x * y_max;
	    vel_out.angular.z = twist_msg_in.angular.z * y_max;
	    vel_out.linear.z = ((1.0+twist_msg_in.linear.z) * z_max/2.0);
	    if (vel_out.linear.z > z_max)
	    {
	        vel_out.linear.z = z_max;
	    }
}

void thrustOn_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response & resp)
{
    thrust = true;
}

void thrustOff_callback(std_srvs::Empty &req)
{
    thrust = false;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "darc_crazyflie_node");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);
	
	ros::Subscriber twist_sub;
	twist_sub = node.subscribe("new_u",1,twist_callback);

	ros::Publisher twist_out;
	twist_out = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
	
	ros::ServiceServer thrustOn_srv = n.advertiseService("thrustOn",thrustOn_callback);
	ros::ServiceServer thrustOff_srv = node.advertiseService("thrustOff",thrustOff_callback);
	
	while(ros::ok()){
		ros::spinOnce();
        ROS_INFO("Thrust is %d", thrust);
		twist_out.publish(vel_out);
		
		loop_rate.sleep();
		
		}
	return 0;	
	}
