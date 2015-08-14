/*
this node was created to do ILC for the crazyflie. With the need
for synchronization it was moved to the darc_iterative_learn node and 
has been saved as a working reference before changes on 6/30/15
*/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <string>	
#include <math.h>
//#include <iostream>

geometry_msgs::Vector3 curr_pos, des_pos_out;
std::vector<geometry_msgs::Vector3> prevU, prevPos;
double timestep;
int timedLoop;
double rho;
bool deBug;
int joy_a, joy_b, joy_x, joy_y;

int startWaypoints;
int landing;
int startControl;
//int startItr = 0;

// Read mocap position
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
	curr_pos.x = pos_msg_in.x;
	curr_pos.y = pos_msg_in.y;
	curr_pos.z = pos_msg_in.z;
}

void joy_callback(const sensor_msgs::Joy& joy_in)
{
    joy_a = joy_in.buttons[0]; // A gets flying and starts position hold
    joy_b = joy_in.buttons[1]; // B lands and turns off
    joy_x = joy_in.buttons[2]; // X starts waypoints
    joy_y = joy_in.buttons[3]; // Y stops waypoints and holds
    
    if( (joy_a && !startControl) || (joy_b && startControl) )
    {
        startControl = !startControl;
        startWaypoints = 0;
    }
    else if( (joy_x && !startWaypoints) || (joy_y && startWaypoints) )
    {
        startWaypoints = !startWaypoints;
        //startItr = !startItr;
        //ROS_INFO("Start itr = %d",startItr);
    }
}


void fillPositionList(std::vector<geometry_msgs::Vector3>&, double radius = 0.5);
void findAverageError(std::vector<geometry_msgs::Vector3>& errList, geometry_msgs::Vector3& aveErr, int count);

int main(int argc, char** argv)
{
	ros::init(argc,argv,"ilc_waypoint_generator");
	ros::NodeHandle node;
	ros::Rate loop_rate(100);
    
    ros::Subscriber pos_sub, joy_sub;
    pos_sub = node.subscribe("current_position",1,pos_callback);
    joy_sub = node.subscribe("joy",1,joy_callback);

    

    ros::Publisher des_pos_pub;
    des_pos_pub = node.advertise<geometry_msgs::Vector3>("desired_position",1);

    ros::Publisher ilc_des_pos;
    ilc_des_pos = node.advertise<geometry_msgs::Vector3>("ilc_des_pos",1);
    
    ros::Publisher ilc_err_out;
    ilc_err_out = node.advertise<geometry_msgs::Vector3>("ilc_error",1);
	
    ros::Publisher ilc_pos_out;
    ilc_pos_out = node.advertise<geometry_msgs::Vector3>("ilc_pos",1);
    
    ros::Publisher ilc_itr_out;
    ilc_itr_out = node.advertise<std_msgs::Int16>("ilc_itr",1);
    /*
    ros::Publisher ilc_itr_start;
    ilc_itr_start = node.advertise<std_msgs::Int16>("ilc_itr_start",1);
    */
    ros::Publisher ilc_rho;
    ilc_rho = node.advertise<std_msgs::Float32>("ilc_rho",1);


    if ( node.getParam("timed_loop",timedLoop) ) {;}
    else
    {
        ROS_ERROR("Are waypoints timed or position based?");
        return 0;
    }
    if ( timedLoop && node.getParam("waypoint_timestep",timestep) ) {;}
    else
    {
        ROS_ERROR("What is the timing between waypoints?");
        return 0;
    }
    if (node.getParam("rho",rho)){;}
    else
    {
    	ROS_ERROR("Missing iterative learning parameter");
    	return 0;
    }
    if (node.getParam("deBug",deBug)){;}
    else
    {   
        ROS_ERROR("Did not assing debug value, deBug set to false");
        deBug = false;
    }

    //currently only running in a circle. Will bring back if needed
    /*
    if ( node.getParam("robot_name",robotName) ) {;}
    else
    {
        ROS_ERROR("Which robot number is this?");
        return 0;
    }
    if ( node.getParam("trajectory_shape",shape) ) {;}
    else
    {
        ROS_ERROR("What shape to follow?");
        return 0;
    }
    */

    std::vector<geometry_msgs::Vector3> desired_positions, errArray, shrinkPositions;
    geometry_msgs::Vector3 aveError;
    geometry_msgs::Vector3 tempError;
    aveError.x = 0.0; aveError.y=0.0; aveError.z = 0.0;
    //radius defults to 0.5 m
    fillPositionList(desired_positions);
    fillPositionList(shrinkPositions, 0.4);

    double bound = 0.1; //0.025;
    des_pos_out = desired_positions[0];
    int max = desired_positions.size()-1;
    int arg = 0;
    int count = 0;
    int iterCount = 0;
    int numCounts = 0;
    int countBound = 10;
    int firstTime = 0;
    startWaypoints = 0;
    double shrinkScale = 1.0;
    while(ros::ok())
    {
    	ros::spinOnce();

    	//take off but do not start waypoints
    	if (startControl && !startWaypoints)
        {
            
            if (iterCount == 0)
            {
                des_pos_pub.publish(shrinkPositions[arg]);
                ROS_INFO("desPos = [%.3f, %.3f], curPos = [%.3f, %.3f]",shrinkPositions[arg].x, shrinkPositions[arg].y, curr_pos.x, curr_pos.y);
            }
        	else
            {
                des_pos_pub.publish(desired_positions[arg]);
                ROS_INFO("desPos = [%.3f, %.3f], curPos = [%.3f, %.3f]",desired_positions[arg].x, desired_positions[arg].y, curr_pos.x, curr_pos.y);

            }
            

            //if(deBug)
            //{
                //ROS_INFO("desPos = [%.3f, %.3f], curPos = [%.3f, %.3f]",desired_positions[arg].x, desired_positions[arg].y, curr_pos.x, curr_pos.y);
            //}
            //des_pos_pub.publish(desired_positions[arg]);
            //ROS_INFO("desPos = [%.3f, %.3f], curPos = [%.3f, %.3f]",desired_positions[arg].x, desired_positions[arg].y, curr_pos.x, curr_pos.y);
        }
        //hovering and start waypoints
        else if(startControl && startWaypoints)
        {
        	if(timedLoop)
        	{
        		count++;
    			if((double)count/100.0 > timestep)
        		{
        			count = 0;
        			if(arg < max)
                    {
        				if(firstTime != 0){
                            arg++;
                        }
                    }
        			else
        			{   
                        aveError.x = 0.0; aveError.y=0.0; aveError.z = 0.0;
        				findAverageError(errArray, aveError,numCounts);
        				double normError = sqrt(aveError.x*aveError.x + aveError.y*aveError.y);
                        std::cout << "\n" << std::endl;
        				ROS_INFO("Iteration %d average error(m) = %.6f\trho = %.2f", iterCount,normError,rho);
                        
                        ROS_INFO("desPos = [%.3f, %.3f], curPos = [%.3f, %.3f]\n", desired_positions[arg].x, desired_positions[arg].y,curr_pos.x, curr_pos.y);

                        arg = 0;
        				numCounts = 0;
    					iterCount++;

                        firstTime = 0;
        			}
        		}
        		// for the first iteration we need U1 = xd + rho*(xd-x)
        		des_pos_out.z = desired_positions[arg].z;
        		if(!iterCount){
        			//des_pos_out.x = desired_positions[arg].x;
        			//des_pos_out.y = desired_positions[arg].y;
        			
                    des_pos_out.x = shrinkPositions[arg].x;
                    des_pos_out.y = shrinkPositions[arg].y;

                    prevU.push_back(des_pos_out);
        			prevPos.push_back(curr_pos);

        			//tempError.x = desired_positions[arg].x - curr_pos.x;
        			//tempError.y = desired_positions[arg].y - curr_pos.y;

                    tempError.x = shrinkPositions[arg].x - curr_pos.x;
                    tempError.y = shrinkPositions[arg].y - curr_pos.y;
        			
                    errArray.push_back(tempError);
                    /*
        			if(numCounts == 0)
        			{	
        				prevPos.push_back(curr_pos);
        				prevU.push_back(des_pos_out);
        				errArray.push_back(tempError);
        				numCounts++;
        			} 
                    */     			//des_pos_out = desired_positions[arg] + rho*(desired_positions[arg] + curr_pos);
        		}

        		//for every other loop we do U_k+1 = U_k(t) + rho*(xd-x)
        		else
        		{	
        			des_pos_out.x = prevU[numCounts].x + rho*(desired_positions[arg].x - prevPos[numCounts].x);
        			des_pos_out.y = prevU[numCounts].y + rho*(desired_positions[arg].y - prevPos[numCounts].y);
        			//des_pos_out = prevPos[arg] + rho*(desired_positions[arg] - cur_pos);
        			prevU[numCounts] = des_pos_out;
        			errArray[numCounts].x = desired_positions[arg].x - curr_pos.x;
        			errArray[numCounts].y = desired_positions[arg].y - curr_pos.y;
                    prevPos[numCounts] = curr_pos;
        		}
                tempError.x = desired_positions[arg].x - curr_pos.x;
                tempError.y = desired_positions[arg].y - curr_pos.y;
                tempError.z = desired_positions[arg].z - curr_pos.z;

        		//publish the position
                if(deBug)
                {
                    ROS_INFO("desPos = [%.3f, %.3f], curPos = [%.3f, %.3f]",desired_positions[arg].x, desired_positions[arg].y, curr_pos.x, curr_pos.y);
                }

                ilc_pos_out.publish(curr_pos);
                ilc_des_pos.publish(desired_positions[arg]);
                ilc_err_out.publish(tempError);
        		des_pos_pub.publish(des_pos_out);
				std_msgs::Int16 iterCountPub;
				iterCountPub.data = iterCount; 
                //ilc_itr_out.publish(iterCount);
				ilc_itr_out.publish(iterCountPub);
                //ilc_itr_start.publish(startItr);
                //ilc_rho.publish(rho);
                numCounts++;
                if (firstTime == 0)
                    firstTime = 1;

        		ros::spinOnce();
        		loop_rate.sleep();

        	}
        }
        else
        {
            
        	loop_rate.sleep();
        }
    }
    return 0;
}

//, double radius = 0.5
void fillPositionList(std::vector<geometry_msgs::Vector3>& posList, double radius)
{
	geometry_msgs::Vector3 ground;
	double xCen = 0.6,yCen = -0.9; //center of the circle
	double height = 0.75;
	//double radius = 0.5;
	int numSteps = 50;
    
	ground.z = height;
	for (int n = 0; n < numSteps; n++)
	{
		ground.x = xCen + radius*cos((double)n*2.0*M_PI/((double)numSteps));
		ground.y = yCen + radius*sin((double)n*2.0*M_PI/((double)numSteps));
		posList.push_back(ground);
	}
}

void findAverageError(std::vector<geometry_msgs::Vector3>& errList, geometry_msgs::Vector3& aveErr, int count)
{

	for(int n = 0; n < count; n++)
	{
		aveErr.x += fabs(errList[n].x);
		aveErr.y += fabs(errList[n].y);

	}
	aveErr.x = aveErr.x/((double)count);
	aveErr.y = aveErr.y/((double)count);
}
