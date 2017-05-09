#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"

#include <iostream>
#include <stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

int  get_char()
{
	std::cout << "Please input: " << std::endl;
	return getchar();
}

geometry_msgs::Twist twistSet(const double& lx, const double& ly, const double& az)
{
	geometry_msgs::Twist msg;
	msg.linear.x = lx;
	msg.linear.y = ly;
	msg.angular.z = az;
	return msg;
}

brics_actuator::JointPositions armSet(std::vector<double>& newPositions) 
{
	int numberofJoints = 5;
	brics_actuator::JointPositions msg;

	if(newPositions.size() < numberofJoints) return msg;

	for(int i=0; i<numberofJoints; i++)
	{
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		std::stringstream jointName;
		jointName << "arm_joint_" << (i+1);
		joint.joint_uri = jointName.str();

		msg.positions.push_back(joint);
	}
	return msg;
}

brics_actuator::JointPositions gripperSet(double& newPosition)
{
	brics_actuator::JointPositions msg;
	brics_actuator::JointValue joint;

	if(newPosition>0.011) newPosition=0.011;
	else if(newPosition<0) newPosition=0;

	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter);
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);

	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);

	return msg;
}

// make sure that the position is in a valid range.
void armPositionCheck (std::vector<double>& position, const double *min, const double *max) 
{
	for (int i=0; i<position.size(); i++) 
	{
		if (position[i]<min[i]) position[i]=min[i];
		else if (position[i]>max[i]) position[i]=max[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Publisher arm_pub = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command",1);
	ros::Publisher gripper_pub = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command",1);

	int c;
	const std::vector<double> initalPosition = {0.11,0.11,-0.11,0.11,0.111};
	const std::vector<double> straightUp = {2.95,1.05,-2.44,1.73,2.95};
	const double armmin[5]={0.0100693,0.0100693,-5.02654,0.022124,0.11062};
	const double armmax[5]={5.84013,2.61798,-0.015709,3.4291,5.64158};
	const double arm_step_size = 0.1;
	const double gripper_step_size = 0.001;

	std::vector<double> arm_position = {0.11,0.11,-0.11,0.11,0.11};
	geometry_msgs::Twist msg;
	double gripper_position = 0;

	static struct termios oldt, newt;
        /*tcgetattr gets the parameters of the current terminal
   	STDIN_FILENO will tell tcgetattr that it should write the settings
    	of stdin to oldt*/
    	tcgetattr( STDIN_FILENO, &oldt);
   	/*now the settings will be copied*/
    	newt = oldt;

    	/*ICANON normally takes care that one line at a time will be processed
    	that means it will return if it sees a "\n" or an EOF or an EOL*/
    	newt.c_lflag &= ~(ICANON);          

    	/*Those new settings will be set to STDIN
    	TCSANOW tells tcsetattr to change attributes immediately. */
    	tcsetattr( STDIN_FILENO, TCSANOW, &newt);

	while (ros::ok())
	{
		c = get_char();
		switch (c)
		{
			case '5': msg = twistSet(0,0,0); break;
			case '8': msg = twistSet(0.5,0,0); break;
			case '2': msg = twistSet(-0.5,0,0); break;
			case '4': msg = twistSet(0,0.5,0); break;
			case '6': msg = twistSet(0,-0.5,0); break;
			case '7': msg = twistSet(0,0,1); break;
			case '9': msg = twistSet(0,0,-1); break;
			case 'y': arm_position[0]+=arm_step_size; break;
			case 'Y': arm_position[0]-=arm_step_size; break;
			case 'u': arm_position[1]+=arm_step_size; break;
			case 'U': arm_position[1]-=arm_step_size; break;
			case 'i': arm_position[2]+=arm_step_size; break;
			case 'I': arm_position[2]-=arm_step_size; break;
			case 'o': arm_position[3]+=arm_step_size; break;
			case 'O': arm_position[3]-=arm_step_size; break;
			case 'p': arm_position[4]+=arm_step_size; break;
			case 'P': arm_position[4]-=arm_step_size; break;
			case 'r': arm_position = initalPosition; break;
			case '-': gripper_position-=gripper_step_size; break;
			case '=': gripper_position+=gripper_step_size; break;
			default : break;
		}
		twist_pub.publish(msg);
		armPositionCheck(arm_position,armmin,armmax);	
		arm_pub.publish(armSet(arm_position));
		gripper_pub.publish(gripperSet(gripper_position));
	}
	return 0;
}
