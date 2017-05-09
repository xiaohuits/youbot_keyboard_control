#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include <iostream>
#include <stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

int  get_char(){
	std::cout << "Please input: " << std::endl;
	return getchar();
}

geometry_msgs::Twist twistSet(const double& lx, const double& ly, const double& az){
	geometry_msgs::Twist msg;
	msg.linear.x = lx;
	msg.linear.y = ly;
	msg.angular.z = az;
	return msg;
}

brics_actuator::JointPositions armSet(std::vector<double>& newPositions) {
	int numberofJoints = 5;
	brics_actuator::JointPositions msg;

	if(newPositions.size() < numberofJoints) return msg;

	for(int i=0; i<numberofJoints; i++){
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

// make sure that the position is in a valid range.
void armPositionCheck (std::vector<double>& position, const double *min, const double *max) {
	for (int i=0; i<position.size(); i++) {
		if (position[i]<min[i]) position[i]=min[i];
		else if (position[i]>max[i]) position[i]=max[i];
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Publisher arm_pub = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command",1);
	int c;
	geometry_msgs::Twist msg;
	const std::vector<double> initalPosition = {0.11,0.11,-0.11,0.11,0.111};
	const std::vector<double> straightUp = {2.95,1.05,-2.44,1.73,2.95};
	const double armmin[5]={0.0100693,0.0100693,-5.02654,0.022124,0.11062};
	const double armmax[5]={5.84013,2.61798,-0.015709,3.4291,5.64158};

	std::vector<double> armPosition = {0.11,0.11,0.11,0.11,0.11};
	const double stepSize = 0.1;
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
		switch (c){
			case '5': msg = twistSet(0,0,0); break;
			case '8': msg = twistSet(0.5,0,0); break;
			case '2': msg = twistSet(-0.5,0,0); break;
			case '4': msg = twistSet(0,0.5,0); break;
			case '6': msg = twistSet(0,-0.5,0); break;
			case '7': msg = twistSet(0,0,1); break;
			case '9': msg = twistSet(0,0,-1); break;
			case 'y': armPosition[0]+=stepSize; break;
			case 'Y': armPosition[0]-=stepSize; break;
			case 'u': armPosition[1]+=stepSize; break;
			case 'U': armPosition[1]-=stepSize; break;
			case 'i': armPosition[2]+=stepSize; break;
			case 'I': armPosition[2]-=stepSize; break;
			case 'o': armPosition[3]+=stepSize; break;
			case 'O': armPosition[3]-=stepSize; break;
			case 'p': armPosition[4]+=stepSize; break;
			case 'P': armPosition[4]-=stepSize; break;
			case 'r': armPosition = initalPosition; break;
			default : break;
		}
		twist_pub.publish(msg);
		armPositionCheck(armPosition,armmin,armmax);	
		arm_pub.publish(armSet(armPosition));
	}
	return 0;
}
