#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
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

int main(int argc, char** argv){
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	int c;
	geometry_msgs::Twist msg;

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
			case '6': msg = twistSet(0,0.5,0); break;
			case '7': msg = twistSet(0,0,1); break;
			case '9': msg = twistSet(0,0,-1); break;
			default : break;
		}
		twist_pub.publish(msg);	
	}
	return 0;
}
