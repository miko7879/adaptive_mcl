#include "mclTeleop.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "mclTeleop");
  	ros::NodeHandle n;
	ros::Publisher velocity = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	swapConsole();
	
	std::cout<<"Please make a move: (WASD)\n";
	
	while(ros::ok){
		char c = getchar();
		std::cout<<"Please wait\n";
		geometry_msgs::Twist cmd;
		cmd.linear.y = 0;
		cmd.linear.z = 0;
		cmd.angular.x = 0;
		cmd.angular.y = 0;
		switch(c){
			case 'w':
				cmd.linear.x = LIN_VEL;
				cmd.angular.z = 0;
				velocity.publish(cmd);
				break;
			case 'a':
				cmd.linear.x = 0;
				cmd.angular.z = ANG_VEL;
				velocity.publish(cmd);
				break;
			case 's':
				cmd.linear.x = -LIN_VEL;
				cmd.angular.z = 0;
				velocity.publish(cmd);
				break;
			case 'd':
				cmd.linear.x = 0;
				cmd.angular.z = -ANG_VEL;
				velocity.publish(cmd);
				break;
			case 'x':
				std::cout<<"Goodbye\n";
				restoreConsole();
				exit(0);
				break;
			default:
				break;
		}
		ros::topic::waitForMessage<std_msgs::String>("/Synch", n);
		std::cout<<"Enter next command\n";
	
	}	
	restoreConsole();
	return 0;
}
