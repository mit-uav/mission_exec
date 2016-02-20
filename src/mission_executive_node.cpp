#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


class MissionExecutive {
	ros::Publisher cmd_vel;
	ros::Publisher ardrone_stop;
	public:

	MissionExecutive(ros::NodeHandle& nh) {
		cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
		ardrone_stop = nh.advertise<std_msgs::Empty>("ardrone/reset", 1000);
	};

	void check_commands(const geometry_msgs::Twist& msg) {
		ROS_INFO_STREAM("I heard: " << msg);
		cmd_vel.publish(msg);
	};

	void stop(const std_msgs::Empty& msg) {
		ROS_INFO_STREAM("Stopping!!!!");
		ardrone_stop.publish(msg);
	}

};


int main (int argc, char ** argv) {

	ros::init(argc, argv, "mission_executive");

	ros::NodeHandle nh;

	MissionExecutive me(nh);

	ros::Subscriber recieved_command = nh.subscribe("executive/cmd_vel", 1000, &MissionExecutive::check_commands, &me);
	ros::Subscriber emergency_stop = nh.subscribe("executive/stop", 1000, &MissionExecutive::stop, &me);

	ros::Rate loop_rate(10);

	ros::spin();

	return 0;
}


