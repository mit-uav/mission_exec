#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";


class MissionExecutive {
	ros::Publisher correct_dest, kill_switch;
	image_transport::Subscriber image;
	cv_bridge::CvImagePtr cv_ptr;
	bool flying;
	public:

	MissionExecutive(ros::NodeHandle& nh) {
		correct_dest = nh.advertise<geometry_msgs::Twist>("/mituav/true_destination", 1000);
		kill_switch = nh.advertise<std_msgs::Empty>("/mituav/kill_switch", 1000);

		flying = true;
	};

	void check_commands(const geometry_msgs::Twist& msg) {

		if (flying) {
			ROS_INFO_STREAM("I heard: " << msg);
			cmd_vel.publish(msg);
		}
	};

	void stop(const std_msgs::Empty& msg) {
		ROS_INFO_STREAM("Stopping!!!!");
		ardrone_stop.publish(msg);
		flying = false;
	}

	void get_image(const sensor_msgs::ImageConstPtr& msg) {
		cy_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}

	void update_state(const std_msgs::Empty& msg) { return }

};


int main (int argc, char ** argv) {

	ros::init(argc, argv, "mission_executive");

	ros::NodeHandle nh;
	ros::Subscriber full_state, destination;

	MissionExecutive me(nh);

	cv::namedWindow("view");
	cv::startWindowThread();

	full_state = nh.subscribe("/mituav/FullState", 1000, &MissionExecutive::update_state, &me);
	destination = nh.subscribe("/mituav/destination", 1000, &MissionExecutive::check_commands, &me);

	ros::Rate loop_rate(200);

	while (ros::ok())
	{
		ros::spinOnce();
		me.update_env();
		loop_rate.sleep();
	}

	cv::destroyWindow("view");

	return 0;
}


