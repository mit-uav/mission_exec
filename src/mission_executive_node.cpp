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
	ros::Subscriber full_state, destination;
	image_transport::Subscriber image;
	cv_bridge::CvImagePtr cv_ptr;
	bool flying;
	public:

	MissionExecutive(ros::NodeHandle& nh) {
		correct_dest = nh.advertise<geometry_msgs::Twist>("/mituav/true_destination", 1000);
		kill_switch = nh.advertise<std_msgs::Empty>("/mituav/kill_switch", 1000);
		full_state = nh.publish<geometry_msgs::Twist>("/mituav/FullState", 1000);
		destination = nh.publish<std_msgs::Empty>("/mituav/destination", 1000);
		
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

	void update_env() {
		
	}

};


int main (int argc, char ** argv) {

	ros::init(argc, argv, "mission_executive");

	ros::NodeHandle nh;

	MissionExecutive me(nh);

	cv::namedWindow("view");
	cv::startWindowThread();

	ros::Subscriber recieved_command = nh.subscribe("executive/cmd_vel", 1000, &MissionExecutive::check_commands, &me);
	ros::Subscriber emergency_stop = nh.subscribe("executive/stop", 1000, &MissionExecutive::stop, &me);
	//ros::Subscriber front_camera = nh.subscribe("ardrone/front/image_raw", 1, &MissionExecutive::get_image, &me);
	ImageTransport::Subscriber usb_cam = nh.subscribe("usb_cam/image_raw", 1, &MissionExecutive::get_image, &me);

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


