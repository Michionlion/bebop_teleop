#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "webcam_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("bebop/image_raw", 1);

	cv::VideoCapture capture(0);
	if( !capture.isOpened() ) {
		ROS_ERROR("FAILED TO OPEN CAPTURE");
		return -1;
	}

	cv::Mat input;
	cv::Mat output;
	cv::Size size(640, 368);
	sensor_msgs::ImagePtr msg;

	ros::Rate r(10);
	while( nh.ok() ) {
		capture >> input;

		if( !input.empty() ) {
			cv::resize(input, output, size);
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
			pub.publish(msg);
		}

		ros::spinOnce();
		r.sleep();
	}

	pub.shutdown();
	ros::shutdown();
}
