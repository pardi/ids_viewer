#ifndef __IDS_CAMERA_H__
#define __IDS_CAMERA_H__

// Standard Lib
#include <wchar.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <sstream>
// OpenCV Lib
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
// IDS XS Lib
#include <uEye.h> 
#include <ueye_deprecated.h>
// ROS Lib
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define CAM_VIDEO_WIDTH 	1280 
#define CAM_VIDEO_HEIGHT 	720
#define CAM_VIDEO_BPP 		24

using namespace std;
using namespace cv;

namespace ids {

	class ids_camera{

	public:
		// Classic constructor
		ids_camera(const HIDS, const int, const bool verbose = false);
		ids_camera(ros::NodeHandle* n);

		~ids_camera();
		
		// Start camera stream		
		bool cameraOn();

		// Recording video
		bool recON(const std::string);
		bool recOFF();

		// Get current frame
		Mat get_frame();

	private:

		// Init function
		bool init();
		
		// terminate on error
		void terminate_on_error();

		// ROS spin function
		void spin();

		// Variables

		bool verbose_;

		HIDS hCam_;
		int fps_;
		int width_, heigth_, bpp_;

		cv::VideoWriter outVid_;

		bool is_terminate_;
		bool recON_;

		// ROS

		ros::NodeHandle* n_;
		image_transport::Publisher ids_pub_;


	};
} 
#endif 