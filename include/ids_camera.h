#ifndef __IDS_CAMERA_H__
#define __IDS_CAMERA_H__

// Standard Lib
#include <wchar.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <sstream>
#include <fstream> 
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
#include <ids_viewer/IDSparams.h>


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
		bool recON(std::string);
		bool recOFF();

		// Get current frame
		Mat get_frame();

	private:

		// Init function
		bool init();

		// Check if the video already exist
		bool is_file_exist(const char *);

		// terminate on error
		void terminate_on_error();

		// ROS spin function
		void spin();

		// Serivce
		bool paramService(ids_viewer::IDSparams::Request&, ids_viewer::IDSparams::Response&);

		// Variables

		bool verbose_;

		HIDS hCam_;
		int fps_;
		int width_, height_, bpp_;

		cv::VideoWriter outVid_;
		cv::VideoCapture* off_video_;

		bool is_terminate_;
		bool recON_;
		bool offline_;

		// ROS

		ros::NodeHandle* n_;
		image_transport::Publisher ids_pub_;

		// service

		ros::ServiceServer param_ser_;

	};
} 
#endif 