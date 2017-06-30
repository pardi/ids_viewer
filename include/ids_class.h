#ifndef __IDS_CLASS_H__
#define __IDS_CLASS_H__

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

using namespace std;
using namespace cv;

namespace ids {

	enum profiles{
		p5M 			= 4,   	// 2592x1944@
		p3M			= 5,	// 2048x1536@
		FULL_HD169	= 6,	// 1920x1080@
		p1d2M43 		= 8,	// 1280x960@
		HD169		=	9, // 1280x720@
		WVGA		=	12,	// 800x480@15fps
		VGA 		=	13, // 640x480@15fps
		UXGA		=	20,	//1600x1200@
		VGA_HS	= 31, 	// 640x480@30fps
		WVGA_HS	= 32, 	// 800x480@30fps
	};


	class ids_class{

	public:
		// Constructors
		ids_class(const bool verbose = false);
		ids_class(const HIDS, const bool verbose = false);
		// Copy Constructor
		ids_class(const ids_class&);
		// Destructor
		~ids_class();

		// Parameter settings
		// Init the camera 
		int init();
		// Set Display Mode
		int setDisplayMode(const UINT);
		// Get Sensor Info
		int getSensorInfo(SENSORINFO*);
		// Get AOI parameters
		int getAOIparams();
		// Set AOI
		int setAOI(const int x, const int y, const int width, const int height);
		// Get available profile
		UINT getProfile();
		// Set profile
		int setProfile(const UINT);
		// Set pixel clock
		int setPixelClock(UINT);
		// Set pixel clock
		int getPixelClock(UINT*);
		// Set pixel clock range
		int getPixelClockRange(UINT*);
		// Get Compression Range
		int getCompressionRange(IS_RANGE_S32*);
		// Set Compression 
		int setCompression(UINT);
		// Get Compression
		int getCompression(UINT*);
		// Set Frame Rate
		int setFrameRate(const double, double*);
		// Get autofocus properties
		int getAFcapabilities(UINT*);
		// Set autofocus
		int setAF(const UINT);
		// Get autofocus state
		int getAFstate(UINT*);
		// Get Exposure range
		int getExposure(double*);
		// Set Exposure
		int setExposure(const double);
		// Set type of capture
		int setCaptureType(const UINT);
		// Set type of freeze
		int setFreezeType(const UINT);
		// Set ID of the camera
		void setIDCamera(const HIDS);

		// Operators
		template <typename T>
		T& operator>>(const Mat&);


		// Error Handle fcns
		// Print error message
		void print_err();
		// terminate on error
		void close();

		// Start a demo streaming
		int start_demo();
		// Get current frame
		Mat get_frame();

		// Variables

		bool verbose_;

		HIDS hCam_;
		UINT profile_;
		UINT dm_;

		float fps_;
		int width_, height_, bpp_;
		int expostep_;
		
		// Memory Allocation vars
		char* pMem_ ;
		int memID_;
		Mat frame_;

	};
} 
#endif 