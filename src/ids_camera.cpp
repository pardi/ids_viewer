#include "ids_camera.h" 

using namespace ids;

ids_camera::ids_camera(const HIDS id, const int fps, const bool verbose){

	hCam_ = id;
	verbose_ = verbose;

	// Initialize vars

	fps_ = fps;

	is_terminate_ = false;

	if (!init()){
	    terminate_on_error();
	    return;
	}
	
}

ids_camera::ids_camera(ros::NodeHandle* n){

	// Store node handle
	n_ = n;

	// Get param
	std::string rec;
	int hCam;

   	n_->param("/ids/id", hCam, 1);
   	hCam_ = (HIDS) hCam_;

	n_->param("/ids/fps", fps_, 15);
	n_->param("/ids/verbose", verbose_, true);
	n_->param<std::string>("/ids/rec_name", rec, "");

	// Initialize vars
	is_terminate_ = false;


	if (!init()){
	    terminate_on_error();
	    return;
	}

	cameraOn();

	if (!rec.empty())
		recON(rec);
	
	// Generate image transport element
  	image_transport::ImageTransport it(*n_);
  	ids_pub_ = it.advertise("ids_viewer/image", 1);

	spin();
}

ids_camera::~ids_camera(){

	if (verbose_)
		cout << "Closing IDS camera" << endl;

	is_ExitCamera(hCam_);
}

bool ids_camera::init(){
	
	const double enable = 1;
	const double disable = 0;

	// Init camera

	if (verbose_)
		cout << "- Init camera --> ";

	if(is_InitCamera (&hCam_, NULL) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	// Memory allocation and display
	if (verbose_)
		cout << "- Alloc Image Mem --> ";

	char* imgMem;
	int memId;

	if (is_AllocImageMem(hCam_, CAM_VIDEO_WIDTH, CAM_VIDEO_HEIGHT, CAM_VIDEO_BPP, &imgMem, &memId) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	if (verbose_)
		cout << "- Set Image Mem --> ";

	if (is_SetImageMem (hCam_, imgMem, memId) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	if (verbose_)
		cout << "- Set Display Mode --> ";

	if (is_SetDisplayMode (hCam_, IS_SET_DM_DIB) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	if (verbose_)
		cout << "- Set Color Mode --> ";

	if (is_SetColorMode (hCam_, IS_SET_CM_RGB24) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	if (verbose_)
		cout << "- Set Image Size --> ";

	if (is_SetImageSize (hCam_, CAM_VIDEO_WIDTH, CAM_VIDEO_HEIGHT) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	// Params

	// cout << "- Auto Gain --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_GAIN, &enable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto WhiteBalance --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto FrameRate --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Shutter --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Sensor Gain --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0)!= IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Sensor WhiteBalance --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE,&enable,0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Sensor Shutter --> ";
	
	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &disable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	double NEWFPS;

	if (verbose_)
		cout << "- Set FrameRate --> ";

	// Set FrameRate
	if (is_SetFrameRate(hCam_, fps_, &NEWFPS) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	double parameter = 50;
	
	if (verbose_)
		cout << "- Set Exposure --> ";

	// Exposure set
	if (is_Exposure(hCam_, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*) &parameter, sizeof(parameter)) != IS_SUCCESS) 
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	UINT uiCaps = 0;

	if (verbose_)
		cout << "- Enable AutoFocus --> ";

	// Focus camera
	if (is_Focus (hCam_, FOC_CMD_GET_CAPABILITIES, &uiCaps, sizeof (uiCaps) ) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	if (verbose_)
		cout << "- Check if AutoFocus is Enable --> ";

	if (is_Focus (hCam_, FOC_CMD_GET_AUTOFOCUS_ENABLE, &uiCaps, sizeof (uiCaps) ) != IS_SUCCESS)
		return false;

	// Check on supporting of focus option

	if (uiCaps != 1){
		if (verbose_)
			cout << "NO" << endl;
	}
	else{
		if (verbose_)
			cout << "YES" << endl;
	}

	// Get pixel clock range
	UINT nRange[3];
	UINT nMin = 0, nMax = 0, nInc = 0;

	ZeroMemory(nRange, sizeof(nRange));

	if (verbose_)
		cout << "- Pixelclock get range --> ";

	if (is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange)) == IS_SUCCESS)
	{

		if (verbose_)
			cout << "OK" << endl;
	
		nMin = nRange[0];
		nMax = nRange[1];
		nInc = nRange[2];
	
		if (verbose_){
			cout << "\tMin: " << nMin << endl;
			cout << "\tMax: " << nMax << endl;
			cout << "\tInc: " << nInc << endl;
		}
	}
	else
		return false;
	
	if (verbose_)
		cout << "- Set Pixelclock --> ";

	if(is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_SET, (void*)&nMin, sizeof(nMin)) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	return true;
}

// Terminate on camera error 
void ids_camera::terminate_on_error(){

	INT pErr;
	IS_CHAR* ppcErr;

	is_GetError(hCam_, &pErr, &ppcErr);

	if (verbose_){
		cout << "Error" << endl;
		cout << "Code: " << pErr << endl;
		cout << "Text: " << ppcErr << endl;
	}

	is_terminate_ = true;

	// Close camera
	is_ExitCamera(hCam_);

}

// Get Frame from ids camera
Mat ids_camera::get_frame(){

	//pointer to where the image is stored
	uchar* pMemVoid; 

	Mat frame(CAM_VIDEO_HEIGHT, CAM_VIDEO_WIDTH, CV_8UC3);

	// Check if camera is alive

	if (is_terminate_)
		return frame;
	
	// Pointer on matrix
	uchar* x = frame.ptr();

	// Get Image from camera
	if ( is_GetImageMem (hCam_, (void**) &pMemVoid) != IS_SUCCESS){
		if (verbose_)
			cout << "- Get an Image mem --> ERROR" << endl;
	}

	// Copy img in Mat structure

	for (int i = CAM_VIDEO_WIDTH * CAM_VIDEO_HEIGHT * 3; i--; )
		x[i] = (uchar) pMemVoid[i];

	if (recON_)
		outVid_ << frame;

	return frame;

}

bool ids_camera::cameraOn(){

	// Check if camera is alive

	if (is_terminate_)
		return false;

	// Enable camera capture routine
	if (verbose_)
		cout << "- CaptureVideo Enabled --> ";
	
	if (is_CaptureVideo (hCam_, IS_WAIT) != IS_SUCCESS){
	    terminate_on_error();
	    return false;
	}else{
		if (verbose_)
			cout << "OK" << endl;
	}	

	return true;
}

bool ids_camera::recON(const std::string str){

	if (recON_){
		if (verbose_)
			cout << "- REC is already On" << endl;

		return true;
	}

	int ex = static_cast<int>(CV_FOURCC('M', 'P', '4', 'V')); 
	
	// Transform from int to char via Bitwise operators
	char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
	
	// Open videostr.c_str()

	outVid_.open(str.c_str(), ex, 15.0, Size( CAM_VIDEO_WIDTH, CAM_VIDEO_HEIGHT ), true);

	if (outVid_.isOpened()) {
		recON_ = true;
		
		return true;
	}

	ROS_INFO("false");
	return false;

}

bool ids_camera::recOFF(){

	recON_ = false;

}

void ids_camera::spin(){

	ROS_INFO("Start spin()");

	ros::Rate r(fps_);

	Mat frame;

	while(ros::ok()){

		frame = get_frame();

		// Generate message from image

		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

		// Publish the message
		ids_pub_.publish(msg); 

		// ros spin

		ros::spinOnce();
  		cv::waitKey(1);

		// Wait
		r.sleep();
	}

}
