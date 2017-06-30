#include "ids_node.h" 

ids_node::ids_node(ros::NodeHandle* n, const bool verbose) : ids_class(verbose){

	// Store node handle
	n_ = n;

	// Initialize vars
	is_terminate_ = false;
	recON_ = false;

	// Get namespace
	std::string ns = ros::this_node::getNamespace();

	// Get param
	std::string rec, off_vid;
	int hCam, profile, cmpJPG, pxClock, dm;
	bool af;

   	n_->param(ns + "/ids_node/id", hCam, 1);
   	hCam_ = (HIDS) hCam;
	n_->param<float>(ns + "/ids_node/fps", fps_, 15.0);
	n_->param(ns + "/ids_node/profile", profile, (int) WVGA_HS);
	profile_ = (UINT) profile;
	n_->param(ns + "/ids_node/verbose", verbose_, true);
	n_->param<std::string>(ns +"/ids_node/recName", rec, "");
	n_->param(ns + "/ids_node/offline", offline_, false);
	n_->param(ns + "/ids_node/autofocus", af, true);
	n_->param(ns + "/ids_node/compression", cmpJPG, 2);
	n_->param(ns + "/ids_node/pixelClock", pxClock, 9);
	n_->param(ns + "/ids_node/displayMode", dm, (int) IS_SET_DM_DIB);
	n_->param(ns + "/ids_node/exposure", expostep_, (int) 5);
	dm_ = (UINT) dm;

	if (offline_){

		n_->param<std::string>(ns + "/ids_node/offlineName", off_vid, "");

		off_video_ = new VideoCapture(off_vid.c_str()); 

		if(!off_video_->isOpened()){  // check if we succeeded
			ROS_INFO("Can not open the video");
			return;
		}

		// Get Params 
		Mat fp;
		fp = get_frame();
		
		width_ = fp.cols;
		height_ = fp.rows;

	}else{

		if(!setParam(af, cmpJPG, pxClock)){
			close();
			return;
		}
		
		if (!cameraOn()){
			close();
			return;
		}
		
		if (!rec.empty())
			recON(rec);
		
	}
	
	// Service
	param_ser_ = n_->advertiseService(ns + "/ids_viewer/params", &ids_node::paramService, this);

	// Generate image transport element
  	image_transport::ImageTransport it(*n_);
  	ids_pub_ = it.advertise(ns + "/ids_viewer/image", 1);

	spin();
}

ids_node::~ids_node(){

	if (offline_)
		delete off_video_;
	else
		if (verbose_)
			ROS_INFO("Closing IDS node");
}

bool ids_node::setParam(const bool af, const int cmpJPG, const int pxClock){
	
	if (init() != IS_SUCCESS){
		print_err();
		return false;
	}

	if (setProfile(profile_) != IS_SUCCESS)
		print_err();

	if (setPixelClock(pxClock) != IS_SUCCESS)
		print_err();

	if (setCompression(cmpJPG) != IS_SUCCESS)
		print_err();

	UINT AF_CMD;

	if (af)
		AF_CMD = FOC_CMD_SET_ENABLE_AUTOFOCUS;
	else
		AF_CMD = FOC_CMD_SET_DISABLE_AUTOFOCUS;

	if (setAF(FOC_CMD_SET_DISABLE_AUTOFOCUS) != IS_SUCCESS)
		print_err();

	double newFPS;

	if (setFrameRate(30, &newFPS) != IS_SUCCESS)
		print_err();

	double expRange[3];

	if (getExposure(expRange) != IS_SUCCESS)
		print_err();

	for (int i = 0; i < 3; ++i)
		cout << expRange[i] << endl;

	if (setExposure(expRange[0] + expostep_ * expRange[2]) != IS_SUCCESS)
		print_err();

	return true;
}


// Get Frame from ids camera
Mat ids_node::get_frame(){

	if (offline_){

		VideoCapture capture = *off_video_ ;
		capture >> frame_;

	}else{

		ids_class::get_frame();

		if (recON_)
			outVid_ << frame_;
	}

	return frame_;
}

bool ids_node::cameraOn(){

	// Check if camera is alive
	if (is_terminate_)
		return false;

	// Enable camera capture routine
	if (verbose_)
		ROS_INFO("CaptureVideo Enabled");
	

	if (setCaptureType(dm_) != IS_SUCCESS){
		print_err();
		return false;
	}

	return true;
}

bool ids_node::recON(std::string str){

	if (offline_){
		if (verbose_)
			ROS_WARN("Unable to set REC Mode, the offline option is ON");
		return false;
	}

	if (recON_){
		if (verbose_)
			ROS_WARN("REC is already On");
		return true;
	}

	// Check if the file already exist

	int count = 0;

	while (is_file_exist(str.c_str())){

		string str_c;
		stringstream strstream;
		strstream << count;
		strstream >> str_c;

		if (count == 0)
			str.erase(str.end() - 4, str.end());
		else
			str.erase(str.end() - str_c.size()  - 1, str.end());

		str += "_" + str_c + ".avi";

		count++;
	}
	
	int ex = static_cast<int>(CV_FOURCC('M', 'P', '4', 'V')); 
	
	// Transform from int to char via Bitwise operators
	char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
	
	// Open video

	outVid_.open(str.c_str(), ex, fps_, Size( width_, height_ ), true);

	if (outVid_.isOpened()) {
		recON_ = true;
		
		return true;
	}

	return false;
}

bool ids_node::recOFF(){

	recON_ = false;
}


bool ids_node::is_file_exist(const char *fileName){

    std::ifstream infile(fileName);
    return infile.good();
}


void ids_node::spin(){

	ROS_INFO("Start spin()");

	ros::Rate r(fps_);

	/// ----------------------------------------------------> Time manager <----------------------------------------------------

	// struct timeval time_after, time_before;

    	// // Get current time  
    	// gettimeofday(&time_after, NULL);
    	// time_before = time_after;

	/// -------------------------------------------------------------------------------------------------------------------------------------


	while(ros::ok()){

		/// ----------------------------------------------------> Time manager <----------------------------------------------------

		// gettimeofday(&time_after, NULL);

		// cout << "Time: " << diff_ms(time_after, time_before) << endl;

		// time_before = time_after;

		/// -------------------------------------------------------------------------------------------------------------------------------------

		get_frame();

		//
		// cout << "frame" << endl;
		// imshow("Frame", frame );

		//
		
		// Generate message from image

		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();

		// Publish the message
		ids_pub_.publish(msg); 

		// ros spin

		ros::spinOnce();
  		cv::waitKey(1);

		// Wait
		r.sleep();
	}
}

bool ids_node::paramService(ids_viewer::IDSparams::Request  &req, ids_viewer::IDSparams::Response &res){

	res.width = width_;
	res.height = height_;

	return true;
}
