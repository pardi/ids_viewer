#include "ids_class.h" 

using namespace ids;

ids_class::ids_class(const bool verbose){

	hCam_ = 0;
	verbose_ = verbose;
	profile_ = 0;
	width_ = 0;
	height_ = 0;
	bpp_ = 24;
}

ids_class::ids_class(const HIDS id, const bool verbose){

	hCam_ = id;
	verbose_ = verbose;
	profile_ = 0;
	width_ = 0;
	height_ = 0;
	bpp_ = 24;
}

ids_class::ids_class(const ids_class& cam){
	hCam_ = cam.hCam_;
	verbose_ = cam.verbose_;
	profile_ = cam.profile_;
	width_ = cam.width_;
	height_ = cam.height_;
	bpp_ = cam.bpp_;
}

ids_class::~ids_class(){

	if (verbose_)
		cout << "[INFO] Closing IDS camera" << endl;

	is_FreeImageMem(hCam_, pMem_, memID_);

	is_ExitCamera(hCam_);
}

int ids_class::init(){

	// Init camera
	if (verbose_)
		cout << "[INFO] Camera init" << endl;

	return is_InitCamera (&hCam_, NULL);
}

int ids_class::setDisplayMode(const UINT dm){

	if (verbose_)
		cout << "[INFO] Set Display Mode" << endl;

	return is_SetDisplayMode (hCam_, dm);
}

int ids_class::getSensorInfo(SENSORINFO* pinfo){

	if (verbose_)
		cout << "[INFO] Get Sensor Info" << endl;
 	
	int ret = is_GetSensorInfo(hCam_, pinfo);

	if (verbose_ && ret == IS_SUCCESS){
		cout << "[Camera Info]" << endl;
		cout << "Sensor Name\t --> " << pinfo->strSensorName << endl;
		cout << "Sensor ID\t --> " << pinfo->SensorID << endl;
		cout << "Max Width\t --> " << pinfo->nMaxWidth << endl;
		cout << "Max Height\t --> "  << pinfo->nMaxHeight << endl;
		cout << "Pixel Size\t --> "  << pinfo->wPixelSize << endl;
	}

	return ret;
}

int ids_class::getAOIparams(){

	return 0;
}

int ids_class::setAOI(const int x, const int y, const int width, const int height){


	return 0;
}

UINT ids_class::getProfile(){

	if (verbose_)
		cout << "[INFO] Get Profile" << endl;

	return profile_;
}

int ids_class::setProfile(const UINT profile){

	if (verbose_)
		cout << "[INFO] Set Profile" << endl;

	profile_ = profile;

	int ret;

	ret = is_ImageFormat(hCam_, IMGFRMT_CMD_SET_FORMAT, &profile_, sizeof(profile_));

	if (ret != IS_SUCCESS)
		return ret;

	switch(profile_){
		case p5M: {
					width_ = 2592;
					height_ = 1944;
					}break;
		case p3M: {
					width_ = 2048;
					height_ = 1536;
					}break;
		case FULL_HD169: {
							width_ = 1920;
							height_ = 1080;
						}break;
		case p1d2M43: {
							width_ = 1280;
							height_ = 960;
						}break;
		case HD169: {
						width_ = 1280;
						height_ = 720;
						}break;
		case WVGA:
		case WVGA_HS: {
						width_ = 800;
						height_ = 480;
						}break;
		case VGA: 
		case VGA_HS: {
						width_ = 640;
						height_ = 480;
					}break;
		case UXGA: 
		default: {
					width_ = 1600;
					height_ = 1200;
				}break;
	}

	return ret;
}

int ids_class::setPixelClock(UINT pixelClock){

	if (verbose_)
		cout << "[INFO] Set Pixel Clock" << endl;

	return is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_SET, (void*)&pixelClock, sizeof(pixelClock));
}

int ids_class::getPixelClock(UINT* range){

	if (verbose_)
		cout << "[INFO] Get Pixel Clock" << endl;

	return is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_GET, (void*)range, sizeof(range));
}

int ids_class::getPixelClockRange(UINT* range){

	if (verbose_)
		cout << "[INFO] Get Pixel Clock Range" << endl;

	UINT nRange[3];
	ZeroMemory(nRange, sizeof(nRange));

	int ret;

	ret = is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange));

	if (verbose_ && ret == IS_SUCCESS ){

		cout << "[Pixel Clock Range]" << endl;
		cout << "Mininum Clock: " << nRange[0] << endl;
		cout << "Maximum Clock: " << nRange[1] << endl;
		cout << "Increment Clock: " << nRange[2] << endl;
	}

	range[0] = nRange[0];
	range[1] = nRange[1];
	range[2] = nRange[2];

	return ret;
}
 
int ids_class::getCompressionRange(IS_RANGE_S32* rangeJpeg){

	if (verbose_)
		cout << "[INFO] Get JPEG Comprension Range" << endl;

	int ret;

	ret = is_DeviceFeature(hCam_, IS_DEVICE_FEATURE_CMD_GET_JPEG_COMPRESSION_RANGE, (void*)rangeJpeg, sizeof(IS_RANGE_S32));
 
	if (verbose_ && ret == IS_SUCCESS ){

		cout << "[JPEG Comprension Range]" << endl;
		cout << "Mininum: " << rangeJpeg->s32Min << endl;
		cout << "Maximum: " << rangeJpeg->s32Max << endl;
		cout << "Increment: " << rangeJpeg->s32Inc << endl;
	}

	return ret; 
}

int ids_class::getCompression(UINT* value){

	if (verbose_)
		cout << "[INFO] Get JPEG Comprension" << endl;

	return is_DeviceFeature(hCam_, IS_DEVICE_FEATURE_CMD_GET_JPEG_COMPRESSION, (void*)value, sizeof(UINT));

}

int ids_class::setCompression(UINT value){

	if (verbose_)
		cout << "[INFO] Set JPEG Comprension" << endl;
	
	return is_DeviceFeature(hCam_, IS_DEVICE_FEATURE_CMD_SET_JPEG_COMPRESSION, (void*)&value, sizeof(value));
}

int ids_class::setFrameRate(const double fps, double* newFPS){

	int ret;

	if (verbose_)
		cout << "[INFO] Set Frame Rate" << endl;

	ret = is_SetFrameRate(hCam_, fps, newFPS);

	fps_ = *newFPS;

	return ret;
}

int ids_class::setCaptureType(const UINT dm){

	if (verbose_)
		cout << "[INFO] Set Capture Type" << endl;

	int ret;

	ret = setDisplayMode(dm);

	if (ret != IS_SUCCESS)
		return ret;
	
	pMem_ = NULL;
	memID_ = 	0;

	// Allocate image mem for current format, set format

	ret = is_AllocImageMem(hCam_, width_, height_, bpp_, &pMem_, &memID_);

	if (ret != IS_SUCCESS)
		return ret;
   
	ret = is_SetImageMem(hCam_, pMem_, memID_);
	
	if (ret != IS_SUCCESS)
		return ret;

	Mat frame(height_, width_, CV_8UC3);

	frame_ = frame;

	for (UINT i = 0; i < 10; ++i){
		ret = is_CaptureVideo (hCam_, IS_DONT_WAIT);

		if (ret == IS_SUCCESS)
			return ret;
	}

	return ret;
}

int ids_class::setFreezeType(const UINT dm){

	if (verbose_)
		cout << "[INFO] Set Freeze Type" << endl;

	int ret;

	ret = setDisplayMode(dm);

	if (ret != IS_SUCCESS)
		return ret;


	for (UINT i = 0; i < 10; ++i){
		ret = is_FreezeVideo (hCam_, IS_DONT_WAIT);

		if (ret == IS_SUCCESS)
			return ret;
	}

	return ret;
}

void ids_class::print_err(){

	INT pErr;
	IS_CHAR* ppcErr;

	is_GetError(hCam_, &pErr, &ppcErr);

	cout << "[Error description]" << endl;
	cout << "Code: " << pErr << endl;
	cout << "Text: " << ppcErr << endl;
}

void ids_class::close(){

	if (verbose_)
		cout << "[INFO] Close camera" << endl;

	// Close camera
	is_ExitCamera(hCam_);

}

Mat ids_class::get_frame(){

	//pointer to where the image is stored
	uchar* pMemVoid; 

	// Pointer on matrix
	uchar* x = frame_.ptr();

	// Get Image from camera
	
	if ( is_GetImageMem (hCam_, (void**) &pMemVoid) != IS_SUCCESS)
		print_err();

	// Copy img in Mat structure
	for (int i =  0; i < width_ * height_ * 3; ++i )
		x[i] = (uchar) pMemVoid[i];

		
	return frame_;
}


int ids_class::getAFcapabilities(UINT* capabilieties){
	
	if (verbose_)
		cout << "[INFO] Get AutoFocus capabilieties" << endl;
	
	return is_Focus (hCam_, FOC_CMD_GET_CAPABILITIES, capabilieties, sizeof (UINT) );
}
	


int ids_class::setAF(const UINT af_cmd){

	if (verbose_)
		cout << "[INFO] Set AutoFocus" << endl;

	UINT enable;

	return is_Focus (hCam_, af_cmd, &enable, sizeof (enable) );
}

int ids_class::getAFstate(UINT* enable){

	if (verbose_)
		cout << "[INFO] Check AutoFocus" << endl;

	return is_Focus (hCam_, FOC_CMD_GET_AUTOFOCUS_ENABLE, enable, sizeof (enable) );
}

int ids_class::getExposure(double* range){

	if (verbose_)
		cout << "[INFO] Get Exposure Range" << endl;

	double r[3];

	int ret = is_Exposure (hCam_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE,  (void*) r, sizeof(r));

	if ( ret != IS_SUCCESS)
		print_err();

	range[0] = r[0];  // [MIN]
	range[1] = r[1];  // [MAX]
	range[2] = r[2];  // [INC]

	return ret;
}

int ids_class::setExposure(const double exposure){

	if (verbose_)
		cout << "[INFO] Set Exposure" << endl;

	double exp = exposure;

	int ret = is_Exposure (hCam_, IS_EXPOSURE_CMD_SET_EXPOSURE,  (void*) &exp, sizeof(exp));

	if ( ret != IS_SUCCESS)
		print_err();

	return ret;
}

void ids_class::setIDCamera(const HIDS id){

	hCam_ =  id;
}

// std::istream& operator>>(std::istream& is, T& obj)
// {
//     // read obj from stream
//     if(  T could not be constructed  )
//         is.setstate(std::ios::failbit);
//     return is;
// }