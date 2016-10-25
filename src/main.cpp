#include <ids_camera.h>

using namespace ids;

int main(int argc, char** argv){

	ros::init(argc, argv, "ids_viewer_node");

   	ros::NodeHandle n;

   	ROS_INFO("Start IDS XS camera node");

	ids::ids_camera ids_XS(&n);

	return 0;
}