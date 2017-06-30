#include <ids_node.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "ids_viewer_node");

   	ros::NodeHandle n;

   	ROS_INFO("Start IDS XS camera node");

	ids_node ueye_XS(&n);

	return 0;
}