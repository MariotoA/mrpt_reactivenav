#include <test_lib/mrpt_reactivenav_node.h>


int main(int argc, char **argv)
{
        ReactiveNavNode  the_node(argc, argv);
	ros::spin();
	return 0;
}



