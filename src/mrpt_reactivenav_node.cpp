#include <test_lib/mrpt_reactivenav_node.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(testlib::ReactiveNavNode, nodelet::Nodelet);

int main(int argc, char **argv)
{	
        testlib::ReactiveNavNode  the_node(argc, argv);
	ros::spin();
	return 0;
}



