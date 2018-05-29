#include <test_lib/testlib.h>
#include <test_lib/mrpt_reactivenav_node.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_local_planner.h>
#include <mrpt_bridge/pose.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// This is needed to publish the plans (global and local)
// base_local_planner::publishPlan(plan,publisher);
#include <base_local_planner/goal_functions.h>

namespace testlib 
{

	MyNavigator::MyNavigator() 
	{
		ROS_INFO("BUILDER: WRAPPER NODE NAVIGATOR STARTED");
	};

	MyNavigator::~MyNavigator() 
	{
		
	};

	MyNavigator::MyNavigator(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) 
	{
		//MyNavigator::initialize(name,tf,costmap_ros);	
	};

	MyNavigator::MyNavigator(int argc, char **args)
	{
		reactive = new ReactiveNavNode(argc,args);
		ROS_INFO("BUILDER: WRAPPER NODE NAVIGATOR STARTED WITH ARGUMENTS");
	};

	bool MyNavigator::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		//TODO
		return true;
	};

	bool MyNavigator::isGoalReached() 
	{

		//bool CWaypointsNavigator::checkHasReachedTarget 	( 	const double  	targetDist	) 	const
		// not protected, will need to find another way.
		// TODO
		return false;
	};
		 
	bool MyNavigator::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
	{
		// TODO
		
		/*if (!plan.empty()) 
		{
			geometry_msgs::PoseStamped goal=plan[plan.size()/4];
			ROS_INFO("MyNavigator::setPlan :%f,%f,%f",
			 goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
			goal_pub.publish(goal);
			
		}*/
		if (! ros::isInitialized()) {
      			ROS_ERROR("[MyNavigator::setPlan] This planner has not been initialized, please call initialize() before using this planner");
      			return false;
		}
		g_plan.clear();
		g_plan = plan;
		return true;
	};
	void MyNavigator::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) 
	{
		// TODO
		//int c;
		//char** a = "the_node";
		char x = 'a';
		char *c = &x;
		char **a = &c;
		//MyNavigator::setNavNode(0,&a);
		ROS_INFO("testlib::MyNavigator: INITIALISING FROM METHOD MyNavigator::initialize\n");
		reactive = new ReactiveNavNode(0,a);
		goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/reactive_nav_goal",1);
		
	};


};


PLUGINLIB_EXPORT_CLASS(testlib::MyNavigator, nav_core::BaseLocalPlanner)
int main(int argc, char **argv)
{
        testlib::MyNavigator  the_node(argc, argv);
	ros::spin();
	return 0;
}
