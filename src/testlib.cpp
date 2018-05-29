#include <test_lib/testlib.h>
#include <test_lib/mrpt_reactivenav_node.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_local_planner.h>
#include <mrpt_bridge/pose.h>
#include <ros/ros.h>

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
		m_reactive = new ReactiveNavNode(argc,args);
		ROS_INFO("BUILDER: WRAPPER NODE NAVIGATOR STARTED WITH ARGUMENTS");
	};

	bool MyNavigator::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		//TODO
		// We're gonna ignore cmd_vel and send the goal to mrpt's reactive navigation engine.
		if (!m_g_plan.empty()) 
		{
			int ind = m_g_plan.size() < WAYPOINT_INDEX ? m_g_plan.size() - 1 : WAYPOINT_INDEX;
			geometry_msgs::PoseStamped goal=m_g_plan[ind];
			ROS_INFO("MyNavigator::computeVelocityCommands :%f,%f,%f",
			 goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
			m_goal_pub.publish(goal);
			
		}
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
		
		if (!ros::isInitialized()) {
      			ROS_ERROR("[MyNavigator::setPlan] This planner has not been initialized, please call initialize() before using this planner");
      			return false;
		}
		m_g_plan.clear();
		m_g_plan = plan;
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
		m_reactive = new ReactiveNavNode(0,a);
		m_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/reactive_nav_goal",1);
		m_pose_sub = m_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1,
			&MyNavigator::poseCallback, this);
		
	};

	void MyNavigator::poseCallback(geometry_msgs::PoseWithCovarianceStamped robot_pose) 
	{
		m_robot_pose_ = robot_pose.pose.pose;
	}


};


PLUGINLIB_EXPORT_CLASS(testlib::MyNavigator, nav_core::BaseLocalPlanner)
int main(int argc, char **argv)
{
        testlib::MyNavigator  the_node(argc, argv);
	ros::spin();
	return 0;
}
