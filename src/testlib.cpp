#include <test_lib/testlib.h>
#include <test_lib/mrpt_reactivenav_node.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_local_planner.h>
#include <mrpt_bridge/pose.h>
#include <ros/ros.h>
#include <cmath>
// This is needed to publish the plans (global and local)
// base_local_planner::publishPlan(plan,publisher);
#include <base_local_planner/goal_functions.h>

namespace testlib 
{

	MyNavigator::MyNavigator() 
	{
		ROS_INFO("BUILDER: WRAPPER NODE FOR MRPT_REACTIVENAV_NODE NAVIGATOR STARTED");
	};

	MyNavigator::~MyNavigator() 
	{
		delete m_reactive;
	};

	bool MyNavigator::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		//TODO
		// We're gonna send the goal to mrpt's reactive navigation engine and then receive the command from them.

		if (!m_g_plan.empty() && isNextWaypointNeeded()) 
		{
			m_is_last_waypoint = m_g_plan.size() < WAYPOINT_INDEX;
			int ind = m_is_last_waypoint ? m_g_plan.size() - 1 : WAYPOINT_INDEX;
			m_waypoint=m_g_plan[ind];
			ROS_INFO("\n\nMyNavigator::sending goal to reactive navigator: Pose[x:%f,y:%f,z:%f]",
			 m_waypoint.pose.position.x,m_waypoint.pose.position.y,m_waypoint.pose.position.z);
			m_goal_pub.publish(m_waypoint);
		}

		cmd_vel = m_cmd_vel;

		return cmd_vel.linear.x >= .001 || cmd_vel.angular.z >= .001;
	};

	bool MyNavigator::isGoalReached() 
	{

		//bool CWaypointsNavigator::checkHasReachedTarget 	( 	const double  	targetDist	) 	const
		// not protected, will need to find another way.
		// TODO
		bool end = m_is_last_waypoint && isWaypointReached();
		if (end)
			m_is_last_waypoint = false;
		// Go back to initial state.
		return end;
	};
		 
	bool MyNavigator::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
	{
		
		if (!ros::isInitialized()) 
		{
      			ROS_ERROR("[MyNavigator::setPlan] This planner has not been initialized, please call initialize() before using this planner");
      			return false;
		}

		// Before storing next plan, last goal is stored. It could be that goal was not last pose of path for all globalplanners
		m_new_navigation = true;
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
		ROS_INFO("testlib::MyNavigator: INITIALISING FROM METHOD MyNavigator::initialize\n");
		m_reactive = new ReactiveNavNode(0,a);
		m_costmap_ros = costmap_ros;
		m_costmap_ros->getRobotPose(m_current_pose);
		m_tf = tf;
		std::string topic_reactive_goal = "/reactive_nav_goal";
		m_localnh.param("topic_relative_nav_goal", topic_reactive_goal,topic_reactive_goal);
		m_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>(topic_reactive_goal,1);
		m_target_allowed_distance = 0.4;
		m_localnh.param(
			"target_allowed_distance", m_target_allowed_distance,
			m_target_allowed_distance);
		std::string topic_cmd_vel = "cmd_vel";
		m_localnh.param("topic_cmd_vel", topic_cmd_vel,topic_cmd_vel);
		m_pub_cmd_vel = m_nh.subscribe<const geometry_msgs::Twist&>(topic_cmd_vel, 1,
			&MyNavigator::velocityCommandCallback, this);
		m_is_last_waypoint = false;
		m_new_navigation = false;
		
	};
	
	bool MyNavigator::isWaypointReached()
	{
		if (!ros::isInitialized()) 
		{
      			ROS_ERROR("[MyNavigator::isWaypointReached] This planner has not been initialized, please call initialize() before using this planner");
      			return false;
		}

		if (!m_costmap_ros->getRobotPose(m_current_pose))
		{
      			ROS_ERROR("[MyNavigator::isWaypointReached] Cannot obtain current_pose from Costmap.");
      			return false;
		}
		double x=m_current_pose.getOrigin().x() - m_waypoint.pose.position.x;
		double y=m_current_pose.getOrigin().y() - m_waypoint.pose.position.y;
		bool res = sqrt(x*x + y*y) <= m_target_allowed_distance;
		ROS_INFO(res? "\n\n\n[MyNavigator::isWaypointReached] Waypoint reached.\n\n"
					: "[MyNavigator::isWaypointReached] Waypoint not reached yet."
		);
		return res;
	}
	
	bool MyNavigator::isNextWaypointNeeded()
	{
		return m_new_navigation || isWaypointReached();
	}
	
	void MyNavigator::velocityCommandCallback(const geometry_msgs::Twist& cmd_vel)
	{
		m_cmd_vel = cmd_vel;
	}
};

PLUGINLIB_EXPORT_CLASS(testlib::MyNavigator, nav_core::BaseLocalPlanner)
