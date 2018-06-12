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
		// Trouble when user cancels navigation.
		if (!m_g_plan.empty() && isNextWaypointNeeded()) 
		{
			m_is_last_waypoint = m_g_plan.size() < WAYPOINT_INDEX;
			int ind = m_is_last_waypoint ? m_g_plan.size() - 1 : WAYPOINT_INDEX;
			m_waypoint=m_g_plan[ind];
			ROS_INFO("\n\nMyNavigator::sending goal to reactive navigator: Pose[x:%f,y:%f,z:%f]",
			 m_waypoint.pose.position.x,m_waypoint.pose.position.y,m_waypoint.pose.position.z);
            m_goal_pub.publish(m_waypoint);
            m_waypoint_initialized = true;
		}
		publishPlan();
		
		return true;
	};

	bool MyNavigator::isGoalReached() 
	{

		//bool CWaypointsNavigator::checkHasReachedTarget 	( 	const double  	targetDist	) 	const
		// not protected, will need to find another way.
		// TODO
		bool end = m_is_last_waypoint && isWaypointReached();
		if (end)
			m_robot_pose_initialized = m_waypoint_initialized = m_is_last_waypoint = false;
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
		
		// Checks if it is first initialization. We suppose is at least 1 in size when goal is sent.
		m_new_navigation = !m_g_plan.empty() 
			&& abs(m_g_plan.back().pose.position.x - plan.back().pose.position.x) > .01 
			&& abs(m_g_plan.back().pose.position.y - plan.back().pose.position.y) > .01 ;
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
		m_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/reactive_nav_goal",1); // this should be using the param TODO
		m_trajectory = m_localnh.advertise<nav_msgs::Path>("/trajectory_generated",1);
		m_goal_move_base_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,
			&MyNavigator::goalMoveBaseCallback,this);
		m_pose_sub = m_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1,
			&MyNavigator::poseCallback, this);
		m_cmd_vel_sub = m_nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&MyNavigator::velCallback,this);
		m_target_allowed_distance = 0.4;
		m_localnh.param(
			"target_allowed_distance", m_target_allowed_distance,
			m_target_allowed_distance);
		m_robot_pose_initialized = false;
		m_waypoint_initialized = false;
		m_is_last_waypoint = false;
		m_new_navigation = false;
		
	};

	void MyNavigator::poseCallback(geometry_msgs::PoseWithCovarianceStamped robot_pose) 
	{
		m_robot_pose_.pose = robot_pose.pose.pose;
		m_robot_pose_.header = robot_pose.header;
		m_robot_pose_initialized = true;
	}
	
	bool MyNavigator::isWaypointReached()
	{
		if (!ros::isInitialized()) 
		{
      			ROS_ERROR("[MyNavigator::isWaypointReached] This planner has not been initialized, please call initialize() before using this planner");
      			return false;
		}

		if (!m_robot_pose_initialized) 
		{
      			ROS_ERROR("[MyNavigator::isWaypointReached] This planner has not received robot pose yet.");
      			return false;
		}
		double x=m_robot_pose_.pose.position.x-m_waypoint.pose.position.x;
		double y=m_robot_pose_.pose.position.y-m_waypoint.pose.position.y;
		bool res = sqrt(x*x + y*y) <= m_target_allowed_distance;
		if (res)
		{
			ROS_INFO("\n\n\n[MyNavigator::isWaypointReached] Waypoint reached.\n\n");	
		} else 
		{
			ROS_INFO("[MyNavigator::isWaypointReached] Waypoint not reached yet.");	
		}
		return res;
	}
	
	bool MyNavigator::isNextWaypointNeeded()
	{
		return m_new_navigation || !m_waypoint_initialized || isWaypointReached();
	}
	
	void MyNavigator::goalMoveBaseCallback(const geometry_msgs::PoseStampedConstPtr& goal) 
	{
		ROS_INFO("\n\n\n[MyNavigator::goalMoveBaseCallback] NEW GOAL!\n\n");
	}

	void MyNavigator::publishPlan() 
	{
	
	}

	void MyNavigator::velCallback(const geometry_msgs::TwistConstPtr& cmd_vel) 
	{
		if (!ros::isInitialized()) 
		{
      			ROS_ERROR("[MyNavigator::velCallback] This planner has not been initialized, please call initialize() before using this planner");
      			return;
		} else if (!m_robot_pose_initialized) 
		{
			ROS_ERROR("[MyNavigator::velCallback] robot pose has not been initialized.");
			return;
		}
		m_cmd_vel = cmd_vel;
		double vx = cmd_vel->linear.x;
		double vy = cmd_vel->linear.y;
		double w = cmd_vel->angular.z;
		double v = sqrt(vx*vx+vy*vy);
		geometry_msgs::PoseStamped cur_pose = m_robot_pose_, prev_pose;
		std::unique_ptr<std::vector<geometry_msgs::PoseStamped>> trajectory;
		trajectory->push_back(cur_pose);
		double end_time = 5, dt = .1; // TODO Change to nav_period
		double th;
		
		ROS_INFO("\n\n\n[MyNavigator::velCallback] NEW VEL COMMAND: Init !\n\n");
		for (double time = 0; time < end_time; time+=dt)
		{
			prev_pose = cur_pose;
			th = cur_pose.pose.orientation.z;
			// Odometry to simulate trajectory
			cur_pose.pose.position.x += (vx * cos(th) - vy * sin(th)) * dt;
			cur_pose.pose.position.y += (vx * sin(th) + vy * cos(th)) * dt;
			cur_pose.pose.orientation.z += w*dt;
			cur_pose.header.seq++;
			cur_pose.header.stamp = ros::Time::now();
			trajectory->push_back(cur_pose);
		}

		nav_msgs::Path msg;
		msg.header = cur_pose.header;
		msg.header.stamp = ros::Time::now();
		msg.poses = *(trajectory);
		m_trajectory.publish(msg);
		ROS_INFO("\n\n\n[MyNavigator::velCallback] NEW VEL COMMAND: vx=%f,vy=%f,w=%f !\n\n",vx,vy, w);
		
	}
};


PLUGINLIB_EXPORT_CLASS(testlib::MyNavigator, nav_core::BaseLocalPlanner)
int main(int argc, char **argv)
{
    testlib::MyNavigator  the_node(argc, argv);
	ros::spin();
	return 0;
}
