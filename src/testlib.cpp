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

		if (m_is_reactive_mrpt_finished && !isNewGoalReceived()) 
		{ 	// If here, mrpt has ended, the goal has not changed from previous plan and is the last one.
			// So, we only need to align the robot to the goal.
			endAlignment();
		} else if (!m_g_plan.empty() && isNextWaypointNeeded()) 
		{	// A new waypoint is needed. This starts a new navigation.
			int inc;
			m_localnh.param("index_waypoint", inc,WAYPOINT_INDEX_INCREMENT);
			m_waypoint_index += inc;
			m_is_last_waypoint = m_g_plan.size() <= m_waypoint_index;
			m_waypoint_index = m_is_last_waypoint ? m_g_plan.size() - 1 : m_waypoint_index;
			m_waypoint=m_g_plan[m_waypoint_index];
			ROS_INFO("\n\n[MyNavigator::computeVelocityCommands] sending goal to reactive navigator: Pose[x:%f,y:%f,z:%f]",
			 m_waypoint.pose.position.x,m_waypoint.pose.position.y,m_waypoint.pose.orientation.z);
			m_goal_pub.publish(m_waypoint);
			m_is_received_path = false;
		}

		cmd_vel = m_cmd_vel;
		bool not_clear_costmap = isInGoalPosition() || cmd_vel.linear.x >= MIN_VEL_VALUE 
			|| cmd_vel.angular.z >= MIN_VEL_VALUE;
		if (!not_clear_costmap)
			m_is_reactive_mrpt_finished = false;
		return not_clear_costmap;
	};

	bool MyNavigator::isInGoalPosition()
	{
		return m_is_last_waypoint && isWaypointReached();
	};
	
	bool MyNavigator::isGoalReached() 
	{

		//bool CWaypointsNavigator::checkHasReachedTarget 	( 	const double  	targetDist	) 	const
		// not protected, will need to find another way.
		// TODO
		bool end = isInGoalPosition() &&
					// Checks if reactive engine is sending null commands.
					// that way plugin and reactive can stop at the same time.
					m_cmd_vel.linear.x < MIN_VEL_VALUE &&
					m_cmd_vel.linear.y < MIN_VEL_VALUE;
		tf::Quaternion q = m_current_pose.getRotation(); 
		double yaw_curr = tf::getYaw(q), yaw_goal = tf::getYaw(m_waypoint.pose.orientation);
		double dist = yaw_curr - yaw_goal;
		end = end && sqrt(dist*dist) <= 0.04;
		if (end)
		{
			m_is_last_waypoint = false; // this enables next navigation.
			m_is_reactive_mrpt_finished = false;
			ROS_INFO("[MyNavigator::isGoalReached] \n\nGoal is reached. Navigation has ended.\n");
		}
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
		m_is_received_path = true;
		m_g_plan.clear();
		m_g_plan = plan;
		m_waypoint_index=0;
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
		//m_reactive = new ReactiveNavNode(0,a);


  		nodelet::M_string remap(ros::names::getRemappings());
  		nodelet::V_string nargv;
  		std::string nodelet_name = ros::this_node::getName();
  		nodelet.load("/move_base", "mrpt_reactivenav/mrpt_reactivenav_nodelet",
		   remap, nargv);



		// Storing costmap and tf to obtain robot pose later
		m_costmap_ros = costmap_ros;
		m_costmap_ros->getRobotPose(m_current_pose);
		m_tf = tf;
		// Check and get reactive goal param
		std::string topic_reactive_goal = "/reactive_nav_goal";
		std::string topic_end_nav = "end_nav_event";
		m_localnh.param("topic_relative_nav_goal", topic_reactive_goal,topic_reactive_goal);
		// Get publisher on reactive goal topic
		m_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>(topic_reactive_goal,1);
		// Get and check target_allowed_distance
		m_target_allowed_distance = 0.4;
		m_localnh.param(
			"target_allowed_distance", m_target_allowed_distance,
			m_target_allowed_distance);
		m_target_allowed_radians = 0.04;
		m_localnh.param(
			"target_allowed_radians", m_target_allowed_radians,
			m_target_allowed_radians);
		m_alignment_command = 0.3;
		m_localnh.param(
			"target_alignment_cmd_vel", m_alignment_command,
		m_alignment_command);
		m_localnh.param(
			"index_waypoint", WAYPOINT_INDEX_INCREMENT,
			50);
		ROS_INFO("[MyNavigator::initialize] WAYPOINT_INDEX_INCREMENT: %d", WAYPOINT_INDEX_INCREMENT);
		m_waypoint_index = 0;
		// Check and get topic name where reactive engine will publish vel commands.
		std::string topic_cmd_vel = "cmd_vel";
		m_localnh.param("topic_cmd_vel", topic_cmd_vel,topic_cmd_vel);
		// Subscribe to said topic and vincule callback
		m_pub_cmd_vel = m_nh.subscribe<const geometry_msgs::Twist&>(topic_cmd_vel, 1,
			&MyNavigator::velocityCommandCallback, this);
		
		m_pub_end_nav_event = m_nh.subscribe<const std_msgs::Bool&>(topic_end_nav, 1,
			&MyNavigator::endNavigationCallback, this);
		// flags initialization
		m_is_last_waypoint = false;
		m_is_received_path = false;
		m_is_reactive_mrpt_finished = false;
		
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
		if (!m_is_reactive_mrpt_finished)
			ROS_INFO(res? "\n\n\n[MyNavigator::isWaypointReached] Waypoint reached.\n\n"
					: "[MyNavigator::isWaypointReached] Waypoint not reached yet.");
		return res;
	}
	
	bool MyNavigator::isNextWaypointNeeded()
	{
		return m_is_received_path || isWaypointReached();
	}
	
	void MyNavigator::velocityCommandCallback(const geometry_msgs::Twist& cmd_vel)
	{
		m_cmd_vel = cmd_vel;
	}
	void MyNavigator::endNavigationCallback(const std_msgs::Bool& msg)
	{
		m_is_reactive_mrpt_finished = msg.data;
	}
	bool MyNavigator::endAlignment()
	{
		ROS_INFO("[MyNavigator::endAlignment] MRPT has finished, plugin is aligning to pose goal. Please wait.");
		geometry_msgs::Twist cmd_vel;
		tf::Quaternion q = m_current_pose.getRotation(); 
		double yaw_curr = tf::getYaw(q), yaw_goal = tf::getYaw(m_waypoint.pose.orientation);
		double dist= fmin(yaw_curr - yaw_goal, M_PI + yaw_goal - yaw_curr);
		if (dist > m_target_allowed_radians) 
		{
			cmd_vel.angular.z = -m_alignment_command;
		} else if (dist < -m_target_allowed_radians) 
		{
			cmd_vel.angular.z = m_alignment_command;
		}
		m_cmd_vel = cmd_vel;
	}
	bool MyNavigator::isNewGoalReceived() {
		auto last_pose_plan = m_g_plan[m_g_plan.size() - 1];
		double x=last_pose_plan.pose.position.x - m_waypoint.pose.position.x;
		double y=last_pose_plan.pose.position.y - m_waypoint.pose.position.y;
		double yaw_new = tf::getYaw(last_pose_plan.pose.orientation);
		double yaw_curr = tf::getYaw(m_waypoint.pose.orientation);
		double a = yaw_new - yaw_curr,b = M_PI + yaw_curr - yaw_new;
		double radians_diff = sqrt(fmin(a*a, b*b));
		return sqrt(x*x + y*y) > m_target_allowed_distance 
			&& radians_diff > m_target_allowed_radians;
	}
};

PLUGINLIB_EXPORT_CLASS(testlib::MyNavigator, nav_core::BaseLocalPlanner)
