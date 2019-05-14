#ifndef TESTLIB__TESTLIB_H_
#define TESTLIB__TESTLIB_H_

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <test_lib/mrpt_reactivenav_node.h>

namespace testlib // this namespace will be changed.
{
	/**
	*	This class is a wrapper for ReactiveNavNode of mrpt navigation.
	*	For now the name is just a placeholder.
	*	Methods need to be filled looking at ReactiveNavNode, 
	*	inspired by TrajectoryPlanner and TrajectoryPlannerROS
	*
	*	TrajectoryPlannerRos: 
	*	https://github.com/ros-planning/navigation/blob/kinetic-devel/base_local_planner/include/base_local_planner/trajectory_planner_ros.h
	*	https://github.com/ros-planning/navigation/blob/kinetic-devel/base_local_planner/src/trajectory_planner_ros.cpp
	*
	*	TrajectoryPlanner:
	*	https://github.com/ros-planning/navigation/blob/kinetic-devel/base_local_planner/include/base_local_planner/trajectory_planner.h
	*	https://github.com/ros-planning/navigation/blob/kinetic-devel/base_local_planner/src/trajectory_planner.cpp
	**/
	class MyNavigator : public nav_core::BaseLocalPlanner 
	{
		private:
			ReactiveNavNode *m_reactive;
			ros::NodeHandle m_nh;
			ros::NodeHandle m_localnh{"~"}; // this is the way to initialize non const static attributes.
			// Publisher object to send waypoints to mrpt's reactive navigator.
			ros::Publisher m_goal_pub;
			// Subscribers:
			// Subscriber to receive velocity commands from mrpt's reactive navigator
			ros::Subscriber m_sub_cmd_vel;
			// Subscriber to receive end navigation event from mrpt's reactive navigator
			ros::Subscriber m_sub_end_nav_event;
			// The global plan sended by move_base.
			std::vector<geometry_msgs::PoseStamped> m_g_plan;
			// Robot pose. To check if waypoint is reached. TODO. Issue #9
			tf::Stamped<tf::Pose> m_current_pose;
			// Current waypoint.
			geometry_msgs::PoseStamped m_waypoint;
			// Pointer to current cmd_vel.
			geometry_msgs::Twist m_cmd_vel;
			// It is used to compute next waypoint from global plan.
			int WAYPOINT_INDEX_INCREMENT;
			int m_waypoint_index; // The starting waypoint for navigation. Each iteration will be incremented by WAYPOINT_INDEX_INCREMENT
			// Constant minimum vel command values allowed
			const double MIN_VEL_VALUE = .001; // Less than this is not showed by ReactiveNavEngine (MRPT) debug output.
			// Costmap received from move_base
			costmap_2d::Costmap2DROS* m_costmap_ros;
			// tfListener to be used with cloudpoints
			tf::TransformListener* m_tf;
			
			////////////////////// Parameters
			// Tolerance for x,y (meters) in computing goal (waypoint) completion.
			double m_target_allowed_distance;
			// Tolerance for yaw (radians) in computing goal (waypoint) completion.
			double m_target_allowed_radians;
			// Current velocity command when aligning the robot with goal.
			double m_alignment_command;
			// Flags to control the wrapper state.
			bool m_robot_pose_initialized;
			bool m_is_last_waypoint;
			bool m_is_received_path;
			bool m_is_reactive_mrpt_finished;
			////////////////////// Methods:

			void velocityCommandCallback(const geometry_msgs::Twist& cmd_vel);
			void endNavigationCallback(const std_msgs::Bool& msg);
			bool isWaypointReached();
			bool isNextWaypointNeeded();
			bool isInGoalPosition();
			bool isNewGoalReceived();
			bool endAlignment();
		public:
			/**
			* @brief  Default constructor for the ros wrapper
			*/
			MyNavigator();
			~MyNavigator();
			
			/**
			* @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
			* @param cmd_vel Will be filled with the velocity command to be passed to the robot base
			* @return True if a valid velocity command was found, false otherwise
			*/
			bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
			/**
			* @brief  Check if the goal pose has been achieved by the local planner
			* @return True if achieved, false otherwise
			*/
			bool isGoalReached() override;

			/**
			*	This needs to calculate a plan guided by plan
			*	@arg-global_plan: Vector of poses presumably sent by global planner
			*	
			* @brief  Set the plan that the local planner is following
			* @param plan The plan to pass to the local planner
			* @return True if the plan was updated successfully, false otherwise
			**/
			bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) override;
			/**
			* @brief  Constructs the local planner
			* @param name The name to give this instance of the local planner
			* @param tf A pointer to a transform listener
			* @param costmap_ros The cost map to use for assigning costs to local plans
 			*/
			void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

      
	};
};



#endif
