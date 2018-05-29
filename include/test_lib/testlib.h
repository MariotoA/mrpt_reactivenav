#ifndef TESTLIB__TESTLIB_H_
#define TESTLIB__TESTLIB_H_

#include <geometry_msgs/PoseStamped.h>
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
	*	Methods need to be filled looking at ReactiveNavNode, TrajectoryPlanner and TrajectoryPlannerROS
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
			ReactiveNavNode *reactive;
			ros::NodeHandle m_nh;
			ros::Publisher goal_pub;
			ros::Subscriber pose_sub;
			std::vector<geometry_msgs::PoseStamped> g_plan;
			geometry_msgs::PoseStamped robot_pose_;
		public:
			/**
			* @brief  Default constructor for the ros wrapper
			*/
			MyNavigator();
		 	MyNavigator(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
			MyNavigator(int argc, char **args);
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
			*	@arg-global_plan: Vector of poses presumably sended by global planner
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


			void poseCallback(geometry_msgs::PoseStamped robotPose);

      
	};
};



#endif
