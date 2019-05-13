#ifndef TESTLIB__MRPT_REACTIVENAV_NODE_H_
#define TESTLIB__MRPT_REACTIVENAV_NODE_H_
/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014-2015, Jose-Luis Blanco <jlblanco@ual.es> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. *
 ***********************************************************************************/

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <mrpt/version.h>

// Use modern headers ------------
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
using namespace mrpt::nav;
using mrpt::maps::CSimplePointsMap;

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFile.h>
using namespace mrpt::system;
using namespace mrpt::config;
#else
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFile.h>
using namespace mrpt::utils;
#endif

#include <mrpt/system/filesystem.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/point_cloud.h>
#include <mrpt_bridge/point_cloud2.h>
#include <mrpt_bridge/time.h>

#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>

#include <mutex>
#include <mrpt/nav/reactive/TWaypoint.h>
// The ROS node
class ReactiveNavNode
{
   private:
	struct TAuxInitializer
	{
		TAuxInitializer(int argc, char** argv)
		{
			ros::init(argc, argv, "mrpt_reactivenav");
		}
	};

	CTimeLogger m_profiler;
	TAuxInitializer m_auxinit;  //!< Just to make sure ROS is init first
	ros::NodeHandle m_nh;  //!< The node handle
	ros::NodeHandle m_localn;  //!< "~"

	/** @name ROS pubs/subs
	 *  @{ */
	ros::Subscriber m_sub_nav_goal;
	ros::Subscriber m_sub_local_obs;
	ros::Subscriber m_sub_robot_shape;
	ros::Publisher m_pub_cmd_vel;
	ros::Publisher m_pub_end_nav;
	tf::TransformListener m_tf_listener;  //!< Use to retrieve TF data
	/** @} */

	bool m_1st_time_init;  //!< Reactive initialization done?
	double m_target_allowed_distance;
	double m_nav_period;

	std::string m_pub_topic_reactive_nav_goal;
	std::string m_sub_topic_local_obstacles;
	std::string m_sub_topic_robot_shape;
	std::string m_pub_topic_cmd_vel;
	std::string m_pub_topic_end_nav_event;	

	std::string m_frameid_reference;
	std::string m_frameid_robot;

	std::string nav_type="";

	bool m_save_nav_log;
	ros::Timer m_timer_run_nav;
	CSimplePointsMap m_last_obstacles;
	std::mutex m_last_obstacles_cs;

	struct MyReactiveInterface :

		public mrpt::nav::CRobot2NavInterface
	{
		ReactiveNavNode& m_parent;
		MyReactiveInterface(ReactiveNavNode& parent) : m_parent(parent) {}
		/** Get the current pose and speeds of the robot.
		 *   \param curPose Current robot pose.
		 *   \param curV Current linear speed, in meters per second.
		 *	 \param curW Current angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool getCurrentPoseAndSpeeds(
			mrpt::math::TPose2D& curPose, mrpt::math::TTwist2D& curVel,
			mrpt::system::TTimeStamp& timestamp,
			mrpt::math::TPose2D& curOdometry, std::string& frame_id) override
		{
			double curV, curW;

			CTimeLoggerEntry tle(
				m_parent.m_profiler, "getCurrentPoseAndSpeeds");
			tf::StampedTransform txRobotPose;
			try
			{
				CTimeLoggerEntry tle2(
					m_parent.m_profiler,
					"getCurrentPoseAndSpeeds.lookupTransform_sensor");
				m_parent.m_tf_listener.lookupTransform(
					m_parent.m_frameid_reference, m_parent.m_frameid_robot,
					ros::Time(0), txRobotPose);
			}
			catch (tf::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return false;
			}

			mrpt::poses::CPose3D curRobotPose;
			mrpt_bridge::convert(txRobotPose, curRobotPose);

			mrpt_bridge::convert(txRobotPose.stamp_, timestamp);
			// Explicit 3d->2d to confirm we know we're losing information
			curPose =
#if MRPT_VERSION >= 0x199
				mrpt::poses::CPose2D(curRobotPose).asTPose();
#else
				mrpt::math::TPose2D(mrpt::poses::CPose2D(curRobotPose));
#endif
			curOdometry = curPose;

			curV = curW = 0;
			MRPT_TODO("Retrieve current speeds from odometry");
			ROS_INFO(
				"[MyReactiveInterface::getCurrentPoseAndSpeeds] Latest pose: %s",
				curPose.asString().c_str());

			curVel.vx = curV * cos(curPose.phi);
			curVel.vy = curV * sin(curPose.phi);
			curVel.omega = curW;

			return true;
		}

		/** Change the instantaneous speeds of robot.
		 *   \param v Linear speed, in meters per second.
		 *	 \param w Angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool changeSpeeds(
			const mrpt::kinematics::CVehicleVelCmd& vel_cmd) override
		{
			using namespace mrpt::kinematics;
			const CVehicleVelCmd_DiffDriven* vel_cmd_diff_driven =
				dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
			ASSERT_(vel_cmd_diff_driven);

			const double v = vel_cmd_diff_driven->lin_vel;
			const double w = vel_cmd_diff_driven->ang_vel;
			ROS_DEBUG(
				"changeSpeeds: v=%7.4f m/s  w=%8.3f deg/s", v,
				w * 180.0f / M_PI);
			geometry_msgs::Twist cmd;
			cmd.linear.x = v;
			cmd.angular.z = w;
			//if (v >= 0.001 || w >= 0.001)
			
			m_parent.m_pub_cmd_vel.publish(cmd);
			return true;
		}

		bool stop(bool isEmergency) override
		{
			/*mrpt::kinematics::CVehicleVelCmd_DiffDriven vel_cmd;
			vel_cmd.lin_vel = 0;
			vel_cmd.ang_vel = 0;
			*/
			geometry_msgs::Twist cmd;
			cmd.linear.x = 0;
			cmd.angular.z = 0;
			m_parent.m_pub_cmd_vel.publish(cmd);
			
			return true ; //changeSpeeds(vel_cmd);
		}

		/** Start the watchdog timer of the robot platform, if any.
		 * \param T_ms Period, in ms.
		 * \return false on any error. */
		virtual bool startWatchdog(float T_ms) override { return true; }
		/** Stop the watchdog timer.
		 * \return false on any error. */
		virtual bool stopWatchdog() override { return true; }
		/** Return the current set of obstacle points.
		 * \return false on any error. */
		bool senseObstacles(
			CSimplePointsMap& obstacles,
			mrpt::system::TTimeStamp& timestamp) override
		{
			timestamp = mrpt::system::now();
			std::lock_guard<std::mutex> csl(m_parent.m_last_obstacles_cs);
			obstacles = m_parent.m_last_obstacles;

			MRPT_TODO("TODO: Check age of obstacles!");
			return true;
		}

		mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override
		{
			return getStopCmd();
		}
		mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override
		{
			mrpt::kinematics::CVehicleVelCmd::Ptr ret =
				mrpt::kinematics::CVehicleVelCmd::Ptr(
					new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
			ret->setToStop();
			return ret;
		}

		mrpt::kinematics::CVehicleVelCmd::Ptr getAlignCmd(const double relative_heading_radians)
		override
		{
			mrpt::kinematics::CVehicleVelCmd_DiffDriven::Ptr ret =
				mrpt::kinematics::CVehicleVelCmd_DiffDriven::Ptr(
					new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
			ROS_INFO("\n\n[MyReactiveInterface::getAlignCmd] Called.\n\n");
			if (relative_heading_radians - .01 < 0)
			{
				ret->ang_vel =  -0.5;
			} else if (relative_heading_radians + .01 > 0)
			{
				ret->ang_vel = 0.5;
			}
			return ret;
		}
		
		void sendNavigationEndEvent() 
		{
			std_msgs::Bool msg;
			msg.data = true;
			ROS_INFO("[MyReactiveInterface::sendNavigationEndEvent] Finishing navigation...");
			m_parent.m_pub_end_nav.publish(msg);
			
		}
		void sendNavigationStartEvent() {
			std_msgs::Bool msg;
			msg.data = false;
			ROS_INFO("[MyReactiveInterface::sendNavigationStartEvent] Starting navigation...");
			m_parent.m_pub_end_nav.publish(msg);
		}
	};

	MyReactiveInterface m_reactive_if;

	std::unique_ptr<CAbstractPTGBasedReactive> m_reactive_nav_engine;
	std::mutex m_reactive_nav_engine_cs;

   public:
	/**  Constructor: Inits ROS system */
	ReactiveNavNode(int argc, char** argv);

	/**
	 * @brief Issue a navigation command
	 * @param target The target location
	 */
	void navigateTo(const mrpt::math::TPose2D& target);

	/** Callback: On run navigation */
	void onDoNavigation(const ros::TimerEvent&);
	void onRosGoalReceived(const geometry_msgs::PoseStampedConstPtr& trg_ptr);
	void onRosLocalObstacles(const sensor_msgs::PointCloud2ConstPtr& obs);
	void onRosSetRobotShape(const geometry_msgs::PolygonConstPtr& newShape);
	// Not a callback. Used on constructor
	void onRosSetRobotShape3D(const mrpt::nav::TRobotShape& newShape);

};  // end class

#endif
