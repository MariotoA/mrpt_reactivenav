#include <test_lib/mrpt_reactivenav_node.h>

ReactiveNavNode::ReactiveNavNode(int argc, char** argv)
		: m_auxinit(argc, argv),
		  m_nh(),
		  m_localn("~"),
		  m_1st_time_init(false),
		  m_target_allowed_distance(0.40f),
		  m_nav_period(0.100),
		  m_pub_topic_reactive_nav_goal("reactive_nav_goal"),
		  m_pub_topic_end_nav_event("end_nav_event"),
		  m_sub_topic_local_obstacles("local_map_pointcloud"),
		  m_sub_topic_robot_shape(""),
		  m_pub_topic_cmd_vel("cmd_vel"),
		  m_frameid_reference("/map"),
		  m_frameid_robot("base_link"),
		  m_save_nav_log(false),
		  m_reactive_if(*this)
	{
		// Load params:
		std::string cfg_file_reactive;
        m_localn.param("nav_type", nav_type, nav_type); // could be 2D or 3D

        if( nav_type == "2D" )
        	m_reactive_nav_engine.reset(new CReactiveNavigationSystem(m_reactive_if));
        else if ( nav_type == "3D" )
        	m_reactive_nav_engine.reset(new CReactiveNavigationSystem3D(m_reactive_if));
        else
        {
        	ROS_INFO("Incorrect navigation type. It should be 2D or 3D");
        	return;
        }
		m_localn.param(
			"cfg_file_reactive", cfg_file_reactive, cfg_file_reactive);
		m_localn.param(
			"target_allowed_distance", m_target_allowed_distance,
			m_target_allowed_distance);
		m_localn.param("nav_period", m_nav_period, m_nav_period);
		m_localn.param(
			"frameid_reference", m_frameid_reference, m_frameid_reference);
		m_localn.param("frameid_robot", m_frameid_robot, m_frameid_robot);
		m_localn.param(
			"topic_robot_shape", m_sub_topic_robot_shape,
			m_sub_topic_robot_shape);
		m_localn.param(
			"topic_relative_nav_goal", m_pub_topic_reactive_nav_goal,
			m_pub_topic_reactive_nav_goal);
		m_localn.param(
			"topic_end_navigation", m_pub_topic_end_nav_event,
			m_pub_topic_end_nav_event);
		m_localn.param("save_nav_log", m_save_nav_log, m_save_nav_log);
		m_localn.param("topic_cmd_vel", m_pub_topic_cmd_vel,m_pub_topic_cmd_vel);
		ROS_ASSERT(m_nav_period > 0);
		ROS_ASSERT_MSG(
			!cfg_file_reactive.empty(),
			"Mandatory param 'cfg_file_reactive' is missing!");
		ROS_ASSERT_MSG(
			mrpt::system::fileExists(cfg_file_reactive),
			"Config file not found: '%s'", cfg_file_reactive.c_str());

		m_reactive_nav_engine->enableLogFile(m_save_nav_log);

		// Load reactive config:
		// ----------------------------------------------------
		try
		{
			CConfigFile cfgFil(cfg_file_reactive);
			m_reactive_nav_engine->loadConfigFile(cfgFil);
		}
		catch (std::exception& e)
		{
			ROS_ERROR(
				"Exception initializing reactive navigation engine:\n%s",
				e.what());
			throw;
		}

		// load robot shape: (1) default, (2) via params, (3) via topic
		// ----------------------------------------------------------------
		// m_reactive_nav_engine->changeRobotShape();

		// Init this subscriber first so we know asap the desired robot shape,
		// if provided via a topic:
		if (!m_sub_topic_robot_shape.empty() && nav_type == "2D")
		{
			m_sub_robot_shape = m_nh.subscribe<geometry_msgs::Polygon>(
				m_sub_topic_robot_shape, 1,
				&ReactiveNavNode::onRosSetRobotShape, this);
			ROS_INFO(
				"Params say robot shape will arrive via topic '%s'... waiting "
				"3 seconds for it.",
				m_sub_topic_robot_shape.c_str());
			ros::Duration(3.0).sleep();
			for (size_t i = 0; i < 100; i++) ros::spinOnce();
			ROS_INFO("Wait done.");
		} else if (!m_sub_topic_robot_shape.empty())
		{
			/*mrpt::nav::TRobotShape rob;
			if (shape::getTRobotShape(rob,m_localn))
				onRosSetRobotShape3D(rob);
			else
				ROS_INFO("Could not read 3D shape");*/
		}

		// Init ROS publishers:
		// -----------------------
		m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>(m_pub_topic_cmd_vel, 1);
		m_pub_end_nav = m_nh.advertise<std_msgs::Bool>(m_pub_topic_end_nav_event, 1);

		// Init ROS subs:
		// -----------------------
		// "/reactive_nav_goal", "/move_base_simple/goal" (
		// geometry_msgs/PoseStamped )
		m_sub_nav_goal = m_nh.subscribe<geometry_msgs::PoseStamped>(
			m_pub_topic_reactive_nav_goal, 1,
			&ReactiveNavNode::onRosGoalReceived, this);
		m_sub_local_obs = m_nh.subscribe<sensor_msgs::PointCloud2>(
			m_sub_topic_local_obstacles, 1,
			&ReactiveNavNode::onRosLocalObstacles, this);

		// Init timers:
		// ----------------------------------------------------
		m_timer_run_nav = m_nh.createTimer(
			ros::Duration(m_nav_period), &ReactiveNavNode::onDoNavigation,
			this);

}  // end ctor

	void ReactiveNavNode::navigateTo(const mrpt::math::TPose2D& target)
	{
		ROS_INFO(
			"[ReactiveNavNode::navigateTo] Starting navigation to %s",
			target.asString().c_str());
		
		CAbstractPTGBasedReactive::TNavigationParamsPTG navParams;
		CAbstractNavigator::TargetInfo target_info, target_info_mult;
		target_info.target_coords.x = target_info_mult.target_coords.x = target.x;
		target_info.target_coords.y = target_info_mult.target_coords.y = target.y;
		// Added to test
		target_info.target_coords.phi = target_info_mult.target_coords.phi = target.phi;
		ROS_INFO("navigateTo :: TARGET_INFO = %s, safeDist= %f", target.asString().c_str(), m_target_allowed_distance);
		//
		target_info.targetAllowedDistance  = target_info_mult.targetAllowedDistance = m_target_allowed_distance;
		target_info.targetIsRelative = target_info_mult.targetIsRelative = false;

		navParams.multiple_targets.push_back(target_info_mult);
		navParams.target = target_info;
		//navParams.target = target_info;
		// Optional: restrict the PTGs to use
		// navParams.restrict_PTG_indices.push_back(1);

		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine->navigate(&navParams);
		}
	}


void ReactiveNavNode::onDoNavigation(const ros::TimerEvent&)
	{
		// 1st time init:
		// ----------------------------------------------------
		if (!m_1st_time_init)
		{
			m_1st_time_init = true;
			ROS_INFO(
				"[ReactiveNavNode] Initializing reactive navigation "
				"engine...");
			{
				std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
				m_reactive_nav_engine->initialize();
			}
			ROS_INFO(
				"[ReactiveNavNode] Reactive navigation engine init done!");
		}

		CTimeLoggerEntry tle(m_profiler, "onDoNavigation");
		m_reactive_nav_engine->navigationStep();
	}

	void ReactiveNavNode::onRosGoalReceived(const geometry_msgs::PoseStampedConstPtr& trg_ptr) // TODO
	{
		geometry_msgs::PoseStamped trg = *trg_ptr;
		ROS_INFO(
			"[ReactiveNavNode::onRosGoalReceived] Nav target received via topic sub: (%.03f,%.03f, %.03fdeg) "
			"[frame_id=%s]",
			trg.pose.position.x, trg.pose.position.y,
			trg.pose.orientation.z * 180.0 / M_PI, trg.header.frame_id.c_str());

		// Convert to the "m_frameid_reference" frame of coordinates:
		if (trg.header.frame_id != m_frameid_reference)
		{
			try
			{
				geometry_msgs::PoseStamped trg2;
				m_tf_listener.transformPose(m_frameid_reference, trg, trg2);
				trg = trg2;
			}
			catch (tf::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return;
			}
		}
			double yaw_angle = tf::getYaw(trg.pose.orientation);
			ROS_INFO("[ReactiveNavNode::onRosGoalReceived] yaw_angle=%f", yaw_angle);




			this->navigateTo(mrpt::math::TPose2D(trg.pose.position.x, trg.pose.position.y, yaw_angle));
	}

	void ReactiveNavNode::onRosLocalObstacles(const sensor_msgs::PointCloud2ConstPtr& obs)
	{
		std::lock_guard<std::mutex> csl(m_last_obstacles_cs);
		mrpt_bridge::copy(*obs, m_last_obstacles);
		// ROS_DEBUG("Local obstacles received: %u points", static_cast<unsigned
		// int>(m_last_obstacles.size()) );
	}

	void ReactiveNavNode::onRosSetRobotShape(const geometry_msgs::PolygonConstPtr& newShape)
	{
		ROS_INFO_STREAM("[ReactiveNavNode::onRosSetRobotShape] Robot shape received via topic: " <<  *newShape );

                CReactiveNavigationSystem* rns2D = dynamic_cast<CReactiveNavigationSystem*>(m_reactive_nav_engine.get());

                if( rns2D )
                {
                    mrpt::math::CPolygon poly;
                    poly.resize(newShape->points.size());
                    for (size_t i=0;i<newShape->points.size();i++)
                    {
                            poly[i].x = newShape->points[i].x;
                            poly[i].y = newShape->points[i].y;
                    }

                    {
                        std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
                        rns2D->changeRobotShape(poly);
                    }
                }
	}
	// Not a callback. Used on constructor
	void ReactiveNavNode::onRosSetRobotShape3D(const mrpt::nav::TRobotShape& newShape)
	{
		ROS_INFO_STREAM("[ReactiveNavNode::onRosSetRobotShape3D] Robot shape received via params");
		CReactiveNavigationSystem3D* rns3D = dynamic_cast<CReactiveNavigationSystem3D*>(m_reactive_nav_engine.get());
		if (rns3D)
		{
			{
				std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
                rns3D->changeRobotShape(newShape);
				for (size_t l = 0; l < newShape.size(); l++)
				{
					std::vector< double > x,y;
					newShape.polygon(l).getAllVertices(x, y);
					double hei = newShape.getHeight(l);
					double rad = newShape.getRadius(l);
					for (int i = 0; i < x.size(); i++) 
					{
						ROS_INFO("[ReactiveNavNode::onRosSetRobotShape3D] level: %d, x:%f, y:%f",(int) l, x[i], y[i]);
					}
					ROS_INFO("[ReactiveNavNode::onRosSetRobotShape3D] level: %d, h:%f", (int) l, hei);
					ROS_INFO("[ReactiveNavNode::onRosSetRobotShape3D] level: %d, r:%f", (int) l, rad);
				}
            }
		}
			
			
	}

int main(int argc, char **argv)
{
        ReactiveNavNode  the_node(argc, argv);
	ros::spin();
	return 0;
}



