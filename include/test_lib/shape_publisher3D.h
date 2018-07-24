#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>


#include <memory>
#include <iostream>
#include <string>
#include <cstdio>

namespace shape {
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
	using namespace std; //Don't if you're in a header-file
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}


bool getTRobotShape(mrpt::nav::TRobotShape& robotShape, const ros::NodeHandle& nh)
{
	int levels=1, initial=0;
	nh.param<int>("levels", levels, levels);
	nh.param<int>("initial", initial, initial);
	ROS_INFO("[shape::getTRobotShape] levels: %d, initial: %d",levels,initial);
	robotShape.resize(levels);
	for (size_t i = 0; i < levels; i++)
	{
		std::string height_str = string_format("height_%d", initial);
		
		ROS_INFO("[shape::getTRobotShape] height string: %s",height_str.c_str());
		std::string points_x_str = string_format("shape_x%d", initial);
		
		ROS_INFO("[shape::getTRobotShape] points_x string: %s",points_x_str.c_str());
		std::string points_y_str = string_format("shape_y%d", initial);

		ROS_INFO("[shape::getTRobotShape] points_y string: %s",points_y_str.c_str());
		double height;
		std::vector<double> points_x,points_y;
		
		if (!nh.getParam(height_str, height) ||
		!nh.getParam(points_x_str, points_x) ||
		!nh.getParam(points_y_str, points_y) )
		{
			ROS_INFO("[shape::getTRobotShape] FAILURE");
			return false;
		}

		robotShape.polygon(i).setAllVertices(points_x,points_y);
		
			ROS_INFO("[shape::getTRobotShape] Setting height ");
		robotShape.setHeight(i, height);
		initial++;
	}
	return true;
}
}
