#include "ros/ros.h"
#include <waypoint_checker/CheckCollision.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  waypoint_checker::CheckCollision srv;

  float x, y;	
  if (argc == 3){
      x = atof(argv[1]);
      y = atof(argv[2]);
  }	
  else
      ROS_ERROR("Usage: rosrun waypoint_checker test_service <x> <y>");	
  
  srv.request.x = x;
  srv.request.y = y;
  	  
  if (client.call(srv))
  {
    if (srv.response.collision)
	ROS_INFO("Point (%f, %f) is in collision!", (float)x, (float)y);
    else
        ROS_INFO("Point (%f, %f) is free!", (float)x, (float)y);
  }
  else
  {
    ROS_ERROR("Failed to call service shadow_heading");
    return 1;
  }

  return 0;
}



