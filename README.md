# SRC2-waypoint-checker

This node subscribes to topic ``/output`` to get a filtered, obstalce only, point cloud and provides a service that that checks if a given point is in collision with the point cloud.

The point cloud is transformed to frame ``odom``, so the waypoints tested must be in the frame.

To run the node:

``rosrun waypoint_checker waypoint_checker_node ``

A test/example code is provided. To run the test:

``rosrun waypoint_checker test_service x y`` 

where x and y must be replaced by the numerical values to be tested. 
