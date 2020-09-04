#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <Eigen/Dense>
#include <math.h>
#include <waypoint_checker/CheckCollision.h>

std::string robot_name;
std::string odometry_frame_id;
std::string odometry_child_frame_id;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  tf2_ros::Buffer tfBuffer;
  geometry_msgs::TransformStamped Tt2_v;
  sensor_msgs::PointCloud2 trns_cloud_msg;
  tf2_ros::TransformListener tf2_listener(tfBuffer);

  // change frame of the point cloud

  try{

  	Tt2_v = tfBuffer.lookupTransform(odometry_frame_id, (*cloud_msg).header.frame_id, ros::Time(0), ros::Duration(1.0));
	//Tt2_v = tfBuffer.lookupTransform("scout_1_tf/base_footprint", (*cloud_msg).header.frame_id, ros::Time(0));
  	tf2::doTransform(*cloud_msg, trns_cloud_msg, Tt2_v);

  	// Convert from ROS to PCL data type
  	pcl::fromROSMsg (trns_cloud_msg, *cloud);

  	// This is necessary
  	std::vector<int> ind;
  	pcl::removeNaNFromPointCloud(*cloud, *cloud, ind);

  } // try
  catch (tf::TransformException ex){
  	ROS_ERROR("%s",ex.what());
  	ros::Duration(0.1).sleep();
  } // catch
}


bool serviceCallback(waypoint_checker::CheckCollision::Request &req, waypoint_checker::CheckCollision::Response &resp)
{

	ROS_INFO("Service Called");

	resp.collision = false;

        ROS_INFO("Cloud Size: %d", (int)cloud->size());

        if (cloud->size() == 0)  // If there is no point in the cloud, there is no collision
		return true;


   	// Perform the actual clustering
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.setClusterTolerance (0.25);
  	ec.setMinClusterSize (150);
  	ec.extract (cluster_indices); // Does the work

	ROS_INFO("Number of clusters: %d", (int)cluster_indices.size());

	int j = 0;

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){ // iteract over all clusters

		// each cluster is represented by it
      		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); pit++){
          		cloud_cluster->points.push_back(cloud->points[*pit]);
      		}
      		cloud_cluster->width = int (cloud_cluster->points.size ());
      		cloud_cluster->height = 1;
      		cloud_cluster->is_dense = true;

      		// Find the centroid, max and min of the cluster (X, Y, Z)

      		Eigen::Vector4f centroid, max_pt, min_pt;
      		pcl::compute3DCentroid (*cloud_cluster, centroid);
		pcl::getMinMax3D (*cloud_cluster, min_pt, max_pt);

		// Find the radius of the cluster -> We assume it is symetric. A rock, for example.
		float dx = max_pt[0]-min_pt[0];
		float dy = max_pt[1]-min_pt[1];
		float radius = dx>dy?dx/2.0:dy/2.0; // We may need to increase the radio a bit to consider the size of the robot


                if ( (req.x > (centroid[0] - radius)) && (req.x < (centroid[0] + radius)) && (req.y > (centroid[1] - radius)) && (req.y < (centroid[1] + radius)) ){
			resp.collision = true;
			return true;
		}

	}

	return true;

}



int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "waypoint_checker");
	ros::NodeHandle nh;
	std::string node_name = "waypoint_checker_node";

	if(ros::param::get(node_name+"/odometry_frame_id",odometry_frame_id)==false)
	{
		ROS_FATAL("No parameter 'odometry_frame_id' specified");
		ros::shutdown();
		exit(1);
	}
	if(ros::param::get(node_name+"/odometry_child_frame_id",odometry_child_frame_id)==false)
	{
		ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
		ros::shutdown();
		exit(1);
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("inference/point_cloud", 1, cloud_cb);

	// Create a ROS service for the input point cloud
	ros::ServiceServer service = nh.advertiseService("waypoint_checker", serviceCallback);

	// Spin
	ros::spin ();
}
