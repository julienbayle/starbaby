/*
 * starbaby_trilateration_points_tracking.cpp
 *
 *  Created on: 18 mars 2018
 *      Author: Yann BOURRIGAULT
 */

#include <sensor_msgs/LaserScan.h>
#include <filters/filter_chain.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/shared_ptr.hpp>

boost::shared_ptr<filters::FilterChain<sensor_msgs::LaserScan> > laser_scan_filter_chain;

boost::shared_ptr<tf::TransformListener> tf_listener;
boost::shared_ptr<laser_geometry::LaserProjection> laser_projection;

ros::Publisher filtered_points_publisher;

void laser_scan_callback(const sensor_msgs::LaserScanConstPtr& scan) {
	sensor_msgs::LaserScan filtered_scan;

	// Keep only scans with intensity above threshold
	laser_scan_filter_chain->update (*scan, filtered_scan);

	// Convert scan to point cloud in the odom frame
	if(!tf_listener->waitForTransform(
			scan->header.frame_id,
			"odom",
			scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
			ros::Duration(1.0))){
		return;
	}

	// Create point cloud
	sensor_msgs::PointCloud2 cloud_msg;
	laser_projection->transformLaserScanToPointCloud("odom", filtered_scan, cloud_msg, *tf_listener);

	// Container for original & filtered PCL data
	pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr pcl_cloudPtr(pcl_cloud);
	pcl::PCLPointCloud2 pcl_cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(cloud_msg, *pcl_cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (pcl_cloudPtr);
	sor.setLeafSize (0.5, 0.5, 0.5);
	sor.filter (pcl_cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(pcl_cloud_filtered, output);

	// Publish the data
	filtered_points_publisher.publish(output);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "starbaby_trilateration_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// Init pointers
	laser_scan_filter_chain.reset(new filters::FilterChain<sensor_msgs::LaserScan>("sensor_msgs::LaserScan"));
	tf_listener.reset(new tf::TransformListener());
	laser_projection.reset(new laser_geometry::LaserProjection());

	// Init laser scan filters with private node parameters
	laser_scan_filter_chain->configure("laser_scan_filters", nh_priv);

	// Init publishers
	filtered_points_publisher = nh_priv.advertise<sensor_msgs::PointCloud2>("filtered_points", 1, false);

	// Init subscribers
	ros::Subscriber laser_scan_subscriber = nh_priv.subscribe("laser_scan", 1, &laser_scan_callback);

	// Spin
	ros::spin();

	return 0;
}
