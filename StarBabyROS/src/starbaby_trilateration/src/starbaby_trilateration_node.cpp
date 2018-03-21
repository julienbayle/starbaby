/*
 * starbaby_trilateration_points_tracking.cpp
 *
 *  Created on: 18 mars 2018
 *      Author: Yann BOURRIGAULT
 */

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <filters/filter_chain.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

#include <pcl_conversions/pcl_conversions.h>

#include <boost/shared_ptr.hpp>

boost::shared_ptr<filters::FilterChain<sensor_msgs::LaserScan> > laser_scan_filter_chain;

boost::shared_ptr<tf::TransformListener> tf_listener;
boost::shared_ptr<laser_geometry::LaserProjection> laser_projection;

ros::Publisher filtered_points_publisher;
ros::Publisher estimated_pos_publisher;

sensor_msgs::LaserScanConstPtr last_scan;

void laser_scan_callback(const sensor_msgs::LaserScanConstPtr& scan) {
	last_scan = scan;
}

void handle_scan() {
	if (last_scan) {
		sensor_msgs::LaserScanConstPtr scan = last_scan;

		sensor_msgs::LaserScan filtered_scan;

		// Keep only scans with intensity above threshold
		laser_scan_filter_chain->update (*scan, filtered_scan);

		/*// Convert scan to point cloud in the odom frame
		if(!tf_listener->waitForTransform(
				scan->header.frame_id,
				"odom",
				scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
				ros::Duration(1.0))){
			return;
		}*/

		// Create point cloud
		sensor_msgs::PointCloud2 cloud_msg;
		laser_projection->transformLaserScanToPointCloud(scan->header.frame_id, filtered_scan, cloud_msg, *tf_listener);

		// Container for original & filtered PCL data
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::fromROSMsg(cloud_msg, *pcl_cloud);

		// Cluster the data to find beacons as centroids
		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
		tree->setInputCloud (pcl_cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
		ec.setClusterTolerance (0.3); // 30cm
		ec.setMinClusterSize (3);
		ec.setMaxClusterSize (100);
		ec.setSearchMethod (tree);
		ec.setInputCloud (pcl_cloud);
		ec.extract(cluster_indices);

		// Create point cloud for beacon centers
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

		// Extract cluster centroids
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointXYZI centroid;
			pcl::computeCentroid(*pcl_cloud, it->indices, centroid);

			cloud_cluster->points.push_back(centroid);
		}

		// Compute the pose if the right number of points is found
		// TODO robustify
		if (cloud_cluster->size() == 3) {
			float distance_1 = pcl::squaredEuclideanDistance(cloud_cluster->points[0], cloud_cluster->points[1]);
			float distance_2 = pcl::squaredEuclideanDistance(cloud_cluster->points[1], cloud_cluster->points[2]);
			float distance_3 = pcl::squaredEuclideanDistance(cloud_cluster->points[2], cloud_cluster->points[0]);

			pcl::PointXYZI *point_1, *point_2, *point_3;

			// The third point is the farthest one, pick point_1 and point_2 randomly
			if (distance_1 < distance_2 && distance_1 < distance_3) {
				point_1 = &cloud_cluster->points[0];
				point_2 = &cloud_cluster->points[1];
				point_3 = &cloud_cluster->points[2];
			} else if (distance_2 < distance_1 && distance_2 < distance_3) {
				point_1 = &cloud_cluster->points[1];
				point_2 = &cloud_cluster->points[2];
				point_3 = &cloud_cluster->points[0];
			} else {
				point_1 = &cloud_cluster->points[0];
				point_2 = &cloud_cluster->points[2];
				point_3 = &cloud_cluster->points[1];
			}

			// Compute the outer product, if it is negative then we must invert point_1 and point_2
			if ((point_3->x - point_1->x) * (point_2->y - point_1->y) - (point_3->y - point_1->y) * (point_2->x - point_1->x) < 0) {
				pcl::PointXYZI *tmp = point_1;
				point_1 = point_2;
				point_2 = tmp;
			}

			point_1->intensity = 100.0;
			point_2->intensity = 50.0;
			point_3->intensity = 0.0;

			// Convert to ROS data type
			sensor_msgs::PointCloud2 output;
			pcl::toROSMsg(*cloud_cluster, output);
			output.header.frame_id = scan->header.frame_id;
			output.header.stamp = scan->header.stamp;

			// Publish the data
			filtered_points_publisher.publish(output);

			// Compute coefficients
			//pcl::PointXYZ p1()
			float r1 = point_1->getVector3fMap().norm();
			float r2 = point_2->getVector3fMap().norm();
			float r3 = point_3->getVector3fMap().norm();
			float d = 1.900;
			float i = d / 2;
			float j = 3.188;

			float x = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
			float y = (r1 * r1 - r3 * r3 + i * i + j * j) / (2 * j) - i * x / j;

			// Compute the point coordinates by trilateration
			// https://en.wikipedia.org/wiki/Trilateration
			geometry_msgs::PoseWithCovarianceStamped point_r;
			point_r.header.frame_id = "odom"; //scan->header.frame_id;
			point_r.header.stamp = scan->header.stamp;
			point_r.pose.pose.position.x = x - i;
			point_r.pose.pose.position.y = j / 2 - y;
			point_r.pose.pose.position.z = 0;
			point_r.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

			estimated_pos_publisher.publish(point_r);
		}
	}
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
	estimated_pos_publisher = nh_priv.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pos", 1, false);

	// Init subscribers
	ros::Subscriber laser_scan_subscriber = nh_priv.subscribe("laser_scan", 1, &laser_scan_callback);

	// Compute according to a rate to earn ressources
	ros::Rate rate(10);

	// Spin
	while(ros::ok()) {
		handle_scan();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
