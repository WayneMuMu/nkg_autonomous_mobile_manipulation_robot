#ifndef NKG_RECOGNITION_UTIL_H
#define NKG_RECOGNITION_UTIL_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include "nkg_demo_msgs/Table.h"
#include "nkg_demo_msgs/MultiBBox.h"
#include "moveit_msgs/ObjectColor.h"
#include <map>

#define DEBUG 1

namespace recognition_util
{
struct BBox {	// unit: pixel
	int cs  = 0, ce = 0, rs = 0, re = 0;
	int classId = -1;
	float prob = -1.0;
};

struct Cluster {
	std::vector<int> indices;
};

struct Cuboid {	// unit: m
	pcl::PointXYZ center;
	int classId = -1;
	float ori = 0.0;
	float dim[3];
	std::vector<int> cluster_idx;
};

struct Table {	// unit: m
	Table(): width(0.0), length(0.0), depth(0.0){};
	Table(const std::vector<float>&);
	float iou(const Table&) const;
	void reset(){ width = length = depth = 0.0;};

	pcl::PointXYZ center;
	float width, length, depth, angle;
	float coeffs[4];
};

class RecognitionUtil {
public:
	RecognitionUtil(const ros::NodeHandle&);
	~RecognitionUtil();
	
	void start();
	bool getTable(nkg_demo_msgs::Table::Request&, nkg_demo_msgs::Table::Response&);	// ros service

private:
	void configParam();													// configure parameters
	void cameraCB(const sensor_msgs::PointCloud2ConstPtr&, const nkg_demo_msgs::MultiBBoxConstPtr&);
	void updateTable(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&, 
										const tf2::Stamped<tf2::Transform>&);
	void updateBBox(const nkg_demo_msgs::MultiBBoxConstPtr&);
	void generateCuboids(const tf2::Stamped<tf2::Transform>&);
	void pubToPlanNode() const;
	void pubToOctomap() const;
#if DEBUG
	void debugThread() const;
#endif

	message_filters::Subscriber<sensor_msgs::PointCloud2> _rgbd_sub;
	message_filters::Subscriber<nkg_demo_msgs::MultiBBox> _bb_sub;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nkg_demo_msgs::MultiBBox> MySyncPolicy;
	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> _sync;
//	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nkg_demo_msgs::MultiBBox> _sync;
	std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> _tfl;
	ros::ServiceServer _table_srv;
	ros::NodeHandle _n;
	ros::Publisher _obj_pub, _pcl_pub;
	std::vector<Cluster> _clusters;
	std::vector<Cuboid> _cuboids;
	std::vector<BBox> _bboxes;
	std::map<std::pair<int,int>, std::vector<int> > _bb_map;
	std::map<std::string, moveit_msgs::ObjectColor> _color_map;
	int _col, _row;
	double _fx, _fy, _ppx, _ppy;										// camera parameters
	pcl::PointCloud<pcl::PointXYZ>::Ptr _xyz_ptr;
	Table _table;
	std::vector<std::string> _classes;									// names in COCO dataset
#if DEBUG
	ros::Publisher _obj_pcl;
	ros::Publisher _marker_pub;
	visualization_msgs::Marker _draw_pts;
#endif
};
}
#endif
