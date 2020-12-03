#ifndef NKG_RECOGNITION_UTIL_H
#define NKG_RECOGNITION_UTIL_H

#include "nkg_demo_msgs/Table.h"
#include "nkg_demo_msgs/MultiBBox.h"
#include "moveit_msgs/PlanningScene.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>

namespace recognition_util
{
enum SEG: uint8_t{
	NOPLANE = 0,
	HORIZ = 1,
	VERT = 2,
	UNKNOWN = 3 
};

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
	float yaw = 0.0;
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
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
															nkg_demo_msgs::MultiBBox> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

public:
	RecognitionUtil(const ros::NodeHandle&);
	~RecognitionUtil();
	
	bool start();
	bool getTable(nkg_demo_msgs::Table::Request&, nkg_demo_msgs::Table::Response&);	// ros service

private:
	void configParam();													// configure parameters
	void cameraCB(const sensor_msgs::PointCloud2ConstPtr&, const nkg_demo_msgs::MultiBBoxConstPtr&);
	void updateTable(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, 
										const tf2::Stamped<tf2::Transform>&);
	void updateBBox(const nkg_demo_msgs::MultiBBoxConstPtr&);
	void generateCuboids(const tf2::Stamped<tf2::Transform>&);
	void pubVoxel(const pcl::PCLPointCloud2&) const;
	void pubCluster(std::vector<uint32_t>&) const;
	void pubVisualCuboid() const;
	uint8_t getPlane(const tf2::Stamped<tf2::Transform>&, pcl::PointIndices::Ptr&, pcl::ModelCoefficients::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&) const;

	// better visualization
	void pubVisualMesh(const tf2::Stamped<tf2::Transform>&) const;
	void pclMeshToShapeMsg(const pcl::PolygonMesh&, shape_msgs::Mesh&) const;
	void pclMeshToMarkerMsg(const pcl::PolygonMesh&, visualization_msgs::Marker&) const;

	message_filters::Subscriber<sensor_msgs::PointCloud2> _rgbd_sub;	// point cloud topic
	message_filters::Subscriber<nkg_demo_msgs::MultiBBox> _bb_sub;		// bounding box topic
	boost::shared_ptr<Sync> _sync;										// sync rgbd and bb messages
	std::shared_ptr<tf2_ros::Buffer> _tf_buffer;						// tf transform buffer
	std::shared_ptr<tf2_ros::TransformListener> _tfl;					// tf transform listener
	ros::Publisher _voxel_pub, _cluster_pub, _obj_pub, _marker_pub;		// output publishers
	ros::ServiceServer _table_srv;										// table service
	ros::NodeHandle _n;
	std::vector<Cluster> _clusters;								// proccessed clusters
	std::vector<Cuboid> _cuboids;								// generated cuboids
	std::vector<BBox> _bboxes;									// bounding boxes info
	std::map<std::pair<int,int>, std::vector<int> > _bb_map;	// classes that image(c,r) belongs to
	std::map<std::string, std_msgs::ColorRGBA> _color_map;		// color for COCO dataset
	int _col, _row;												// image size
	double _fx, _fy, _ppx, _ppy, _plane_tol;					// camera intrinsics, plane tolerance
	bool _seg;													// segment plane after voxel or not
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _xyzrgb_ptr;			// pointcloud
	Table _table;												// table
	std::vector<std::string> _classes;							// names in COCO dataset
	std::string _ref_robot_link;								// transform camera to robot frame
	std_msgs::ColorRGBA _default_color;							// yolov3 object default color: green
	std::vector<uint32_t> _rgbs;								// for drawing clusters

};
}
#endif
