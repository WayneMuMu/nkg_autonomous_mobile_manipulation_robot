#ifndef NKG_ORIENTATION_UTIL_H
#define NKG_ORIENTATION_UTIL_H

#include "nkg_demo_msgs/MultiBBox.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <shape_msgs/Mesh.h>

#include <map>

namespace orientation_util
{
enum class SEG_RESULT{
	NOPLANE,
	HORIZ,
	VERT,
	UNKNOWN 
};
/*
enum class VISUAL{
	MESH,
	CUBOID
};
*/
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

class OrientationUtil {
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
															nkg_demo_msgs::MultiBBox> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

public:
	OrientationUtil(const ros::NodeHandle&);
	~OrientationUtil();
	
	bool start();

private:
	void configParam();
	void cameraCB(const sensor_msgs::PointCloud2ConstPtr&, const nkg_demo_msgs::MultiBBoxConstPtr&);
	void updateBBox(const nkg_demo_msgs::MultiBBoxConstPtr&);
	void generateCuboids(const tf2::Stamped<tf2::Transform>&);
	void pubVoxel(const pcl::PCLPointCloud2&) const;
	void pubCluster() const;
	void pubVisualCuboid() const;
	SEG_RESULT getPlane(const tf2::Stamped<tf2::Transform>&, pcl::PointIndices::Ptr&, pcl::ModelCoefficients::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&) const;

	// better visualization
	void pubVisualMesh() const;
	void pclMeshToShapeMsg(const pcl::PolygonMesh&, shape_msgs::Mesh&) const;
	void pclMeshToMarkerMsg(const pcl::PolygonMesh&, visualization_msgs::Marker&) const;

	message_filters::Subscriber<sensor_msgs::PointCloud2> _rgbd_sub;	// point cloud topic
	message_filters::Subscriber<nkg_demo_msgs::MultiBBox> _bb_sub;		// bounding box topic
	boost::shared_ptr<Sync> _sync;										// sync rgbd and bb messages
	std::shared_ptr<tf2_ros::Buffer> _tf_buffer;						// tf transform buffer
	std::shared_ptr<tf2_ros::TransformListener> _tfl;					// tf transform listener
	ros::Publisher _voxel_pub, _cluster_pub, _ori_pub, _marker_pub;		// output publishers
	ros::NodeHandle _n;
	std::vector<Cluster> _clusters;								// proccessed clusters
	std::vector<Cuboid> _cuboids;								// generated cuboids
	std::vector<BBox> _bboxes;									// bounding boxes info
	std::map<std::pair<int,int>, std::vector<int> > _bb_map;	// classes that image(c,r) belongs to
	std::map<std::string, std_msgs::ColorRGBA> _color_map;		// color for COCO dataset
	int _col, _row;
	double _fx, _fy, _ppx, _ppy, _plane_tol;					// camera intrinsics, plane tolerance
	bool _seg;													// segment plane after voxel or not
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _xyzrgb_ptr;			// point cloud
	std::vector<std::string> _classes;							// names in COCO dataset
	std::string _ref_robot_link;								// transform camera to robot frame
	std_msgs::ColorRGBA _default_color;							// yolov3 object default color: green
	mutable std::vector<std::array<unsigned int, 3> > _rgbs;	// for drawing clusters
};
}
#endif
