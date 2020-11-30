/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "nkg_recognition/recognition_util.h"

#include "moveit_msgs/PlanningScene.h"
#include "geometry_msgs/Pose.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/mesh_operations.h"
#include "moveit_msgs/PlanningScene.h"
#include "moveit_msgs/CollisionObject.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <thread>
#include <cmath>

#define MAP_FRAME "dobot_m1_base_link"
#define INFO 0

namespace recognition_util
{
Table::Table(const std::vector<float>& vec){
	center.x = vec[0];
	center.y = vec[1];
	center.z = vec[2];
	width = vec[3];
	length = vec[4];
	depth = vec[5];
	angle = vec[6];
};

float Table::iou(const Table& t) const{
	float area_new = width * length, area_old = t.width * t.length;
	if (area_new == 0)
		return 1.0;
	else if (area_old == 0 || std::abs(center.z - t.center.z) > (depth + t.depth)/2)
		return 0.0;
	else{
		cv::RotatedRect t_new(cv::Point2f(center.x,center.y), cv::Size2f(width,length), angle);
		cv::RotatedRect t_old(cv::Point2f(t.center.x,t.center.y), cv::Size2f(t.width,t.length), t.angle);
		std::vector<cv::Point2f> vertices;
		int intersectionType = cv::rotatedRectangleIntersection(t_new, t_old, vertices);
		if (vertices.empty())
			return 0.0;
		else{
			std::vector<cv::Point2f> order_pts;
			cv::convexHull(cv::Mat(vertices), order_pts);
			float area = cv::contourArea(order_pts);
			return area / (area_new + area_old - area);
		}
	}
};

RecognitionUtil::RecognitionUtil(const ros::NodeHandle& nh): _n(nh){
	// tf_buffer
	_tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
	_tfl = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

	// configure parameters
	configParam();

	// pcl pointcloud
	_xyz_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

	// camera parameters (from relasense2 topic: camera_info)
	_fx = 615.0352783203125;
	_fy = 615.0865478515625;
	_ppx = 321.55682373046875;
	_ppy = 237.33897399902344;		

#if DEBUG
	_draw_pts.ns = "table";
	_draw_pts.action = visualization_msgs::Marker::ADD;
	_draw_pts.type = visualization_msgs::Marker::POINTS;
	_draw_pts.id = 0;
	_draw_pts.scale.x = 0.1;
	_draw_pts.scale.y = 0.1;
	_draw_pts.color.g = 1.0;
	_draw_pts.color.a = 1.0;
#endif
}

RecognitionUtil::~RecognitionUtil(){

}

void RecognitionUtil::start(){
	ros::NodeHandle n, private_n("~");
	std::string topic;
	if (!private_n.getParam("output_topic", topic)){ROS_ERROR("No output_topic!"); return;};
	_pcl_pub = n.advertise<sensor_msgs::PointCloud2>(topic,1);
	if (!private_n.getParam("rgbd_topic", topic)){ROS_ERROR("No rgbd_topic!"); return;};
	_rgbd_sub.subscribe(_n, topic, 1);
	if (!private_n.getParam("bb_topic", topic)){ROS_ERROR("No bb_topic!"); return;};
	_bb_sub.subscribe(_n, topic, 1);
	// cameraCB
	_sync.reset(new Sync(MySyncPolicy(5), _rgbd_sub, _bb_sub));
	_sync->registerCallback(boost::bind(&RecognitionUtil::cameraCB, this, _1, _2));

	_obj_pub = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	_table_srv = n.advertiseService("get_table", &RecognitionUtil::getTable, this);

#if DEBUG
	_obj_pcl = n.advertise<sensor_msgs::PointCloud2>("cluster_pcl",1);
	_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 5);
#endif
}

void RecognitionUtil::configParam(){
	ros::NodeHandle n;
	if(!n.getParam("camera/realsense2_camera/color_width", _col)){ROS_ERROR("No width"); return;};
	if(!n.getParam("camera/realsense2_camera/color_height", _row)){ROS_ERROR("No height"); return;};

	std::string line;
	std::ifstream file (ros::package::getPath("nkg_recognition")+"/config/names.txt");
	if (file.is_open()){
		while ( getline (file,line) ){
			std::stringstream ss(line);
			getline(ss, line, ',');
			_classes.push_back(line);
			std::vector<float> rgba;
			if(getline(ss, line, ','))
				rgba.push_back(std::stof(line));	// r
			if(getline(ss,line,','))
				rgba.push_back(std::stof(line));	// g
			if(getline(ss,line,','))
				rgba.push_back(std::stof(line));	// b
			if(getline(ss,line,','))
				rgba.push_back(std::stof(line));	// a
			if (rgba.size() == 3)
				rgba.push_back(1.0);	// a
			if (rgba.size() == 4){
				moveit_msgs::ObjectColor oc;
				oc.color.r = rgba[0];
				oc.color.g = rgba[1];
				oc.color.b = rgba[2];
				oc.color.a = rgba[3];
				_color_map[_classes.back()] = oc;	// oc.id not yet filled
			}
		}
		file.close();
	}
	else
		ROS_ERROR("Unable to read Names!");
	ROS_INFO("Total %lu Classes.", _classes.size());
}

void RecognitionUtil::cameraCB(const sensor_msgs::PointCloud2ConstPtr& pcl_msg, const nkg_demo_msgs::MultiBBoxConstPtr& bb_msg){
#if INFO
	ROS_ERROR("Start CB");
#endif
	// get transform to map_frame
	tf2::Stamped<tf2::Transform> trans;
	try{
		tf2::fromMsg(_tf_buffer->lookupTransform(MAP_FRAME, pcl_msg->header.frame_id, pcl_msg->header.stamp), trans);
	}
	catch (tf2::TransformException& ex){
		ROS_ERROR_STREAM("Transform error: " << ex.what() << "; quitting callback");
		return;
	}

	// threads for updating bbox, table
	std::thread t1(&RecognitionUtil::updateBBox, this, bb_msg);
#if INFO
	ROS_ERROR("updateBBox thread start");
#endif

	// transform to pcl cloud
	pcl::PCLPointCloud2Ptr in(new pcl::PCLPointCloud2), out(new pcl::PCLPointCloud2);
	pcl_conversions::toPCL(*pcl_msg, *in);

	// voxel filter
	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	vg.setInputCloud(in);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*out);

	// transform to pcl::PointCloud<pcl::PointXYZ>
	pcl::fromPCLPointCloud2(*out, *_xyz_ptr);
#if INFO
	ROS_ERROR("Finish voxel filtering, points: %lu", _xyz_ptr->size());
#endif

	// thread to publish to pointcloud_octomap_updater
	std::thread t2(&RecognitionUtil::pubToOctomap, this), t3;	//	t3 for later use
#if INFO
	ROS_ERROR("pubToOctomap thread start");
#endif

	// create the segmentation object for the planar model
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud (_xyz_ptr);
	seg.segment (*inliers, *coefficients);

	// in case that _xyz_ptr is changed in the following
	t2.join();
#if INFO
	ROS_ERROR("pubToOctomap thread finish");
#endif

	bool found_t = true;
	if (inliers->indices.empty()){
		ROS_WARN("Could not estimate a planar model for the given dataset.");
		found_t = false;
		_table.reset();
	}
	else{
		pcl::PointCloud<pcl::PointXYZ>::Ptr neg_t(new pcl::PointCloud<pcl::PointXYZ>),
											table(new pcl::PointCloud<pcl::PointXYZ>);
	    // set the planar inliers to NAN
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(_xyz_ptr);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*table);
		extract.setNegative(true);
		extract.filter(*neg_t);
		_xyz_ptr->swap(*neg_t);

		// update _table
		_table.coeffs[0] = coefficients->values[0];
		_table.coeffs[1] = coefficients->values[1];
		_table.coeffs[2] = coefficients->values[2];
		_table.coeffs[3] = coefficients->values[3];
		t3 = std::thread(&RecognitionUtil::updateTable, this, table, trans);
#if INFO
		ROS_ERROR("updateTable thread start");
#endif
	}
#if INFO
	ROS_ERROR("Finish removing table, points: %lu", _xyz_ptr->size());
#endif
	
	// create the KdTree object for searching for clustering
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(_xyz_ptr);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.01);
	ec.setMinClusterSize(20);		// after removing plane, pts left about 2XXX - 4XXX
	ec.setMaxClusterSize(2000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(_xyz_ptr);
	ec.extract(cluster_indices);

	_clusters.clear();
	if (!cluster_indices.empty()){
		_clusters.resize(cluster_indices.size());
		// construct _clusters
		for (int i=0; i<_clusters.size(); ++i)
			_clusters[i].indices = std::move(cluster_indices[i].indices);
	}
#if INFO
	ROS_ERROR("Finish clustering");
#endif

#if DEBUG
	std::thread t4(&RecognitionUtil::debugThread, this);
#endif

	// check bbox thread to generate cuboids
	t1.join();
#if INFO
	ROS_ERROR("updateBBox thread finish");
#endif

	// generate cuboids
	generateCuboids(trans);

	// check table thread to publish to moveit
	if (found_t){
		t3.join();
#if INFO
		ROS_ERROR("updateTable thread finish");
#endif
	}

	// publish to nkg_move_plan node
	pubToPlanNode();

#if DEBUG
	t4.join();
#endif

#if INFO
	ROS_ERROR("Finish CB");
#endif
}

#if DEBUG
void RecognitionUtil::debugThread() const{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

	for (std::vector<Cluster>::const_iterator it=_clusters.cbegin(); it != _clusters.cend(); ++it){
		for (std::vector<int>::const_iterator pit = it->indices.cbegin(); pit != it->indices.cend(); ++pit)
			cloud_cluster->push_back(_xyz_ptr->at(*pit));
	}
	// in camera_frame
	cloud_cluster->header.frame_id = _xyz_ptr->header.frame_id;
	cloud_cluster->width = cloud_cluster->size();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;
	sensor_msgs::PointCloud2 obj_output;
	pcl::toROSMsg(*cloud_cluster, obj_output);
	_obj_pcl.publish(obj_output);
}
#endif

void RecognitionUtil::pubToOctomap() const{
	// in camera_frame
	sensor_msgs::PointCloud2 pcl_output;
	pcl::toROSMsg(*_xyz_ptr, pcl_output);
	_pcl_pub.publish(pcl_output);
}

void RecognitionUtil::pubToPlanNode() const{
	static std::vector<moveit_msgs::CollisionObject> last_obj_list;
	std::vector<moveit_msgs::CollisionObject> obj_list;
	moveit_msgs::CollisionObject obj;
	obj.header.frame_id = MAP_FRAME;
	geometry_msgs::Pose pose;
	tf2::Quaternion quat;

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	obj.primitives.resize(1);
	obj.primitive_poses.resize(1);
	obj.operation = obj.ADD;

	// publish
	moveit_msgs::PlanningScene ps;
	moveit_msgs::ObjectColor oc;
	ps.is_diff = true;

	// remove old objects
	if (!last_obj_list.empty()){
		for (int i=0; i<last_obj_list.size(); ++i)
			last_obj_list[i].operation = last_obj_list[i].REMOVE;
		ps.world.collision_objects = std::move(last_obj_list);
		_obj_pub.publish(ps);
	}

	// add new objects
	for (int i=0; i<_cuboids.size(); ++i){
		if (_cuboids[i].classId != -1){
			pose.position.x = _cuboids[i].center.x;
			pose.position.y = _cuboids[i].center.y;
			pose.position.z = _cuboids[i].center.z;
			quat.setRPY(0, 0, _cuboids[i].ori * M_PI/180);
			pose.orientation = tf2::toMsg(quat);
			primitive.dimensions.assign(std::begin(_cuboids[i].dim), std::end(_cuboids[i].dim));
			obj.id = _classes[_cuboids[i].classId] +"_cuboids_"+std::to_string(i);
			if (_color_map.find(_classes[_cuboids[i].classId])!=_color_map.end())
				oc = _color_map.at(_classes[_cuboids[i].classId]);
			else{
				oc.color.r = 1.0;
				oc.color.g = 1.0;
				oc.color.b = 0.0;
				oc.color.a = 1.0;
			}
			oc.id = obj.id;
			ps.object_colors.push_back(oc);
			obj.primitives[0] = primitive;
			obj.primitive_poses[0] = pose;
			obj_list.push_back(obj);
		}
	}

	// add table, TODO integrate in ps plane
	if (_table.width != 0 && _table.length != 0){
		pose.position.x = _table.center.x;
		pose.position.y = _table.center.y;
		pose.position.z = _table.center.z;
		quat.setRPY(0, 0, _table.angle * M_PI/180);
		pose.orientation = tf2::toMsg(quat);
		std::vector<double> temp{_table.width, _table.length, _table.depth};
		primitive.dimensions = std::move(temp);
		obj.id = "table";
		oc.id = obj.id;
		oc.color.r = 1.0;
		oc.color.g = 1.0;
		oc.color.b = 1.0;
		oc.color.a = 1.0;
		ps.object_colors.push_back(oc);
		obj.primitives[0] = primitive;
		obj.primitive_poses[0] = pose;
		obj_list.push_back(obj);
	}

	if (!obj_list.empty()){
		ps.world.collision_objects.assign(obj_list.begin(), obj_list.end());
		last_obj_list = std::move(obj_list);
		_obj_pub.publish(ps);
	}
}

void RecognitionUtil::generateCuboids(const tf2::Stamped<tf2::Transform>& trans){
	_cuboids.clear();
	if (_bboxes.empty() || _clusters.empty())
		return;
#if INFO
	ROS_ERROR("#_bboxes: %lu, #_clusters: %lu", _bboxes.size(), _clusters.size());
#endif
	_cuboids.resize(_bboxes.size());

	for (int i=0; i<_clusters.size(); ++i){
		int vote[_bboxes.size()] = {0};
		std::vector<bool> flag(_bboxes.size(), false);
		for (std::vector<int>::const_iterator it=_clusters[i].indices.cbegin(); it!=_clusters[i].indices.cend(); ++it){
			// projection
			int px = static_cast<int>(_xyz_ptr->at(*it).x / _xyz_ptr->at(*it).z * _fx + _ppx),
				py = static_cast<int>(_xyz_ptr->at(*it).y / _xyz_ptr->at(*it).z * _fy + _ppy);
			for (int r=py-1; r<=py+1; ++r){
				for (int c=px-1; c<=px+1; ++c){
					std::pair<int,int> idx = std::make_pair(c,r);
					if (_bb_map.find(idx) != _bb_map.end()){
						// should have found corresponding bboxes
						for (std::vector<int>::const_iterator bb=_bb_map[idx].cbegin(); bb!=_bb_map[idx].cend(); ++bb){
							if (!flag[*bb])
								flag[*bb] = true;
						}
					}
				}
			}
			for (std::vector<bool>::const_iterator f=flag.cbegin(); f!=flag.cend(); ++f){
				if (*f)
					++vote[f-flag.begin()];
			}
		}
		auto max_ptr = std::max_element(vote, vote+_bboxes.size());
		int idx = max_ptr - vote;
		if (*max_ptr > _clusters[i].indices.size() * 0.8){
			_cuboids[idx].cluster_idx.push_back(i);
#if INFO
			ROS_ERROR("_cuboids[%d] has _clusters[%d], ratio: %f", idx, i, static_cast<float>(*max_ptr) / _clusters[i].indices.size());
#endif
		}
	}

	for (std::vector<Cuboid>::iterator cu=_cuboids.begin(); cu!=_cuboids.end(); ++cu){
		if (cu->cluster_idx.empty())
			continue;
		std::vector<cv::Point2f> pts;
		float z_upper = -2.0, z_lower = 2.0;
		for (std::vector<int>::const_iterator cl=cu->cluster_idx.cbegin(); cl!=cu->cluster_idx.cend(); ++cl){
			tf2::Vector3 p;
			for (std::vector<int>::const_iterator id=_clusters[*cl].indices.cbegin(); id!=_clusters[*cl].indices.cend(); ++id){
				p = trans * tf2::Vector3(_xyz_ptr->at(*id).x,_xyz_ptr->at(*id).y,_xyz_ptr->at(*id).z);
				pts.push_back(cv::Point2f(p.getX(), p.getY()));
				if (p.getZ() > z_upper)
					z_upper = p.getZ();
				else if (p.getZ() < z_lower)
					z_lower = p.getZ();
			}
		}

		if (pts.size() < 5){
			ROS_ERROR("Cuboids too small!");
			continue;
		}
		/*********************************************************************
		TODO transform and re-transform for inclination

		*********************************************************************/
		cv::RotatedRect top = cv::minAreaRect(pts);
		cu->dim[0] = top.size.width;
		cu->dim[1] = top.size.height;
		cu->dim[2] = z_upper - z_lower;
		cu->ori = top.angle;
		cu->classId = _bboxes[cu-_cuboids.begin()].classId;
		cu->center.x = top.center.x;
		cu->center.y = top.center.y;
		cu->center.z = (z_upper + z_lower)/2;
	}
}

void RecognitionUtil::updateBBox(const nkg_demo_msgs::MultiBBoxConstPtr& bb_msg){
	_bboxes.clear();
	_bb_map.clear();

	if (bb_msg->bbox.layout.dim[0].size == 0)
		return;
	_bboxes.resize(bb_msg->bbox.layout.dim[0].size);
	for (int i=0; i < _bboxes.size(); ++i){
		_bboxes[i].classId  = static_cast<int>(bb_msg->bbox.data[6*i]);
		_bboxes[i].prob 	= bb_msg->bbox.data[6*i+5];
		_bboxes[i].cs = std::max(0, static_cast<int>(bb_msg->bbox.data[6*i+1]));
		_bboxes[i].rs = std::max(0, static_cast<int>(bb_msg->bbox.data[6*i+2]));
		_bboxes[i].ce = std::min(_col, _bboxes[i].cs + static_cast<int>(bb_msg->bbox.data[6*i+3]));
		_bboxes[i].re = std::min(_row, _bboxes[i].rs + static_cast<int>(bb_msg->bbox.data[6*i+4]));
		for (int r=_bboxes[i].rs; r<=_bboxes[i].re; ++r){
			for (int c=_bboxes[i].cs; c<=_bboxes[i].ce; ++c){
				_bb_map[std::make_pair(c,r)].push_back(i);
			}
		}
	}
}

void RecognitionUtil::updateTable(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const tf2::Stamped<tf2::Transform>& trans){
	std::vector<cv::Point2f> pts;
	pts.resize(cloud->size());
	cv::Point2f temp;

	/*********************************************************************
	TODO transform and re-transform for inclination

	*********************************************************************/
	float z_upper = -2.0, z_lower = 2.0;
	for (int i=0; i<pts.size(); ++i){
		tf2::Vector3 xyz = trans * tf2::Vector3(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
		temp.x = xyz.getX();
		temp.y = xyz.getY();
		if (xyz.getZ() > z_upper)
			z_upper = xyz.getZ();
		else if (xyz.getZ() < z_lower)
			z_lower = xyz.getZ();
		pts[i] = temp;
	}

	cv::RotatedRect table = cv::minAreaRect(pts);
	_table.center.x = table.center.x;
	_table.center.y = table.center.y;
	_table.width = table.size.width;
	_table.length = table.size.height;
	_table.center.z = (z_upper + z_lower)/2;
	_table.depth = z_upper - z_lower;
	_table.angle = table.angle;

#if DEBUG
	_draw_pts.header.frame_id = MAP_FRAME;
	geometry_msgs::Point pt;
	_draw_pts.points.clear();
	pt.z = _table.center.z;
	cv::Point2f vertices[4];
	table.points(vertices);
	for (int i=0; i<4; ++i){
		pt.x = vertices[i].x;
		pt.y = vertices[i].y;
		_draw_pts.points.push_back(pt);
	}
	_marker_pub.publish(_draw_pts);
#endif
}

bool RecognitionUtil::getTable(nkg_demo_msgs::Table::Request &req,nkg_demo_msgs::Table::Response &res){
	res.table.resize(7);
	res.table[0] = _table.center.x;
	res.table[1] = _table.center.y;
	res.table[2] = _table.center.z;
	res.table[3] = _table.width;
	res.table[4] = _table.length;
	res.table[5] = _table.depth;
	res.table[6] = _table.angle;
	ROS_ERROR("%f,%f,%f,%f,%f,%f,%f", _table.center.x, _table.center.y, _table.center.z, _table.width, _table.length, _table.depth, _table.angle);

	// according to openCV, angle is within (-90,0] (degree)
	float angle_thres = 90.0, iou_thres = 0.3;	// TODO threshold tuning
	if (_table.angle > -angle_thres || _table.angle+90 < angle_thres){	// consider aligned
		res.ready = true;
		Table cleaned_table(req.cleaned_table);
		res.update = (_table.iou(cleaned_table) < iou_thres);
	}
	else{
		res.ready = false;
		res.update = false;
	}
	return true;
}
}	// namespace recognition_util
