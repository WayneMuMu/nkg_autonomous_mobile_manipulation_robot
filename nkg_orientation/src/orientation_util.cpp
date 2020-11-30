/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "nkg_orientation/orientation_util.h"
#include "nkg_demo_msgs/ObjectInfo.h"
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/CameraInfo.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <thread>
#include <limits>
#include <cmath>
#include <random>
#include <std_msgs/MultiArrayDimension.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define INFO 1

namespace orientation_util
{
OrientationUtil::OrientationUtil(const ros::NodeHandle& nh): _n(nh){
	// tf_buffer
	_tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
	_tfl = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

	// configure parameters
	configParam();

	// pcl pointcloud
	_xyzrgb_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
}

OrientationUtil::~OrientationUtil(){

}

bool OrientationUtil::start(){
	ros::NodeHandle pn("~");
	// get _ref_robot_link
	if(!pn.getParam("ref_robot_link", _ref_robot_link)){
		ROS_ERROR_STREAM("No ref_robot_link provided in launch file!");
		return false;
	}

	std::string topic;
	// camera parameters
	if(!pn.getParam("caminfo_topic", topic)){
		ROS_ERROR_STREAM("No caminfo_topic in config!");
		return false;
	}
	sensor_msgs::CameraInfoConstPtr cam_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, _n, ros::Duration(2.0));
	if (!cam_msg){
		ROS_ERROR_STREAM("No camera message received, camera params failed!");
		return false;
	}
	_col = cam_msg->width;
	_row = cam_msg->height;
	_fx = cam_msg->K[0];
	_fy = cam_msg->K[4];
	_ppx = cam_msg->K[2];
	_ppy = cam_msg->K[5];

	// topic constructions
	if (!pn.getParam("rgbd_topic", topic)){
		ROS_ERROR_STREAM("No rgbd_topic in config!");
		return false;
	}
	_rgbd_sub.subscribe(_n, topic, 1);
	if (!pn.getParam("bb_topic", topic)){ROS_ERROR_STREAM("No bb_topic in config!"); return false;}
	_bb_sub.subscribe(_n, topic, 1);
	pn.param<std::string>("voxel_output", topic, "voxel_points");
	_voxel_pub = _n.advertise<sensor_msgs::PointCloud2>(topic, 1);
	pn.param<std::string>("cluster_output", topic, "cluster_points");
	_cluster_pub = _n.advertise<sensor_msgs::PointCloud2>(topic, 1);
	pn.param<std::string>("orientation_output", topic, "orientation_info");
	_ori_pub = _n.advertise<nkg_demo_msgs::ObjectInfo>(topic, 1);
	pn.param<std::string>("visual_output", topic, "visual_output");
	_marker_pub = _n.advertise<visualization_msgs::MarkerArray>(topic, 1);

	// cameraCB
	_sync.reset(new Sync(MySyncPolicy(5), _rgbd_sub, _bb_sub));
	_sync->registerCallback(boost::bind(&OrientationUtil::cameraCB, this, _1, _2));
	return true;
}

void OrientationUtil::configParam(){
	// for publish 3D scene object
	_default_color.r = 0.0;
	_default_color.g = 1.0;
	_default_color.b = 0.0;
	_default_color.a = 1.0;

	// ros params
	ros::NodeHandle pn("~");
	pn.param("seg", _seg, true);
	// plane tolerance when extracting horizontal plane (radian)
	pn.param("plane_tol", _plane_tol, M_PI/20);

	// read clusters' colors
	std::string line;
	std::ifstream file (ros::package::getPath("nkg_orientation")+"/config/cluster_colors.txt");
	if (file.is_open()){
		std::vector<uint8_t> rgb;
		while ( std::getline (file, line) ){
			std::stringstream ss(line);			
			if(std::getline(ss, line, ','))
				rgb.emplace_back(std::stoi(line));	// r
			if(std::getline(ss,line,','))
				rgb.emplace_back(std::stoi(line));	// g
			if(std::getline(ss,line,','))
				rgb.emplace_back(std::stoi(line));	// b
			if (rgb.size() != 3)
				continue;
			else{
				_rgbs.emplace_back((uint32_t)rgb[0]<<16 | (uint32_t)rgb[1]<<8 |(uint32_t)rgb[2]);
			}
			rgb.clear();
		}
		file.close();
	}
	else
		ROS_ERROR_STREAM("Unable to read clusters' colors!");	
	ROS_INFO_STREAM("Default "<< _rgbs.size() << " cluster_colors.");

	// read classes names
	file.open(ros::package::getPath("nkg_orientation")+"/config/names.txt");
	if (file.is_open()){
		while ( std::getline (file, line) ){
			std::stringstream ss(line);
			std::getline(ss, line, ',');
			_classes.emplace_back(std::move(line));
			std::vector<float> rgba;
			if(std::getline(ss, line, ','))
				rgba.emplace_back(std::stof(line));	// r
			if(std::getline(ss,line,','))
				rgba.emplace_back(std::stof(line));	// g
			if(std::getline(ss,line,','))
				rgba.emplace_back(std::stof(line));	// b
			if(std::getline(ss,line,','))
				rgba.emplace_back(std::stof(line));	// a
			if (rgba.size() == 3)
				rgba.emplace_back(1.0);	// append a
			if (rgba.size() == 4){
				std_msgs::ColorRGBA color;
				color.r = rgba[0];
				color.g = rgba[1];
				color.b = rgba[2];
				color.a = rgba[3];
				_color_map[_classes.back()] = color;
			}
		}
		file.close();
	}
	else
		ROS_ERROR_STREAM("Unable to read class names!");
	ROS_INFO_STREAM("Total "<< _classes.size() << " classes.");
}

void OrientationUtil::cameraCB(const sensor_msgs::PointCloud2ConstPtr& pcl_msg, const nkg_demo_msgs::MultiBBoxConstPtr& bb_msg){
#if INFO
	ROS_INFO_STREAM("Start CB");
#endif
	// get transform to _ref_robot_link
	tf2::Stamped<tf2::Transform> trans;
	try{
		tf2::fromMsg(_tf_buffer->lookupTransform(_ref_robot_link, pcl_msg->header.frame_id, pcl_msg->header.stamp), trans);
	}
	catch (tf2::TransformException& ex){
		ROS_ERROR_STREAM("Transform error: " << ex.what() << "; quitting callback");
		return;
	}

	// threads for updating bbox
	std::thread t1(&OrientationUtil::updateBBox, this, bb_msg);
#if INFO
	ROS_INFO_STREAM("updateBBox thread start");
#endif

	// transform to pcl cloud
	pcl::PCLPointCloud2Ptr in(new pcl::PCLPointCloud2), out(new pcl::PCLPointCloud2);
	pcl_conversions::toPCL(*pcl_msg, *in);

	// voxel filter
	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	vg.setInputCloud(in);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*out);

	// transform to pcl::PointCloud<pcl::PointXYZRGB>
	pcl::fromPCLPointCloud2(*out, *_xyzrgb_ptr);
#if INFO
	ROS_INFO_STREAM("Finish voxel filtering, points: " << _xyzrgb_ptr->size());
#endif

	// publish voxel
	std::thread t2(&OrientationUtil::pubVoxel, this, *out);

	if (_seg){
		pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coeffs1(new pcl::ModelCoefficients);		
		uint8_t result1 = getPlane(trans, inliers1, coeffs1, _xyzrgb_ptr);
		if(result1 == SEG::NOPLANE)
			ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
		else if (result1 == SEG::HORIZ){
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(_xyzrgb_ptr);
			extract.setIndices(inliers1);
			extract.setNegative(true);
			extract.filter(*_xyzrgb_ptr);
		}
		else if (result1 == SEG::VERT){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr vert(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(_xyzrgb_ptr);
			extract.setIndices(inliers1);
			extract.setNegative(true);
			extract.setKeepOrganized(true);
			extract.filter(*vert);
			pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coeffs2(new pcl::ModelCoefficients);
			uint8_t result2 = getPlane(trans, inliers2, coeffs2, vert);
			if (result2 == SEG::HORIZ){
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				extract.setInputCloud(_xyzrgb_ptr);
				extract.setIndices(inliers2);
				extract.setNegative(true);
				extract.filter(*_xyzrgb_ptr);
#if INFO
				ROS_INFO_STREAM("Both vertical and horizontal plane found. Segment horizontal one");
#endif
			}
			else
				ROS_WARN_STREAM("Only vertical ones found. No plane segmented.");
		}
		else	// result1 == SEG::UNKNOWN
			ROS_WARN_STREAM("The plane's orientation is unknown");

#if INFO
		ROS_INFO_STREAM("Finish removing plane, points: " << _xyzrgb_ptr->size());
#endif
	}
	
	// create the KdTree object for searching for clustering
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(_xyzrgb_ptr);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(20000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(_xyzrgb_ptr);
	ec.extract(cluster_indices);

	if (!cluster_indices.empty()){
		_clusters.resize(cluster_indices.size());
		// construct _clusters
		for (int i=0; i<_clusters.size(); ++i)
			_clusters[i].indices = std::move(cluster_indices[i].indices);
	}
	else{
		ROS_WARN_STREAM("Clustering Fails");
		_clusters.clear();	
	}
#if INFO
	ROS_INFO_STREAM("Finish clustering");
#endif

	// publish cluster
	std::thread t3(&OrientationUtil::pubCluster, this, std::ref(_rgbs));

	// check bbox thread and then generate cuboids
	t1.join();
	generateCuboids(trans);

	// publish 3D
	pubVisual();

	// check pub voxel, cluster
	t2.join();
	t3.join();

#if INFO
	ROS_INFO_STREAM("Finish CB");
#endif
}

uint8_t OrientationUtil::getPlane(const tf2::Stamped<tf2::Transform>& trans, pcl::PointIndices::Ptr& inliers, pcl::ModelCoefficients::Ptr& coeffs, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const{
	// create the segmentation object for the planar model
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coeffs);

	if (inliers->indices.empty())
		return SEG::NOPLANE;
	else{
		tf2::Vector3 horiz(0,0,1), p_origin(0,0,0),
							p_normal(coeffs->values[0],coeffs->values[1],coeffs->values[2]);
		tf2::Vector3 normal = trans * p_normal - trans * p_origin;
		if (std::abs(horiz.angle(normal)) < _plane_tol)
			return SEG::HORIZ;
		else if (std::abs(horiz.dot(normal.normalize())) < sin(_plane_tol))
			return SEG::VERT;
		else
			return SEG::UNKNOWN;
	}
}

void OrientationUtil::pubCluster(std::vector<uint32_t>& colors) const{
	if (_clusters.empty())
		return;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (colors.size() < _clusters.size()){
		std::random_device rd;
		std::default_random_engine gen(rd());
		std::uniform_int_distribution<uint8_t> rgb(0, 255);
		int add = _clusters.size() - colors.size();
		for (int i=0; i<add; ++i){
			colors.emplace_back((uint32_t)rgb(gen)<<16 | (uint32_t)rgb(gen)<<8 |(uint32_t)rgb(gen));
		}
	}

	for (std::vector<Cluster>::const_iterator it=_clusters.cbegin(); it != _clusters.cend(); ++it){
		for (std::vector<int>::const_iterator pit = it->indices.cbegin(); pit != it->indices.cend(); ++pit){
			pcl::PointXYZRGB point = _xyzrgb_ptr->at(*pit);
			point.rgb = *reinterpret_cast<float*>(&colors[it-_clusters.cbegin()]);
			cloud_cluster->push_back(point);
		}
	}
	// in camera_frame
	cloud_cluster->header.frame_id = _xyzrgb_ptr->header.frame_id;
	cloud_cluster->width = cloud_cluster->size();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_cluster, output);
	output.header.stamp = ros::Time::now();
	_cluster_pub.publish(output);
}

void OrientationUtil::pubVoxel(const pcl::PCLPointCloud2& voxel_pcl) const{
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(voxel_pcl, output);
	_voxel_pub.publish(output);
}

void OrientationUtil::pubVisual() const{
	visualization_msgs::MarkerArray vis;
	visualization_msgs::Marker marker;

	// clean
	marker.action = visualization_msgs::Marker::DELETEALL;
	vis.markers.emplace_back(std::move(marker));
	_marker_pub.publish(vis);
	vis.markers.clear();
	
	geometry_msgs::Pose pose;
	tf2::Quaternion quat;

	for (std::vector<Cuboid>::const_iterator it=_cuboids.cbegin(); it != _cuboids.cend(); ++it){
		if (it->classId == -1)
			continue;
		marker.header.frame_id = _ref_robot_link;
		marker.ns="visual";
		marker.id = it-_cuboids.cbegin();
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		pose.position.x = it->center.x;
		pose.position.y = it->center.y;
		pose.position.z = it->center.z;
		quat.setRPY(0, 0, it->yaw * M_PI/180);
		pose.orientation = tf2::toMsg(quat);
		marker.pose = pose;
		marker.scale.x = it->dim[0];
		marker.scale.y = it->dim[1];
		marker.scale.z = it->dim[2];
		if (_color_map.find(_classes[it->classId])!=_color_map.end())
			marker.color = _color_map.at(_classes[it->classId]);
		else
			marker.color = _default_color;
		marker.lifetime = ros::Duration();
		marker.header.stamp = ros::Time::now();
		vis.markers.emplace_back(std::move(marker));
	}
	if (!vis.markers.empty())
		_marker_pub.publish(vis);
}

void OrientationUtil::generateCuboids(const tf2::Stamped<tf2::Transform>& trans){
	_cuboids.clear();

	if (_bboxes.empty() || _clusters.empty())
		return;
#if INFO
	ROS_INFO_STREAM("#_bboxes: " << _bboxes.size() << ", #_clusters: " << _clusters.size());
#endif
	_cuboids.resize(_bboxes.size());

	for (int i=0; i<_clusters.size(); ++i){
		int vote[_bboxes.size()] = {0};
		std::vector<bool> flag(_bboxes.size(), false);
		for (std::vector<int>::const_iterator it=_clusters[i].indices.cbegin(); it!=_clusters[i].indices.cend(); ++it){
			// projection
			int px = static_cast<int>(_xyzrgb_ptr->at(*it).x / _xyzrgb_ptr->at(*it).z * _fx + _ppx),
				py = static_cast<int>(_xyzrgb_ptr->at(*it).y / _xyzrgb_ptr->at(*it).z * _fy + _ppy);
			// find 8-neighbor belongs to bboxes or not
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
					++vote[f-flag.cbegin()];
			}
		}
		// find the bbox matches the most with the cluster
		auto max_ptr = std::max_element(vote, vote+_bboxes.size());
		int idx = max_ptr - vote;
		// accept the matching if # matched points greter than a threshold
		if (*max_ptr > _clusters[i].indices.size() * 0.8){
			_cuboids[idx].cluster_idx.emplace_back(i);
		}
#if INFO
		ROS_INFO_STREAM("_cuboids[" << idx << "] covers _clusters[" << i << "] (size: " << _clusters[i].indices.size() <<") with ratio: " << static_cast<float>(*max_ptr) / _clusters[i].indices.size());
#endif
	}

	nkg_demo_msgs::ObjectInfo obj_list;
	int number = 0;

	// for each cuboid, combine the clusters belong to it
	for (std::vector<Cuboid>::iterator cu=_cuboids.begin(); cu!=_cuboids.end(); ++cu){
		if (cu->cluster_idx.empty())
			continue;
		std::vector<cv::Point2f> pts;
		float z_upper = std::numeric_limits<float>::lowest(),
			  z_lower = std::numeric_limits<float>::max();
		for (std::vector<int>::const_iterator cl=cu->cluster_idx.cbegin(); cl!=cu->cluster_idx.cend(); ++cl){
			tf2::Vector3 p;
			for (std::vector<int>::const_iterator id=_clusters[*cl].indices.cbegin(); id!=_clusters[*cl].indices.cend(); ++id){
				// transform coordinate, w.r.t _ref_robot_link
				p = trans * tf2::Vector3(_xyzrgb_ptr->at(*id).x,_xyzrgb_ptr->at(*id).y,_xyzrgb_ptr->at(*id).z);
				// record projection to XY-plane for later minAreaRect
				pts.emplace_back(p.getX(), p.getY());
				// update height of the object
				if (p.getZ() > z_upper)
					z_upper = p.getZ();
				else if (p.getZ() < z_lower)
					z_lower = p.getZ();
			}
		}

		if (pts.size() < 5){
			ROS_ERROR_STREAM("Cuboids too small!");
			continue;
		}
		/*********************************************************************
		TODO transform and re-transform for inclination

		*********************************************************************/
		cv::RotatedRect top = cv::minAreaRect(pts);
		cu->dim[0] = top.size.width;
		cu->dim[1] = top.size.height;
		cu->dim[2] = z_upper - z_lower;
		cu->yaw = top.angle;
		cu->classId = _bboxes[cu-_cuboids.cbegin()].classId;
		cu->center.x = top.center.x;
		cu->center.y = top.center.y;
		cu->center.z = (z_upper + z_lower)/2;

		// construct obj list for _ori_pub
		obj_list.distance.emplace_back(std::sqrt(cu->center.x * cu->center.x + 
												 cu->center.y * cu->center.y));
		obj_list.theta.emplace_back(atan2(cu->center.y, cu->center.x));
		obj_list.obj_type.emplace_back(cu->classId);
		obj_list.obj_radius.emplace_back(0.5*std::sqrt(cu->dim[0]*cu->dim[0]+cu->dim[1]*cu->dim[1]));
		obj_list.obj_detail.data.emplace_back(cu->center.x);
		obj_list.obj_detail.data.emplace_back(cu->center.y);
		obj_list.obj_detail.data.emplace_back(cu->center.z);
		obj_list.obj_detail.data.emplace_back(cu->dim[0]);
		obj_list.obj_detail.data.emplace_back(cu->dim[1]);
		obj_list.obj_detail.data.emplace_back(cu->dim[2]);
		obj_list.obj_detail.data.emplace_back(cu->yaw);
		++number;
	}

	// publish obj list
	if (number){
		std_msgs::MultiArrayDimension dims;
		dims.label = "number";
		dims.size = number;
		dims.stride = 7;
		obj_list.obj_detail.layout.dim.emplace_back(std::move(dims));
		obj_list.header.stamp = ros::Time::now();
		obj_list.header.frame_id = _ref_robot_link;
		_ori_pub.publish(obj_list);
	}
}

void OrientationUtil::updateBBox(const nkg_demo_msgs::MultiBBoxConstPtr& bb_msg){
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
				_bb_map[std::make_pair(c,r)].emplace_back(i);
			}
		}
	}
}

}	// namespace orientation_util
