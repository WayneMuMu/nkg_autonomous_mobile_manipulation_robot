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

#include <ros/package.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <iostream>
#include <thread>
#include <cmath>

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
	_xyzrgb_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
}

RecognitionUtil::~RecognitionUtil(){

}

bool RecognitionUtil::start(){
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
	pn.param<std::string>("visual_output", topic, "visual_output");
	_marker_pub = _n.advertise<visualization_msgs::MarkerArray>(topic, 1);

	_obj_pub = _n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
	_table_srv = _n.advertiseService("get_table", &RecognitionUtil::getTable, this);

	// cameraCB
	_sync.reset(new Sync(MySyncPolicy(5), _rgbd_sub, _bb_sub));
	_sync->registerCallback(boost::bind(&RecognitionUtil::cameraCB, this, _1, _2));
	return true;
}

void RecognitionUtil::configParam(){
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
	std::ifstream file (ros::package::getPath("nkg_recognition")+"/config/cluster_colors.txt");
	if (file.is_open()){
		std::vector<uint8_t> rgb;
		while ( std::getline (file, line) ){
			std::stringstream ss(line);			
			if(std::getline(ss, line, ','))
				rgb.emplace_back(static_cast<uint8_t>(std::stof(line)*255));	// r
			if(std::getline(ss,line,','))
				rgb.emplace_back(static_cast<uint8_t>(std::stof(line)*255));	// g
			if(std::getline(ss,line,','))
				rgb.emplace_back(static_cast<uint8_t>(std::stof(line)*255));	// b
			if (rgb.size() != 3)
				continue;
			else
				_rgbs.emplace_back((uint32_t)rgb[0]<<16 | (uint32_t)rgb[1]<<8 |(uint32_t)rgb[2]);
			rgb.clear();
		}
		file.close();
	}
	else
		ROS_ERROR_STREAM("Unable to read clusters' colors!");	
	ROS_INFO_STREAM("Default "<< _rgbs.size() << " cluster_colors.");

	// read classes names
	file.open(ros::package::getPath("nkg_recognition")+"/config/names.txt");
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

void RecognitionUtil::cameraCB(const sensor_msgs::PointCloud2ConstPtr& pcl_msg, const nkg_demo_msgs::MultiBBoxConstPtr& bb_msg){
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
	std::thread tb(&RecognitionUtil::updateBBox, this, std::cref(bb_msg));
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
	std::thread tv(&RecognitionUtil::pubVoxel, this, *out), tt;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr table(new pcl::PointCloud<pcl::PointXYZRGB>);
	bool found_t = false;

	if (_seg){
		pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coeffs1(new pcl::ModelCoefficients);		
		uint8_t result1 = getPlane(trans, inliers1, coeffs1, _xyzrgb_ptr);
		if(result1 == SEG::NOPLANE)
			ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
		else if (result1 == SEG::HORIZ){
			found_t = true;
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(_xyzrgb_ptr);
			extract.setIndices(inliers1);
			extract.setNegative(false);
			extract.filter(*table);
			extract.setNegative(true);
			extract.filter(*_xyzrgb_ptr);
			_table.coeffs[0] = coeffs1->values[0];
			_table.coeffs[1] = coeffs1->values[1];
			_table.coeffs[2] = coeffs1->values[2];
			_table.coeffs[3] = coeffs1->values[3];
			tt = std::thread(&RecognitionUtil::updateTable, this, std::cref(table), std::cref(trans));
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
#if INFO
				ROS_INFO_STREAM("Both vertical and horizontal plane found. Segment horizontal one");
#endif
				found_t = true;
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				extract.setInputCloud(_xyzrgb_ptr);
				extract.setIndices(inliers2);
				extract.setNegative(false);
				extract.filter(*table);
				extract.setNegative(true);
				extract.filter(*_xyzrgb_ptr);
				_table.coeffs[0] = coeffs2->values[0];
				_table.coeffs[1] = coeffs2->values[1];
				_table.coeffs[2] = coeffs2->values[2];
				_table.coeffs[3] = coeffs2->values[3];
				tt =std::thread(&RecognitionUtil::updateTable,this,std::cref(table),std::cref(trans));
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
	std::thread tc(&RecognitionUtil::pubCluster, this, std::ref(_rgbs));

	// check bbox thread and then generate cuboids
	tb.join();
	generateCuboids(trans);

	// check table thread
	if (found_t)
		tt.join();

	// publish 3D
//	pubVisualCuboid();
	pubVisualMesh(trans);

	// check pub voxel, cluster
	tv.join();
	tc.join();

#if INFO
	ROS_INFO_STREAM("Finish CB");
#endif
}

uint8_t RecognitionUtil::getPlane(const tf2::Stamped<tf2::Transform>& trans, pcl::PointIndices::Ptr& inliers, pcl::ModelCoefficients::Ptr& coeffs, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const{
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
		double angle = horiz.angle(normal);
		double err = std::abs(angle-M_PI/2);
		if ( err > M_PI/2 - _plane_tol)
			return SEG::HORIZ;
		else if (err < _plane_tol)
			return SEG::VERT;
		else
			return SEG::UNKNOWN;
	}
}

void RecognitionUtil::pubCluster(std::vector<uint32_t>& colors) const{
	if (_clusters.empty())
		return;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (colors.size() < _clusters.size()){
		std::random_device rd;
		std::default_random_engine gen(rd());
		std::uniform_int_distribution<uint8_t> rgb(0, 255);
		int add = _clusters.size() - colors.size();
		for (int i=0; i<add; ++i)
			colors.emplace_back((uint32_t)rgb(gen)<<16 | (uint32_t)rgb(gen)<<8 |(uint32_t)rgb(gen));
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

void RecognitionUtil::pubVoxel(const pcl::PCLPointCloud2& voxel_pcl) const{
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(voxel_pcl, output);
	_voxel_pub.publish(output);
}

void RecognitionUtil::pubVisualCuboid() const{
	static std::vector<moveit_msgs::CollisionObject> last_obj_list;

	// publish
	moveit_msgs::PlanningScene ps;
	ps.is_diff = true;

	// remove old objects
	if (!last_obj_list.empty()){
		for (int i=0; i<last_obj_list.size(); ++i)
			last_obj_list[i].operation = last_obj_list[i].REMOVE;
		ps.world.collision_objects = std::move(last_obj_list);
		last_obj_list.clear();
		_obj_pub.publish(ps);
	}

	if (_cuboids.empty())
		return;

	std::vector<moveit_msgs::CollisionObject> obj_list;
	moveit_msgs::CollisionObject obj;
	obj.header.frame_id = _ref_robot_link;
	geometry_msgs::Pose pose;
	tf2::Quaternion quat;
	moveit_msgs::ObjectColor oc;
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	obj.primitives.resize(1);
	obj.primitive_poses.resize(1);
	obj.operation = obj.ADD;

	// add new objects
	for (std::vector<Cuboid>::const_iterator cu=_cuboids.cbegin(); cu != _cuboids.cend(); ++cu){
		if (cu->classId == -1)
			continue;
		pose.position.x = cu->center.x;
		pose.position.y = cu->center.y;
		pose.position.z = cu->center.z;
		quat.setRPY(0, 0, cu->yaw * M_PI/180);
		pose.orientation = tf2::toMsg(quat);
		primitive.dimensions.assign(std::begin(cu->dim), std::end(cu->dim));
		obj.id = _classes[cu->classId] +"_cuboid_"+std::to_string(cu-_cuboids.cbegin());
		if (_color_map.find(_classes[cu->classId])!=_color_map.end())
			oc.color = _color_map.at(_classes[cu->classId]);
		else
			oc.color = _default_color;
		oc.id = obj.id;
		ps.object_colors.emplace_back(oc);
		obj.primitives[0] = primitive;
		obj.primitive_poses[0] = pose;
		obj_list.emplace_back(obj);
	}

	if (!obj_list.empty()){
		ps.world.collision_objects.assign(obj_list.begin(), obj_list.end());
		last_obj_list = std::move(obj_list);
		_obj_pub.publish(ps);
	}
}

void RecognitionUtil::pubVisualMesh(const tf2::Stamped<tf2::Transform>& trans) const{
	static std::vector<moveit_msgs::CollisionObject> last_obj_list;

	// publish
	moveit_msgs::PlanningScene ps;
	ps.is_diff = true;

	// remove old objects
	if (!last_obj_list.empty()){
		for (int i=0; i<last_obj_list.size(); ++i)
			last_obj_list[i].operation = last_obj_list[i].REMOVE;
		ps.world.collision_objects = std::move(last_obj_list);
		last_obj_list.clear();
		_obj_pub.publish(ps);
	}

	if (_cuboids.empty())
		return;

	std::vector<moveit_msgs::CollisionObject> obj_list;
	moveit_msgs::CollisionObject obj;
	obj.header.frame_id = _ref_robot_link;
	moveit_msgs::ObjectColor oc;
	obj.meshes.resize(1);
	obj.mesh_poses.resize(1);
	obj.operation = obj.ADD;
	geometry_msgs::Pose pose;
	tf2::Vector3 pos = trans.getOrigin();
	pose.position.x = pos.getX();
	pose.position.y = pos.getY();
	pose.position.z = pos.getZ();
	pose.orientation = tf2::toMsg(trans.getRotation());

	// add new objects
	for (std::vector<Cuboid>::const_iterator cu=_cuboids.cbegin(); cu != _cuboids.cend(); ++cu){
		if (cu->classId == -1)
			continue;
		// build cuboid's pointcloud to reconstruct to mesh
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator it=cu->cluster_idx.cbegin(); it!=cu->cluster_idx.cend(); ++it){
			for (std::vector<int>::const_iterator pit = _clusters[*it].indices.cbegin(); pit != _clusters[*it].indices.cend(); ++pit){
				cloud->push_back(_xyzrgb_ptr->at(*pit));
			}
		}

		// in camera_frame
		cloud->header.frame_id = _xyzrgb_ptr->header.frame_id;
		cloud->width = cloud->size();
		cloud->height = 1;
		cloud->is_dense = true;

  		// normal estimation
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(20);
		n.compute(*normals);

		// concatenate the XYZRGB and normal fields
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_N(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_N);

		// create search tree
		pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
		tree2->setInputCloud(cloud_N);

		// initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
		pcl::PolygonMesh triangles;

		// set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius(0.025);

		// set typical values for the parameters
		gp3.setMu(2.5);
		gp3.setMaximumNearestNeighbors(100);
		gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		// get result
		gp3.setInputCloud(cloud_N);
		gp3.setSearchMethod(tree2);
		gp3.reconstruct(triangles);

		// fill in mesh
		shape_msgs::Mesh mesh;
		pclMeshToShapeMsg(triangles, mesh);

		// construct obj_list
		obj.id = _classes[cu->classId] +"_mesh_"+std::to_string(cu-_cuboids.cbegin());
		if (_color_map.find(_classes[cu->classId])!=_color_map.end())
			oc.color = _color_map.at(_classes[cu->classId]);
		else
			oc.color = _default_color;
		oc.id = obj.id;
		ps.object_colors.emplace_back(oc);
		obj.meshes[0] = mesh;
		obj.mesh_poses[0] = pose;
		obj_list.emplace_back(obj);
	}
	if (!obj_list.empty()){
		ps.world.collision_objects.assign(obj_list.begin(), obj_list.end());
		last_obj_list = std::move(obj_list);
		_obj_pub.publish(ps);
	}
}

void RecognitionUtil::pclMeshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::Mesh& mesh) const{
	// pcl mesh msg
	pcl_msgs::PolygonMesh pcl_msg_mesh;
	pcl_conversions::fromPCL(in, pcl_msg_mesh);

	// iterate through poincloud2
	sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

	// construct mesh vertices
	mesh.vertices.resize(pcl_msg_mesh.cloud.height * pcl_msg_mesh.cloud.width);
	for (std::vector<geometry_msgs::Point>::iterator it=mesh.vertices.begin(); it!=mesh.vertices.end(); ++it, ++pt_iter) {
		it->x = pt_iter[0];
		it->y = pt_iter[1];
		it->z = pt_iter[2];
	}

	// construct mesh triangles
	for (std::vector<pcl_msgs::Vertices>::const_iterator it = pcl_msg_mesh.polygons.cbegin(); it != pcl_msg_mesh.polygons.cend(); ++it){
		if (it->vertices.size() < 3){
			ROS_WARN_STREAM("Not enough points in polygon. Ignoring it.");
			continue;
		}
		mesh.triangles.emplace_back();
		mesh.triangles.back().vertex_indices[0] = it->vertices[0];
		mesh.triangles.back().vertex_indices[1] = it->vertices[1];
		mesh.triangles.back().vertex_indices[2] = it->vertices[2];
	}
}
/*
void RecognitionUtil::pclMeshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::Marker& m) const{
	m.ns = "visual";
	m.lifetime = ros::Duration();
	m.type = visualization_msgs::Marker::TRIANGLE_LIST;
	m.header.frame_id = in.cloud.header.frame_id;
	m.header.stamp = ros::Time::now();
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = 1.0;
	m.scale.y = 1.0;
	m.scale.z = 1.0;

	shape_msgs::Mesh mesh;
	pclMeshToShapeMsg(in, mesh);
	m.points.resize(mesh.triangles.size()*3);

	int i=0;
	for (int idx=0; idx<mesh.triangles.size(); ++idx)
		for (int subidx = 0; subidx<3; ++subidx)
			m.points[i++] = mesh.vertices[mesh.triangles[idx].vertex_indices[subidx]];
}
*/
void RecognitionUtil::generateCuboids(const tf2::Stamped<tf2::Transform>& trans){
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
				_bb_map[std::make_pair(c,r)].emplace_back(i);
			}
		}
	}
}

void RecognitionUtil::updateTable(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const tf2::Stamped<tf2::Transform>& trans){
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

	visualization_msgs::MarkerArray vis;
	visualization_msgs::Marker draw_pts;
	draw_pts.ns = "table";
	draw_pts.action = visualization_msgs::Marker::ADD;
	draw_pts.type = visualization_msgs::Marker::POINTS;
	draw_pts.id = 0;
	draw_pts.scale.x = 0.1;
	draw_pts.scale.y = 0.1;
	draw_pts.color.g = 1.0;
	draw_pts.color.a = 1.0;
	draw_pts.header.frame_id = _ref_robot_link;
	geometry_msgs::Point pt;
	pt.z = _table.center.z;
	cv::Point2f vertices[4];
	table.points(vertices);
	for (int i=0; i<4; ++i){
		pt.x = vertices[i].x;
		pt.y = vertices[i].y;
		draw_pts.points.emplace_back(pt);
	}
	vis.markers.emplace_back(draw_pts);
	_marker_pub.publish(vis);
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
	ROS_ERROR_STREAM("Table info: " << _table.center.x <<","<< _table.center.y <<","<< _table.center.z <<","<< _table.width <<","<< _table.length <<","<< _table.depth <<","<< _table.angle);

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
