#include <iostream>
#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl_ros/point_cloud.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <hdl_global_localization/SetGlobalLocalizationEngine.h>

#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_teaser.hpp>

namespace hdl_global_localization {

class GlobalLocalizationNode {
public:
  GlobalLocalizationNode() : nh(), private_nh("~") {
    set_engine(private_nh.param<std::string>("global_localization_engine", "FPFH_RANSAC"));
    // 设置3个服务
    set_engine_server = private_nh.advertiseService("set_engine", &GlobalLocalizationNode::set_engine, this);
    set_global_map_server = private_nh.advertiseService("set_global_map", &GlobalLocalizationNode::set_global_map, this);
    query_server = private_nh.advertiseService("query", &GlobalLocalizationNode::query, this);
  }

private:
  // 降采样
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
  }
  // 设置engine
  bool set_engine(const std::string& engine_name) {
    if (engine_name == "BBS") {
      engine.reset(new GlobalLocalizationBBS(private_nh));
    } else if (engine_name == "FPFH_RANSAC") {
      engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(private_nh));
    }
#ifdef TEASER_ENABLED
    else if (engine_name == "FPFH_TEASER") {
      engine.reset(new GlobalLocalizationEngineFPFH_Teaser(private_nh));
    }
#endif
    else {
      ROS_ERROR_STREAM("Unknown Global Localization Engine:" << engine_name);
      return false;
    }
    // 调用set_global_map设置全局变量globalmap
    if (global_map) {
      engine->set_global_map(global_map);
    }

    return true;
  }
  // SetGlobalLocalizationEngine.srv
  // TODO: 为什么有Request Response，srv里只有request engine_name
  // A: 自定义服务类型，编译后，会自动生成如下边形参里的类，调用时::Request或SetGlobalMapRequest都可以
  bool set_engine(SetGlobalLocalizationEngine::Request& req, SetGlobalLocalizationEngine::Response& res) {
    ROS_INFO_STREAM("Set Global Localization Engine");
    return set_engine(req.engine_name.data); // 上边的set_engine TODO: engine_name.data哪来的
                                                                // A: 类型是std_msgs/String 里边有data成员变量
  }
  // TODO: 这两个类哪里来的，/srv下只有SetGlobalMap.srv，里边只有request globalmap
  // A: 
  bool set_global_map(SetGlobalMapRequest& req, SetGlobalMapResponse& res) {
    ROS_INFO_STREAM("Global Map Received");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.global_map, *cloud);
    cloud = downsample(cloud, private_nh.param<double>("globalmap_downsample_resolution", 0.5));

    globalmap_header = req.global_map.header;
    global_map = cloud;
    // 调用GlobalLocalizationEngine类中的set_global_map
    engine->set_global_map(global_map);

    ROS_INFO_STREAM("DONE");

    return true;
  }

  // request: max_num_candidates、cloud
  // response: header、globalmap_header、float64[] inlier_fractions、float64[] errors、poses
  bool query(QueryGlobalLocalizationRequest& req, QueryGlobalLocalizationResponse& res) {
    ROS_INFO_STREAM("Query Global Localization");
    if (global_map == nullptr) {
      ROS_WARN_STREAM("No Globalmap");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.cloud, *cloud);
    cloud = downsample(cloud, private_nh.param<double>("query_downsample_resolution", 0.5));
    // 调用GlobalLocalizationEngine类中的query
    auto results = engine->query(cloud, req.max_num_candidates);

    res.inlier_fractions.resize(results.results.size());
    res.errors.resize(results.results.size());
    res.poses.resize(results.results.size());

    res.header = req.cloud.header;
    res.globalmap_header = globalmap_header;

    // 将engine->query返回的结果存进response
    for (int i = 0; i < results.results.size(); i++) {
      const auto& result = results.results[i];
      Eigen::Quaternionf quat(result->pose.linear());
      Eigen::Vector3f trans(result->pose.translation());

      res.inlier_fractions[i] = result->inlier_fraction;
      res.errors[i] = result->error;
      res.poses[i].orientation.x = quat.x();
      res.poses[i].orientation.y = quat.y();
      res.poses[i].orientation.z = quat.z();
      res.poses[i].orientation.w = quat.w();

      res.poses[i].position.x = trans.x();
      res.poses[i].position.y = trans.y();
      res.poses[i].position.z = trans.z();
    }

    return !results.results.empty();
  }

private:
  // ROS 3个句柄，3个server，1个消息头
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::ServiceServer set_engine_server;
  ros::ServiceServer set_global_map_server;
  ros::ServiceServer query_server;

  std_msgs::Header globalmap_header;

  // globalmap指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;

  std::unique_ptr<GlobalLocalizationEngine> engine;
};

}  // namespace hdl_global_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "hdl_global_localization");
  ROS_INFO_STREAM("Start Global Localization Node");
  hdl_global_localization::GlobalLocalizationNode node;
  ros::spin();

  return 0;
}