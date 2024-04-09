#include <iostream>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
// #include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_types.h"
#include <pcl/filters/voxel_grid.h>

using namespace laser_geometry;
using namespace rclcpp;

class Laser2Pcd : public rclcpp::Node
{
public:
  Laser2Pcd()
  : Node("laser2pcd")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/feature_scan", 1, std::bind(&Laser2Pcd::scanCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd", 1);
    LaserProjection projector_;
    // tf2::TransformListener tfListener_;
  }
private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    publisher_->publish(cloud);
  }
  Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  LaserProjection projector_;
  // tf2::TransformListener tfListener_;
};


int main(int argc, char ** argv)
{
  init(argc, argv);
  spin(std::make_shared<Laser2Pcd>());
  shutdown();
  return 0;
}
