//
// Created by myx on 2024/3/20.
//

// ref: https://github.com/umtclskn/ros2_kitti_publishers

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "WGS84toCartesian.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <cstdlib>

template <typename T>
T getParam(rclcpp::Node& node, const std::string& param_name, const T& default_val)
{
  T param_val;

  node.declare_parameter(param_name, default_val);
  node.get_parameter(param_name, param_val);
  return param_val;
}

namespace data2bag
{
class Data2BagKitti
{
public:
  enum PublisherType
  {
    POINT_CLOUD = 0,
    IMAGE_LEFT_GRAY = 1,
    IMAGE_RIGHT_GRAY = 2,
    IMAGE_LEFT_COLOR = 3,
    IMAGE_RIGHT_COLOR = 4,
    ODOMETRY = 5
  };

  Data2BagKitti(std::shared_ptr<rclcpp::Node>& node);
  ~Data2BagKitti();

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t start_frame_id_;
  size_t end_frame_id_;
  unsigned int pub_rate_;

  void get_data_path();
  void create_data_file_names();
  void timer_callback();

  std::string get_path(PublisherType publisher_type);
  std::vector<std::string> get_filenames(PublisherType publisher_type);
  void set_filenames(PublisherType publisher_type, std::vector<std::string> file_names);

  void convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2& msg, const std::string path);
  void convert_image_to_msg(sensor_msgs::msg::Image& msg, const std::string path, const std::string frame_id);
  void prepare_imu_msg(std::vector<std::string>& oxts_tokenized_array, sensor_msgs::msg::Imu& msg);
  void prepare_navsatfix_msg(std::vector<std::string>& oxts_tokenized_array, sensor_msgs::msg::NavSatFix& msg);
  void prepare_marker_array_msg(std::vector<std::string>& oxts_tokenized_array,
                                visualization_msgs::msg::MarkerArray& msg);
  std::vector<std::string> parse_file_data_into_string_array(std::string file_name, std::string delimiter);
  std::string mat_type2encoding(int mat_type);

  std::string data_path_;
  std::string pub_topic_name_pc_;
  std::string pub_topic_name_img_gray_l_;
  std::string pub_topic_name_img_gray_r_;
  std::string pub_topic_name_img_color_l_;
  std::string pub_topic_name_img_color_r_;
  std::string pub_topic_name_imu_;
  std::string pub_topic_name_nav_sat_fix_;
  std::string pub_topic_name_marker_arr_;

  std::string frame_id_pc_;
  std::string frame_id_img_gray_l_;
  std::string frame_id_img_gray_r_;
  std::string frame_id_img_color_l_;
  std::string frame_id_img_color_r_;
  std::string frame_id_imu_;
  std::string frame_id_nav_sat_fix_;
  std::string frame_id_marker_arr_;

  std::string path_point_cloud_;
  std::string path_image_gray_left_;
  std::string path_image_gray_right_;
  std::string path_image_color_left_;
  std::string path_image_color_right_;
  std::string path_oxts_;

  std::vector<std::string> file_names_point_cloud_;
  std::vector<std::string> file_names_image_gray_left_;
  std::vector<std::string> file_names_image_gray_right_;
  std::vector<std::string> file_names_image_color_left_;
  std::vector<std::string> file_names_image_color_right_;
  std::vector<std::string> file_names_oxts_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;  // velodyne point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      publisher_image_gray_left_;  // left rectified grayscale image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      publisher_image_gray_right_;  // right rectified grayscale image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      publisher_image_color_left_;  // left rectified color image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      publisher_image_color_right_;                                           // right rectified color image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_odometry_;  // oxts odometry publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;         // oxts odometry publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_nav_sat_fix_;  // oxts odometry publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_;  // oxts odometry publisher
};
}  // namespace data2bag