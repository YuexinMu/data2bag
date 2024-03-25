//
// Created by myx on 2024/3/22.
//

// ref: https://github.com/umtclskn/ros2_kitti_publishers

#include <memory>
#include "data2bag/data2bag_kitti.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  /*
   * set the standard output stream to unbuffered,
   * causing output to occur immediately without for the buffer to fill
   * */
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  auto node = std::make_shared<rclcpp::Node>("data2bag_kitti");

  node->declare_parameter("path_point_cloud", " ");
  node->declare_parameter("path_image_gray_left", " ");
  node->declare_parameter("path_image_gray_right", " ");
  node->declare_parameter("path_image_color_left", " ");
  node->declare_parameter("path_image_color_right", " ");
  node->declare_parameter("path_oxts", " ");

  node->declare_parameter("point_cloud_topic_name", "kitti/point_cloud");
  node->declare_parameter("image_gray_left_topic_name", "kitti/image/gray/left");
  node->declare_parameter("image_gray_right_topic_name", "kitti/image/gray/right");
  node->declare_parameter("image_color_left_topic_name", "kitti/image/color/left");
  node->declare_parameter("image_color_right_topic_name", "kitti/image/color/right");
  node->declare_parameter("imu_topic_name", "kitti/imu");
  node->declare_parameter("nav_sat_fix_topic_name", "kitti/nav_sat_fix");
  node->declare_parameter("marker_arr_topic_name", "kitti/marker_array");

  node->declare_parameter("point_cloud_frame_id", "base_link");
  node->declare_parameter("image_gray_left_frame_id", "base_link");
  node->declare_parameter("image_gray_right_frame_id", "base_link");
  node->declare_parameter("image_color_left_frame_id", "base_link");
  node->declare_parameter("image_color_right_frame_id", "base_link");
  node->declare_parameter("imu_frame_id", "base_link");
  node->declare_parameter("nav_sat_frame_id", "base_link");
  node->declare_parameter("marker_array_frame_id", "base_link");

  node->declare_parameter("publish_rate", 10);

  data2bag::Data2BagKitti data_2_bag_kitti(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

using namespace cv;
using namespace std::chrono_literals;

namespace data2bag
{

Data2BagKitti::Data2BagKitti(std::shared_ptr<rclcpp::Node>& node) : node_(node), file_index_(0)
{
  node_->get_parameter("path_point_cloud", path_point_cloud_);
  node_->get_parameter("path_image_gray_left", path_image_gray_left_);
  node_->get_parameter("path_image_gray_right", path_image_gray_right_);
  node_->get_parameter("path_image_color_left", path_image_color_left_);
  node_->get_parameter("path_image_color_right", path_image_color_right_);
  node_->get_parameter("path_oxts", path_oxts_);

  node_->get_parameter("point_cloud_topic_name", pub_topic_name_pc_);
  node_->get_parameter("image_gray_left_topic_name", pub_topic_name_img_gray_l_);
  node_->get_parameter("image_gray_right_topic_name", pub_topic_name_img_gray_r_);
  node_->get_parameter("image_color_left_topic_name", pub_topic_name_img_color_l_);
  node_->get_parameter("image_color_right_topic_name", pub_topic_name_img_color_r_);
  node_->get_parameter("imu_topic_name", pub_topic_name_imu_);
  node_->get_parameter("nav_sat_fix_topic_name", pub_topic_name_nav_sat_fix_);
  node_->get_parameter("marker_arr_topic_name", pub_topic_name_marker_arr_);

  node_->get_parameter("publish_rate", pub_rate_);

  node_->get_parameter("point_cloud_frame_id", frame_id_pc_);
  node_->get_parameter("image_gray_left_frame_id", frame_id_img_gray_l_);
  node_->get_parameter("image_gray_right_frame_id", frame_id_img_gray_r_);
  node_->get_parameter("image_color_left_frame_id", frame_id_img_color_l_);
  node_->get_parameter("image_color_right_frame_id", frame_id_img_color_r_);
  node_->get_parameter("imu_frame_id", frame_id_imu_);
  node_->get_parameter("nav_sat_frame_id", frame_id_nav_sat_fix_);
  node_->get_parameter("marker_array_frame_id", frame_id_marker_arr_);

  publisher_point_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);
  publisher_image_gray_left_ = node_->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/left", 10);
  publisher_image_gray_right_ = node_->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/right", 10);
  publisher_image_color_left_ = node_->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/left", 10);
  publisher_image_color_right_ = node_->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/right", 10);
  publisher_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 10);
  publisher_nav_sat_fix_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/nav_sat_fix", 10);
  publisher_marker_array_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/marker_array", 10);

  for (int type_index = 0; type_index != 6; type_index++)
  {
    size_t index = 0;
    files_index_.push_back(index);
  }

  create_data_file_names();

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000 / pub_rate_),
                                    std::bind(&Data2BagKitti::timer_callback, this));
}

Data2BagKitti::~Data2BagKitti()
{
}

void Data2BagKitti::create_data_file_names()
{
  for (int type_index = 0; type_index != 6; type_index++)
  {
    PublisherType type = static_cast<PublisherType>(type_index);
    std::vector<std::string> file_names = get_filenames(type);

    try
    {
      // Loop through the specified directory
      for (const auto& entry : std::filesystem::directory_iterator(get_path(type)))
      {
        if (entry.is_regular_file())
        {
          file_names.push_back(entry.path().filename());
          files_index_[type_index]++;
        }
      }

      // Order lidar file names
      std::sort(file_names.begin(), file_names.end(), [](const auto& lhs, const auto& rhs) { return lhs < rhs; });
      set_filenames(type, file_names);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "File: " << get_path(type) << " path not found.");
    }
  }
}

void Data2BagKitti::timer_callback()
{
  // 01- KITTI POINT CLOUDS2 MESSAGES
  if (file_index_ >= files_index_[PublisherType::POINT_CLOUD])
  {
    RCLCPP_INFO(node_->get_logger(), "pcl data read complete, the last file NO: '%d'", file_index_ - 1);
  }
  else
  {
    sensor_msgs::msg::PointCloud2 point_cloud2_msg;
    std::string point_cloud_path = path_point_cloud_ + file_names_point_cloud_[file_index_];
    convert_pcl_to_pointcloud2(point_cloud2_msg, point_cloud_path);
    publisher_point_cloud_->publish(point_cloud2_msg);
  }

  // 02- KITTI IMAGE MESSAGES
  if (file_index_ >= files_index_[PublisherType::IMAGE_LEFT_GRAY])
  {
    RCLCPP_INFO(node_->get_logger(), "image gray left data read complete, the last file NO: '%d'", file_index_ - 1);
  }
  else
  {
    auto image_message_gray_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_gray_left = path_image_gray_left_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_gray_left, img_pat_gray_left, frame_id_img_gray_l_);
    publisher_image_gray_left_->publish(std::move(image_message_gray_left));
  }
  if (file_index_ >= files_index_[PublisherType::IMAGE_RIGHT_GRAY])
  {
    RCLCPP_INFO(node_->get_logger(), "image gray right data read complete, the last file NO: '%d'", file_index_ - 1);
  }
  else
  {
    auto image_message_gray_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_gray_right = path_image_gray_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_gray_right, img_pat_gray_right, frame_id_img_gray_r_);
    publisher_image_gray_right_->publish(std::move(image_message_gray_right));
  }
  if (file_index_ >= files_index_[PublisherType::IMAGE_LEFT_COLOR])
  {
    RCLCPP_INFO(node_->get_logger(), "image color left data read complete, the last file NO: '%d'", file_index_ - 1);
  }
  else
  {
    auto image_message_color_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_left = path_image_color_right_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_color_left, img_pat_color_left, frame_id_img_color_l_);
    publisher_image_color_left_->publish(std::move(image_message_color_left));
  }
  if (file_index_ >= files_index_[PublisherType::IMAGE_RIGHT_COLOR])
  {
    RCLCPP_INFO(node_->get_logger(), "image color right data read complete, the last file NO: '%d'", file_index_ - 1);
  }
  else
  {
    auto image_message_color_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_right = path_image_color_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_color_right, img_pat_color_right, frame_id_img_color_r_);
    publisher_image_color_right_->publish(std::move(image_message_color_right));
  }

  // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE START//
  if (file_index_ >= files_index_[PublisherType::ODOMETRY])
  {
    RCLCPP_INFO(node_->get_logger(), "otxs data read complete, the last file NO: '%d'", file_index_ - 1);
  }
  else
  {
    std::string oxts_file_name = path_oxts_ + file_names_oxts_[file_index_];
    const std::string delimiter = " ";
    std::vector<std::string> oxts_parsed_array = parse_file_data_into_string_array(oxts_file_name, delimiter);

    auto nav_sat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    prepare_navsatfix_msg(oxts_parsed_array, *nav_sat_fix_msg);
    publisher_nav_sat_fix_->publish(std::move(nav_sat_fix_msg));

    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    prepare_imu_msg(oxts_parsed_array, *imu_msg);
    publisher_imu_->publish(std::move(imu_msg));

    auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    prepare_marker_array_msg(oxts_parsed_array, *marker_array_msg);
    publisher_marker_array_->publish(std::move(marker_array_msg));
  }

  bool end_flag_ = true;
  for (int type_index = 0; type_index < 6; type_index++)
  {
    if (file_index_ < files_index_[type_index])
    {
      end_flag_ = false;
    }
  }
  if (end_flag_)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "all file read over!");
    //    rclcpp::shutdown();
    timer_->cancel();
  }

  file_index_++;
}

std::string Data2BagKitti::get_path(PublisherType publisher_type)
{
  //  RCLCPP_INFO(node_->get_logger(), "get_path: '%i'", publisher_type);
  std::string path;
  if (publisher_type == PublisherType::POINT_CLOUD)
  {
    path = path_point_cloud_;
  }
  else if (publisher_type == PublisherType::IMAGE_LEFT_GRAY)
  {
    path = path_image_gray_left_;
  }
  else if (publisher_type == PublisherType::IMAGE_RIGHT_GRAY)
  {
    path = path_image_gray_right_;
  }
  else if (publisher_type == PublisherType::IMAGE_LEFT_COLOR)
  {
    path = path_image_color_left_;
  }
  else if (publisher_type == PublisherType::IMAGE_RIGHT_COLOR)
  {
    path = path_image_color_right_;
  }
  else if (publisher_type == PublisherType::ODOMETRY)
  {
    path = path_oxts_;
  }
  return path;
}

std::vector<std::string> Data2BagKitti::get_filenames(PublisherType publisher_type)
{
  if (publisher_type == PublisherType::POINT_CLOUD)
  {
    return file_names_point_cloud_;
  }
  else if (publisher_type == PublisherType::IMAGE_LEFT_GRAY)
  {
    return file_names_image_gray_left_;
  }
  else if (publisher_type == PublisherType::IMAGE_RIGHT_GRAY)
  {
    return file_names_image_gray_right_;
  }
  else if (publisher_type == PublisherType::IMAGE_LEFT_COLOR)
  {
    return file_names_image_color_left_;
  }
  else if (publisher_type == PublisherType::IMAGE_RIGHT_COLOR)
  {
    return file_names_image_color_right_;
  }
  return file_names_oxts_;
}

void Data2BagKitti::set_filenames(PublisherType publisher_type, std::vector<std::string> file_names)
{
  if (publisher_type == PublisherType::POINT_CLOUD)
  {
    file_names_point_cloud_ = file_names;
  }
  else if (publisher_type == PublisherType::IMAGE_LEFT_GRAY)
  {
    file_names_image_gray_left_ = file_names;
  }
  else if (publisher_type == PublisherType::IMAGE_RIGHT_GRAY)
  {
    file_names_image_gray_right_ = file_names;
  }
  else if (publisher_type == PublisherType::IMAGE_LEFT_COLOR)
  {
    file_names_image_color_left_ = file_names;
  }
  else if (publisher_type == PublisherType::IMAGE_RIGHT_COLOR)
  {
    file_names_image_color_right_ = file_names;
  }
  else if (publisher_type == PublisherType::ODOMETRY)
  {
    file_names_oxts_ = file_names;
  }
}

void Data2BagKitti::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2& msg, const std::string path)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  // Reading in binary input mode
  std::fstream input(path, std::ios::in | std::ios::binary);
  if (!input.good())
  {
    RCLCPP_INFO(node_->get_logger(), "Could not read Velodyne's point cloud. Check your file path!");
    exit(EXIT_FAILURE);
  }
  // seek get: read file from first pose in start
  input.seekg(0, std::ios::beg);

  for (int i = 0; input.good() && !input.eof(); i++)
  {
    pcl::PointXYZI point;
    input.read((char*)&point.x, 3 * sizeof(float));
    input.read((char*)&point.intensity, sizeof(float));
    cloud.push_back(point);
  }

  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id_pc_;
  msg.header.stamp = node_->now();
}

void Data2BagKitti::convert_image_to_msg(sensor_msgs::msg::Image& msg, const std::string path,
                                         const std::string frame_id)
{
  Mat frame;
  frame = imread(path);
  if (frame.empty())  // Check for invalid input
  {
    RCLCPP_ERROR(node_->get_logger(), "Image does not exist. Check your files path!");
    rclcpp::shutdown();
  }

  msg.height = frame.rows;
  msg.width = frame.cols;
  std::string type = mat_type2encoding(frame.type());
  msg.encoding = type;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = frame_id;
  msg.header.stamp = node_->now();
}

void Data2BagKitti::prepare_imu_msg(std::vector<std::string>& oxts_tokenized_array, sensor_msgs::msg::Imu& msg)
{
  msg.header.frame_id = frame_id_imu_;
  msg.header.stamp = node_->now();

  //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
  //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
  //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
  msg.linear_acceleration.x = std::atof(oxts_tokenized_array[11].c_str());
  msg.linear_acceleration.y = std::atof(oxts_tokenized_array[12].c_str());
  msg.linear_acceleration.z = std::atof(oxts_tokenized_array[13].c_str());

  //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
  //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
  //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
  msg.angular_velocity.x = std::atof(oxts_tokenized_array[8].c_str());
  msg.angular_velocity.y = std::atof(oxts_tokenized_array[9].c_str());
  msg.angular_velocity.z = std::atof(oxts_tokenized_array[10].c_str());

  //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  tf2::Quaternion q;
  q.setRPY(std::atof(oxts_tokenized_array[3].c_str()), std::atof(oxts_tokenized_array[4].c_str()),
           std::atof(oxts_tokenized_array[5].c_str()));

  msg.orientation.x = q.getX();
  msg.orientation.y = q.getY();
  msg.orientation.z = q.getZ();
  msg.orientation.w = q.getW();
}

void Data2BagKitti::prepare_navsatfix_msg(std::vector<std::string>& oxts_tokenized_array,
                                          sensor_msgs::msg::NavSatFix& msg)
{
  msg.header.frame_id = frame_id_nav_sat_fix_;
  msg.header.stamp = node_->now();

  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;

  msg.latitude = std::atof(oxts_tokenized_array[0].c_str());
  msg.longitude = std::atof(oxts_tokenized_array[1].c_str());
  msg.altitude = std::atof(oxts_tokenized_array[2].c_str());

  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  msg.position_covariance[0] = std::atof(oxts_tokenized_array[23].c_str());
  msg.position_covariance[1] = 0.0f;
  msg.position_covariance[2] = 0.0f;
  msg.position_covariance[3] = 0.0f;
  msg.position_covariance[4] = std::atof(oxts_tokenized_array[23].c_str());
  msg.position_covariance[5] = 0.0f;
  msg.position_covariance[6] = 0.0f;
  msg.position_covariance[7] = 0.0f;
  msg.position_covariance[8] = std::atof(oxts_tokenized_array[23].c_str());
}

// https://github.com/iralabdisco/kitti_player/blob/public/src/kitti_player.cpp#L1252
// https://github.com/chrberger/WGS84toCartesian
void Data2BagKitti::prepare_marker_array_msg(std::vector<std::string>& oxts_tokenized_array,
                                             visualization_msgs::msg::MarkerArray& msg)
{
  const double lat = std::stod(oxts_tokenized_array[0]);
  const double lon = std::stod(oxts_tokenized_array[1]);

  std::array<double, 2> WGS84Reference{ lat, lon };
  std::array<double, 2> WGS84Position{ lat, lon };
  std::array<double, 2> result{ wgs84::toCartesian(WGS84Reference, WGS84Position) };

  visualization_msgs::msg::Marker RTK_MARKER;

  static int gps_track = 1;
  RTK_MARKER.header.frame_id = frame_id_marker_arr_;
  RTK_MARKER.header.stamp = node_->now();
  RTK_MARKER.ns = "RTK_MARKER";
  RTK_MARKER.id = gps_track++;  // unused
  RTK_MARKER.type = visualization_msgs::msg::Marker::CYLINDER;
  RTK_MARKER.action = visualization_msgs::msg::Marker::ADD;
  RTK_MARKER.pose.orientation.w = 1;
  RTK_MARKER.scale.x = 0.5;
  RTK_MARKER.scale.y = 0.5;
  RTK_MARKER.scale.z = 3.5;
  RTK_MARKER.color.a = 0.80;
  RTK_MARKER.color.r = 0;
  RTK_MARKER.color.g = 0.0;
  RTK_MARKER.color.b = 1.0;
  RTK_MARKER.pose.position.x = result[0];
  RTK_MARKER.pose.position.y = result[1];
  RTK_MARKER.pose.position.z = 0;

  msg.markers.push_back(RTK_MARKER);
}

std::vector<std::string> Data2BagKitti::parse_file_data_into_string_array(std::string file_name, std::string delimiter)
{
  std::ifstream f(file_name.c_str());  // taking file as inputstream

  if (!f.good())
  {
    RCLCPP_INFO(node_->get_logger(), "Could not read OXTS data. Check your file path!");
    exit(EXIT_FAILURE);
  }

  std::string file_content_string;
  if (f)
  {
    std::ostringstream ss;
    ss << f.rdbuf();  // reading data
    file_content_string = ss.str();
  }

  // https://www.codegrepper.com/code-examples/whatever/c%2B%2B+how+to+tokenize+a+string
  std::vector<std::string> tokens;
  size_t first = 0;
  while (first < file_content_string.size())
  {
    size_t second = file_content_string.find_first_of(delimiter, first);
    // first has index of start of token
    // second has index of end of token + 1;
    if (second == std::string::npos)
    {
      second = file_content_string.size();
    }
    std::string token = file_content_string.substr(first, second - first);
    tokens.push_back(token);
    first = second + 1;
  }

  return tokens;
}

std::string Data2BagKitti::mat_type2encoding(int mat_type)
{
  switch (mat_type)
  {
    // eight bit unsigned single channel mat
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

}  // namespace data2bag
