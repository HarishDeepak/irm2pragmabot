// Copyright (c) 2019-2024 Autonomous Robots and Cognitive Systems Laboratory
// Universidad de Costa Rica
// Authors: Daniel Garcia Vaglio degv364@gmail.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


// General includes
#include <cstdint>
#include <mutex>
#include <string>
#include <queue>
#include <deque>
#include <unordered_map>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/objdetect/aruco_dictionary.hpp>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/camera_common.hpp"
//#include <time.hpp>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TODO: add tf2? or should tf2 be in a separate node that takes the pose?
//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
//#include "tf2_ros/buffer.h"
//#include "tf2_ros/transform_listener.h"
//#include "tf2_ros/transform_broadcaster.h"

// Own includes
//#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "aruco_detector_msgs/msg/marker_poses_stamped.hpp"
#include "aruco_detector_msgs/msg/status.hpp"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

/*
  Convert a pair of vectors into a Pose. One vector indicates the rotation and the
  other vector indicates the position.
 */
geometry_msgs::msg::Pose vectors_to_pose(const cv::Vec3d & in_rot, const cv::Vec3d & in_pos){
  geometry_msgs::msg::Pose pose_out;

  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(in_rot, rot);

  tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
    rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
    rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
  tf2::Quaternion tf_quat;
  tf_rot.getRotation(tf_quat);

  pose_out.position.x = in_pos[0];
  pose_out.position.y = in_pos[1];
  pose_out.position.z = in_pos[2];
  tf2::convert(tf_quat, pose_out.orientation);

  return pose_out;
}

// Helper to access nicely the different dictionaries
const std::unordered_map<std::string, int> ARUCO_DICT_MAP = {
  {"4x4_50", cv::aruco::DICT_4X4_50},
  {"4x4_100", cv::aruco::DICT_4X4_100},
  {"4x4_250", cv::aruco::DICT_4X4_250},
  {"4x4_1000", cv::aruco::DICT_4X4_1000},
  {"5x5_50", cv::aruco::DICT_5X5_50},
  {"5x5_100", cv::aruco::DICT_5X5_100},
  {"5x5_250", cv::aruco::DICT_5X5_250},
  {"5x5_1000", cv::aruco::DICT_5X5_1000},
  {"6x6_50", cv::aruco::DICT_6X6_50},
  {"6x6_100", cv::aruco::DICT_6X6_100},
  {"6x6_250", cv::aruco::DICT_6X6_250},
  {"6x6_1000", cv::aruco::DICT_6X6_1000},
  {"7x7_50", cv::aruco::DICT_7X7_50},
  {"7x7_100", cv::aruco::DICT_7X7_100},
  {"7x7_250", cv::aruco::DICT_7X7_250},
  {"7x7_1000", cv::aruco::DICT_7X7_1000},
};


class ArucoDetector : public rclcpp::Node{
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
public:
  ArucoDetector() : Node("aruco_detector"){
    this->got_cam_info = false;
    this->received_images = 0;
    this->processed_images = 0;

    // Image pre-processing
    this->declare_parameter("resize_factor", 1.0);

    // Create the Aruco detector stuff
    this->declare_parameter("marker_size", 0.019);
    double marker_size = this->get_parameter("marker_size").as_double();
    marker_obj_points = cv::Mat(4, 1, CV_32FC3);
    marker_obj_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size / 2.f, marker_size / 2.f, 0);
    marker_obj_points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_size / 2.f, marker_size / 2.f, 0);
    marker_obj_points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_size / 2.f, -marker_size / 2.f, 0);
    marker_obj_points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_size / 2.f, -marker_size / 2.f, 0);

    this->declare_parameter("aruco_dict", "4x4_50");
    auto dict_name = this->get_parameter("aruco_dict").as_string();
    if (ARUCO_DICT_MAP.find(dict_name) == ARUCO_DICT_MAP.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Unsupported dictionary name: " << dict_name);
      return;
    }
    this->dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary
                                                          (ARUCO_DICT_MAP.at(dict_name)));
    // TODO: Create ROS parameters for this
    this->detector_parameters = cv::makePtr<cv::aruco::DetectorParameters>();
    //detector = cv::aruco::ArucoDetector(dictionary, detectorParams);

    // Camera intrinsics
    this->declare_parameter("image_is_rectified", false);
    this->image_is_rectified = this->get_parameter("image_is_rectified").as_bool();
    this->camera_matrix = cv::Mat(3, 3, CV_64FC1);
    this->distortion_coeffs = cv::Mat(4, 1, CV_64FC1, cv::Scalar(0));

    // ROS COmmunications
    this->general_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = general_group;
#ifdef COMPLEX_QOS  // TODO: Add parameters for all of this
    rmw_qos_profile_t image_sub_qos = rmw_qos_profile_default;
    image_sub_qos.reliability =
      static_cast<rmw_qos_reliability_policy_t>(image_sub_qos_reliability_);
    image_sub_qos.durability = static_cast<rmw_qos_durability_policy_t>(image_sub_qos_durability_);
    image_sub_qos.depth = image_sub_qos_depth_;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_sub_qos), image_sub_qos);
    this->img_sub = this->create_subscription<sensor_msgs::msg::Image>
      ("image", qos, bind(&ArucoDetector::read_image_callback, this, _1), options);
#else
    this->img_sub = this->create_subscription<sensor_msgs::msg::Image>
      ("image", 10, bind(&ArucoDetector::read_image_callback, this, _1), options);
#endif

    // Camera info sub
    this->camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>
      ("camera_info", 1, bind(&ArucoDetector::camera_info_callback, this, _1));

    // Publisher for detected poses
    this->detection_pub = this->create_publisher<aruco_detector_msgs::msg::MarkerPosesStamped>
      ("aruco_detection", 10);

    this->status_pub = this->create_publisher<aruco_detector_msgs::msg::Status>
      ("status", 10);

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    this->timer = this->create_wall_timer(10ms, bind(&ArucoDetector::timer_callback, this), this->general_group);
    this->status_timer = this->create_wall_timer(100ms, bind(&ArucoDetector::status_callback, this), this->general_group);
  }

private:
  //cv::aruco::ArucoDetector detector;

  // ROS specifics
  rclcpp::Publisher<aruco_detector_msgs::msg::MarkerPosesStamped>::SharedPtr detection_pub;
  rclcpp::Publisher<aruco_detector_msgs::msg::Status>::SharedPtr status_pub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr status_timer;
  rclcpp::CallbackGroup::SharedPtr general_group;

  // Detection
  mutex queue_mutex;
  std::queue<cv_bridge::CvImageConstPtr> images;
  cv::Mat marker_obj_points;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters;
  cv::Ptr<cv::aruco::Dictionary> dictionary;


  // Camera info
  mutex cam_info_mutex;
  bool got_cam_info;
  bool image_is_rectified;
  cv::Mat camera_matrix;
  cv::Mat distortion_coeffs;

  // status
  mutex status_mutex;
  uint64_t received_images;
  uint64_t processed_images;
  double last_image_proc_duration;
  std::deque<double> durations;

  void read_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {

    auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    double resize_factor = this->get_parameter("resize_factor").as_double();
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(0,0), resize_factor, resize_factor);

    queue_mutex.lock();
    this->received_images++;
    if (images.size() < 100) {
      images.push(cv_ptr);
      queue_mutex.unlock();
    }
    else {
      queue_mutex.unlock();
      RCLCPP_ERROR(this->get_logger(), "Image queue is too full, dropping images");
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info){
    lock_guard<mutex> guard(cam_info_mutex);
    if (this->image_is_rectified ) {
      for (int i = 0; i < 9; ++i) {
        this->camera_matrix.at<double>(i / 3, i % 3) = cam_info->p[i + i / 3];
      }
    } else {
      for (int i = 0; i < 9; ++i) {
        this->camera_matrix.at<double>(i / 3, i % 3) = cam_info->k[i];
      }
      this->distortion_coeffs = cv::Mat(cam_info->d, true);
    }

    this->got_cam_info = true;
  }

  void timer_callback() {
    size_t num_markers;
    vector<int> marker_ids;
    vector<vector<cv::Point2f>> marker_corners;

    cv::Mat internal_camera_matrix;
    cv::Mat internal_distortion_coeffs;

    auto message = aruco_detector_msgs::msg::MarkerPosesStamped();
    queue_mutex.lock();
    if (!images.empty() && this->got_cam_info) {
      auto start_time = this->now();
      this->processed_images++;
      auto img_ptr = images.front();
      images.pop();
      queue_mutex.unlock();

      message.header.stamp = img_ptr->header.stamp;
      message.header.frame_id = img_ptr->header.frame_id;

      cv::aruco::detectMarkers(img_ptr->image, this->dictionary, marker_corners, marker_ids, this->detector_parameters);
      num_markers = marker_corners.size();
      message.marker_ids = marker_ids;
      message.poses.resize(num_markers);

      vector<cv::Vec3d> rot_vecs(num_markers), pos_vecs(num_markers);

      cam_info_mutex.lock();
      internal_camera_matrix = cv::Mat(this->camera_matrix);
      internal_distortion_coeffs = cv::Mat(this->distortion_coeffs);
      cam_info_mutex.unlock();
      cv::parallel_for_(
        cv::Range(0, num_markers), [&](const cv::Range & range){
          for (size_t i = range.start; i< range.end; i++) {
            cv::solvePnP(
              this->marker_obj_points,
              marker_corners[i], internal_camera_matrix, internal_distortion_coeffs,
              rot_vecs[i], pos_vecs[i], false, cv::SOLVEPNP_IPPE_SQUARE);
            message.poses[i] = vectors_to_pose(rot_vecs[i], pos_vecs[i]);
            geometry_msgs::msg::TransformStamped t;

            // 1. Header: Matches your message (tracking_base_frame)
            t.header.stamp = this->now(); // or img_ptr->header.stamp
            t.header.frame_id = img_ptr->header.frame_id;

            // 2. Child Frame: The dynamic name (tracking_marker_frame)
            t.child_frame_id = "marker_" + std::to_string(marker_ids[i]);

            // 3. Translation: Map from Pose position
            t.transform.translation.x = message.poses[i].position.x;
            t.transform.translation.y = message.poses[i].position.y;
            t.transform.translation.z = message.poses[i].position.z;
            t.transform.rotation.x = message.poses[i].orientation.x;
            t.transform.rotation.y = message.poses[i].orientation.y;
            t.transform.rotation.z = message.poses[i].orientation.z;
            t.transform.rotation.w = message.poses[i].orientation.w;

            // Broadcast it!
            tf_broadcaster_->sendTransform(t);
          }
        }
                        );
      auto current_duration = this->now() - start_time;
      this->detection_pub->publish(message);
      status_mutex.lock();
      this->last_image_proc_duration = 1000 * current_duration.seconds();
      status_mutex.unlock();


    } else {
      queue_mutex.unlock();
    }
  }

  void status_callback() {
    auto message = aruco_detector_msgs::msg::Status();
    status_mutex.lock();
    queue_mutex.lock();
    message.queue_size = this->images.size();
    message.received_images = this->received_images;
    message.processed_images = this->processed_images;
    queue_mutex.unlock();
    this->durations.push_back(this->last_image_proc_duration);

    if (this->durations.size() > 100) {
      this->durations.pop_front();
    }
    double average = 0;
    int num_durations = this->durations.size();
    for (auto duration = this->durations.begin();
         duration != this->durations.end();
         duration++) {
      average = average + (*duration) / ((double) num_durations);
    }
    status_mutex.unlock();

    message.image_duration = average;
    this->status_pub->publish(message);
  }

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto detection_node = make_shared<ArucoDetector>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(detection_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
