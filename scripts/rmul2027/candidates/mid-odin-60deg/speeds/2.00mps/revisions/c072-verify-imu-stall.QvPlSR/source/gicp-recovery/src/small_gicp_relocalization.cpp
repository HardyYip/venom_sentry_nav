// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include "small_gicp_relocalization/registration_budget.hpp"
#include <Eigen/Eigenvalues>
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  recovery_epoch_ = this->declare_parameter<int>("recovery_epoch", 0);
  registration_worker_enabled_ = this->declare_parameter<bool>("registration_worker_enabled", false);
  if (registration_worker_enabled_) {
    RCLCPP_INFO(this->get_logger(), "GICP registration worker enabled");
  }
  registration_max_iterations_ = checkedRegistrationBudget(
    this->declare_parameter<int>("registration_max_iterations", 10));
  max_registration_error_per_inlier_ =
    this->declare_parameter<double>("max_registration_error_per_inlier", 0.);
  if (!std::isfinite(max_registration_error_per_inlier_) || max_registration_error_per_inlier_ < 0.) {
    throw std::invalid_argument("max_registration_error_per_inlier must be finite and nonnegative");
  }
  max_reacquired_pose_jump_m_ = this->declare_parameter<double>("max_reacquired_pose_jump_m", 0.);
  if (!std::isfinite(max_reacquired_pose_jump_m_) || max_reacquired_pose_jump_m_ < 0.) {
    throw std::invalid_argument("max_reacquired_pose_jump_m must be finite and nonnegative");
  }
  if (max_registration_error_per_inlier_ > 0. || max_reacquired_pose_jump_m_ > 0.) {
    recovery_permit_sub_ = this->create_subscription<example_interfaces::msg::Bool>(
      "recovery/permit", 1,
      [this](example_interfaces::msg::Bool::ConstSharedPtr permit) {
        // Initial verification also sends false. Retire only an instance
        // that actually observed its running permit before revocation.
        if (!permit->data && observed_running_permit_) {
          publisher_retired_ = true;
        }
        observed_running_permit_ |= permit->data;
        recovery_permitted_ = permit->data;
      });
  }
  recovery_correspondence_distance_m_ =
    this->declare_parameter<double>("recovery_correspondence_distance_m", 0.0);
  if (!std::isfinite(recovery_correspondence_distance_m_) ||
    recovery_correspondence_distance_m_ < 0.0)
  {
    throw std::invalid_argument("Recovery correspondence distance must be finite and nonnegative");
  }
  tracking_correspondence_distance_m_ =
    this->declare_parameter<double>("tracking_correspondence_distance_m", 0.0);
  if (!std::isfinite(tracking_correspondence_distance_m_) ||
    tracking_correspondence_distance_m_ < 0.0)
  {
    throw std::invalid_argument("Tracking correspondence distance must be finite and nonnegative");
  }
  metric_correspondences_ = this->declare_parameter<bool>("metric_correspondences", false);
  deterministic_global_sampling_ = this->declare_parameter<bool>(
    "deterministic_global_sampling", false);
  quality_pub_ = this->create_publisher<std_msgs::msg::String>("recovery/registration", 10);
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  if (metric_correspondences_) {
    metric_register_ = std::make_shared<
      small_gicp::Registration<MetricCorrespondenceFactor, small_gicp::ParallelReductionOMP>>();
  }
  RCLCPP_INFO(this->get_logger(),
    "GICP_METRIC_CORRESPONDENCES enabled=%s neighbor_cap=8 extra_distance_m=0.025",
    metric_correspondences_ ? "true" : "false");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  // Keep the protected legacy sampler and float conversion unless explicitly selected.
  // The CAD prior was already filtered by the map generator. Preserve full voxel
  // centroids and the ROS double parameter precision for the recovery-only sampler.
  if (deterministic_global_sampling_) {
    const double leaf = this->get_parameter("global_leaf_size").as_double();
    if (!std::isfinite(leaf) || leaf <= 0.0) {
      throw std::invalid_argument("deterministic global leaf must be finite and positive");
    }
    target_ = small_gicp::voxelgrid_sampling<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, leaf);
  } else {
    target_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, global_leaf_size_);
  }
  RCLCPP_INFO(this->get_logger(),
    "GICP_GLOBAL_SAMPLING deterministic=%s leaf=%.17g points=%zu",
    deterministic_global_sampling_ ? "true" : "false",
    deterministic_global_sampling_ ?
      this->get_parameter("global_leaf_size").as_double() : double(global_leaf_size_),
    target_->size());

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

SmallGicpRelocalizationNode::~SmallGicpRelocalizationNode()
{
  if (registration_job_.valid()) {
    registration_job_.wait();
  }
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  // The recovery supervisor needs the odometry scan frame to send the seed.
  // Keep the frame available while withholding unseeded registration input.
  if (!scanFollowsRecoverySeed()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

bool SmallGicpRelocalizationNode::scanFollowsRecoverySeed() const
{
  if (publisher_retired_) {
    return false;
  }
  return recovery_epoch_ == 0 ||
         (recovery_seed_stamp_ > 0. && last_scan_time_.seconds() > recovery_seed_stamp_);
}

void SmallGicpRelocalizationNode::performRegistration()
{
  collectRegistration();
  if (registration_job_.valid()) {
    // If the worker crosses another registration tick, keep collecting a
    // fresh batch for the next solve without growing the backlog.
    accumulated_cloud_->clear();
    return;
  }
  if (!scanFollowsRecoverySeed()) {
    return;
  }
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  const auto scan_time = last_scan_time_;
  // During stopped recovery the acknowledged seed is the trusted anchor.
  // Restrict matches to its nearby static geometry so newly seen obstacles
  // cannot pull the solver into a distant alignment.
  const bool aligning_from_seed = recovery_correspondence_distance_m_ > 0.0 &&
    recovery_epoch_ > 0 && recovery_seed_stamp_ > 0.0 &&
    !observed_running_permit_ && !recovery_permitted_;
  const auto initial = aligning_from_seed ? recovery_seed_transform_ : previous_result_t_;
  // Once localized, track against nearby map structure instead of matching
  // returns across an obstacle or wall that is not present in the prior map.
  const double correspondence_distance = aligning_from_seed ? recovery_correspondence_distance_m_ :
    (observed_running_permit_ && recovery_permitted_ ? tracking_correspondence_distance_m_ : 0.0);
  const double correspondence_limit_sq = correspondence_distance > 0.0 ?
    std::min(static_cast<double>(max_dist_sq_), correspondence_distance * correspondence_distance) :
    max_dist_sq_;
  if (!registration_worker_enabled_) {
    const auto measurement = calculateRegistration(*accumulated_cloud_, initial, correspondence_limit_sq);
    applyRegistration(measurement, scan_time);
    accumulated_cloud_->clear();
    return;
  }
  // A single immutable batch goes to the worker. The executor continues to
  // receive subsequent original scans and publish the last accepted correction.
  auto next_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  const auto batch = accumulated_cloud_;
  registration_job_ = std::async(std::launch::async, [this, batch, initial, correspondence_limit_sq]() {
    return calculateRegistration(*batch, initial, correspondence_limit_sq);
  });
  accumulated_cloud_ = std::move(next_cloud);
  pending_scan_time_ = scan_time;
  pending_seed_generation_ = seed_generation_;
}

SmallGicpRelocalizationNode::Measurement SmallGicpRelocalizationNode::calculateRegistration(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const Eigen::Isometry3d & initial,
  double correspondence_limit_sq)
{
  auto source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    cloud, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source, num_neighbors_, num_threads_);

  auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source || !source_tree) {
    throw std::runtime_error("No GICP source tree");
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = correspondence_limit_sq;
  register_->optimizer.max_iterations = registration_max_iterations_;
  if (metric_register_) {
    metric_register_->reduction.num_threads = num_threads_;
    metric_register_->rejector.max_dist_sq = correspondence_limit_sq;
    metric_register_->optimizer.max_iterations = registration_max_iterations_;
  }

  const auto registration_begin = std::chrono::steady_clock::now();
  auto result = metric_register_ ?
    metric_register_->align(*target_, *source, *target_tree_, initial) :
    register_->align(*target_, *source, *target_tree_, initial);
  const double registration_wall_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - registration_begin).count();

  return {std::move(result), registration_wall_s, source->size(), correspondence_limit_sq};
}

void SmallGicpRelocalizationNode::collectRegistration()
{
  if (!registration_job_.valid() ||
    registration_job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return;
  }
  try {
    const auto measurement = registration_job_.get();
    if (publisher_retired_) {
      return;
    }
    if (pending_seed_generation_ != seed_generation_) {
      RCLCPP_INFO(get_logger(), "Discarded registration calculated before the latest seed");
      return;
    }
    applyRegistration(measurement, pending_scan_time_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "GICP worker failed to calculate a correction: %s", error.what());
  }
}

void SmallGicpRelocalizationNode::applyRegistration(
  const Measurement & measurement, const rclcpp::Time & scan_time)
{
  const auto & result = measurement.result;
  // An actual alignment result; the seed and TF timer cannot certify recovery.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(result.H);
  const auto values = eigen.eigenvalues();
  const bool finite = result.T_target_source.matrix().allFinite() && result.H.allFinite() &&
    std::isfinite(result.error) && values.allFinite() && eigen.info() == Eigen::Success;
  const double ratio = finite && values(5) > 0 ? std::max(0.0, values(0) / values(5)) : 0.0;
  const double error_per_inlier = finite && result.num_inliers != 0 ?
    result.error / result.num_inliers : -1.;
  // During a controlled stop the supervisor qualifies the new seed using
  // stationarity, pose consistency and multiple actual registrations.
  const bool enforce_residual_limit =
    max_registration_error_per_inlier_ > 0. && recovery_permitted_;
  const bool residual_accepted = !enforce_residual_limit ||
    (error_per_inlier >= 0. && error_per_inlier <= max_registration_error_per_inlier_);
  // Compare map corrections at the same measured base position so chassis
  // motion between scans does not count as a localization discontinuity.
  // A controlled restart happens at rest with a verified pre-fault seed.
  // Until release, compare with that seed so several updates cannot accumulate drift.
  const bool verify_seed = recovery_epoch_ > 0 && recovery_seed_stamp_ > 0. &&
    !observed_running_permit_ && !recovery_permitted_;
  const bool check_jump = max_reacquired_pose_jump_m_ > 0. &&
    (verify_seed || (observed_running_permit_ && recovery_permitted_));
  const bool reacquisition = check_jump && recent_rejections_ >= 3;
  double jump = -1.;
  if (check_jump && result.converged && finite && residual_accepted) {
    try {
      const auto pose = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_,
        scan_time, rclcpp::Duration::from_seconds(0.));
      const Eigen::Vector3d anchor = tf2::transformToEigen(pose.transform).translation();
      const auto & reference = verify_seed ? recovery_seed_transform_ : previous_result_t_;
      jump = ((result.T_target_source * anchor) - (reference * anchor)).head<2>().norm();
      if (!std::isfinite(jump)) {
        jump = -1.;
      }
    } catch (const tf2::TransformException &) {
      // Keep the accepted correction if the same-scan anchor is unavailable.
    }
  }
  const bool jump_accepted = !check_jump || (std::isfinite(jump) && jump >= 0. &&
    jump <= max_reacquired_pose_jump_m_);
  const bool tracking = observed_running_permit_ && recovery_permitted_ &&
    tracking_correspondence_distance_m_ > 0.0;
  const bool supported = !tracking ||
    (result.num_inliers >= 200 && result.num_inliers >= 0.35 * measurement.source_points &&
    ratio >= 1e-6);
  const bool correction_accepted = result.converged && finite && supported &&
    residual_accepted && jump_accepted;
  std::ostringstream row;
  row << std::setprecision(17) << "{\"epoch\":" << recovery_epoch_
      << ",\"seed_stamp\":" << recovery_seed_stamp_
      << ",\"sequence\":" << ++quality_sequence_
      << ",\"stamp\":" << scan_time.seconds()
      << ",\"converged\":" << (result.converged && finite ? "true" : "false")
      << ",\"pose_correction_accepted\":" << (correction_accepted ? "true" : "false")
      << ",\"runtime_residual_gate\":" << (enforce_residual_limit ? "true" : "false")
      << ",\"recovery_seed_jump_gate\":" << (verify_seed && check_jump ? "true" : "false")
      << ",\"runtime_correction_jump_gate\":" << (check_jump ? "true" : "false")
      << ",\"runtime_correction_jump_m\":" << jump
      << ",\"runtime_reacquisition_gate\":" << (reacquisition ? "true" : "false")
      << ",\"runtime_reacquired_pose_jump_m\":" << (reacquisition ? jump : -1.)
      << ",\"optimizer_iterations\":" << result.iterations + 1
      << ",\"optimizer_max_iterations\":" << registration_max_iterations_
      << ",\"registration_wall_s\":" << measurement.wall_seconds
      << ",\"correspondence_limit_m\":" << std::sqrt(measurement.correspondence_limit_sq)
      << ",\"source_points\":" << measurement.source_points << ",\"inliers\":" << result.num_inliers
      << ",\"error_per_inlier\":" << error_per_inlier
      << ",\"hessian_ratio\":" << ratio << "}";
  std_msgs::msg::String quality; quality.data = row.str(); quality_pub_->publish(quality);

  if (correction_accepted) {
    result_t_ = previous_result_t_ = result.T_target_source;
    recent_rejections_ = 0;
  } else {
    recent_rejections_ = std::min(3, recent_rejections_ + 1);
  }
  if (result.converged && finite && !supported) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Retaining prior GICP correction with inadequate support for nearby matches");
  } else if (result.converged && finite && residual_accepted && !jump_accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Retaining prior correction after GICP update, pose jump %.5f m (limit %.5f)",
      jump, max_reacquired_pose_jump_m_);
  } else if (!correction_accepted && result.converged && finite) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Retaining prior map-to-odom correction after GICP residual %.6f exceeds %.6f",
      error_per_inlier, max_registration_error_per_inlier_);
  } else if (!correction_accepted) {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge to a finite correction.");
  }

}

void SmallGicpRelocalizationNode::publishTransform()
{
  collectRegistration();
  if (result_t_.matrix().isZero() || !scanFollowsRecoverySeed()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (publisher_retired_) {
    return;
  }
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const double source = rclcpp::Time(msg->header.stamp).seconds();
  const double age = this->now().seconds() - source;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (msg->header.frame_id != map_frame_ || !std::isfinite(age) || age < -0.1 || age > 0.5 ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || std::abs(norm-1.0) > 0.01) {
    RCLCPP_WARN(this->get_logger(), "Rejected invalid/stale initial pose");
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    recovery_seed_transform_ = map_to_odom;
    previous_result_t_ = result_t_ = map_to_odom;
    recent_rejections_ = 0;
    recovery_seed_stamp_ = source;
    ++seed_generation_;
    accumulated_cloud_->clear();
    std::ostringstream row;
    row << std::setprecision(17) << "{\"kind\":\"seed_ack\",\"epoch\":" << recovery_epoch_
        << ",\"stamp\":" << source << "}";
    std_msgs::msg::String ack; ack.data = row.str(); quality_pub_->publish(ack);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
