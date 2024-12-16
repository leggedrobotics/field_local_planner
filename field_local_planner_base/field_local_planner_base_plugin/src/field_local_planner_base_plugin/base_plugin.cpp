#include <field_local_planner_base_plugin/base_plugin.hpp>

namespace field_local_planner {

void BasePlugin::initialize(ros::NodeHandle& nh) {
  // Initialie privete node handle
  nh_local_planner_ = ros::NodeHandle("~" + getName());

  // Load basic parameters
  ROS_INFO("Loading base parameters...");
  loadBaseParameters(nh);

  // Load local planner parameters
  ROS_INFO("Loading specific parameters...");
  loadParameters(nh_local_planner_);

  // Setup ROS publishers and subscribers
  ROS_INFO("Setup base ROS...");
  setupBaseRos(nh);

  ROS_INFO("Setup specific ROS...");
  setupRos(nh_local_planner_);

  // Spin
  ROS_INFO("Ready!");
  ros::spin();
}

void BasePlugin::loadBaseParameters(ros::NodeHandle& nh) {
  // Initialize tf listener
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // Read common parameters
  fixed_frame_ = utils::getParameter<std::string>(nh, "fixed_frame");
  valid_goal_frames_ = utils::getParameterVector<std::string>(nh, "valid_goal_frames");

  grid_map_to_cloud_ = utils::getParameter<bool>(nh, "grid_map_to_cloud");
  grid_map_to_cloud_range_ = utils::getParameter<double>(nh, "grid_map_to_cloud_range");
  grid_map_to_cloud_filter_size_ = utils::getParameter<double>(nh, "grid_map_to_cloud_filter_size");

  // Set voxel filter
  voxel_filter_.setLeafSize(grid_map_to_cloud_filter_size_, grid_map_to_cloud_filter_size_, grid_map_to_cloud_filter_size_);

  // If base inverted
  std::string base_frame = utils::getParameter<std::string>(nh, "base_frame");
  base_inverted_ = utils::getParameter<bool>(nh, "base_inverted");

  if (base_inverted_) {
    base_frame_ = base_frame + "_inverted_field_local_planner";
    publishStaticTransform(getBaseInversionTransform(), base_frame, base_frame_);

  } else {
    base_frame_ = base_frame;
  }

  BaseLocalPlanner::Parameters p;
  p.requires_sensing = utils::getParameter<bool>(nh, "requires_sensing");
  p.differential_mode = utils::getParameter<bool>(nh, "differential_mode");
  p.control_rate = utils::getParameter<double>(nh, "control_rate");
  p.robot_length = utils::getParameter<double>(nh, "robot_length");
  p.robot_width = utils::getParameter<double>(nh, "robot_width");
  p.robot_height = utils::getParameter<double>(nh, "robot_height");
  p.distance_to_goal_thr = utils::getParameter<double>(nh, "distance_to_goal_thr");
  p.orientation_to_goal_thr = utils::getParameter<double>(nh, "orientation_to_goal_thr");
  p.max_linear_velocity_x = utils::getParameter<double>(nh, "max_linear_velocity_x");
  p.max_linear_velocity_y = utils::getParameter<double>(nh, "max_linear_velocity_y");
  p.max_angular_velocity_z = utils::getParameter<double>(nh, "max_angular_velocity_z");
  p.progress_threshold = utils::getParameter<double>(nh, "progress_threshold");
  p.failure_timeout_sec = utils::getParameter<double>(nh, "failure_timeout_sec");

  local_planner_->setBaseParameters(p);
}

void BasePlugin::setupBaseRos(ros::NodeHandle& nh) {
  // Setup subscribers
  std::string pose_topic = utils::getParameterDefault(nh, "pose_topic", std::string(""));
  if (!pose_topic.empty()) {
    pose_sub_ = nh.subscribe(pose_topic, 1, &BasePlugin::poseCallback, this);
  }

  std::string twist_topic = utils::getParameterDefault(nh, "twist_topic", std::string(""));
  if (!twist_topic.empty()) {
    twist_sub_ = nh.subscribe(twist_topic, 1, &BasePlugin::twistCallback, this);
  }

  std::string odometry_topic = utils::getParameterDefault(nh, "odometry_topic", std::string(""));
  if (!odometry_topic.empty()) {
    odometry_sub_ = nh.subscribe(odometry_topic, 1, &BasePlugin::odometryCallback, this);
  }

  // Sensing
  std::string point_cloud_topic = utils::getParameterDefault(nh, "point_cloud_topic", std::string(""));
  if (!point_cloud_topic.empty()) {
    point_cloud_sub_ = nh.subscribe(std::string(point_cloud_topic), 1, &BasePlugin::pointCloudCallback, this);
  }

  std::string grid_map_topic = utils::getParameterDefault(nh, "grid_map_topic", std::string(""));
  if (!grid_map_topic.empty()) {
    grid_map_sub_ = nh.subscribe(std::string(grid_map_topic), 1, &BasePlugin::gridMapCallback, this);
  }

  std::string rgb_image_topic = utils::getParameterDefault(nh, "rgb_image_topic", std::string(""));
  if (!rgb_image_topic.empty()) {
    // TODO add synchronized subscriber
    // rgb_image_sub_ =
    //     nh.subscribe(std::string(rgb_image_topic), 1, &BasePlugin::imageRgbCallback, this);
  }

  std::string rgbd_image_topic = utils::getParameterDefault(nh, "rgbd_image_topic", std::string(""));
  if (!rgbd_image_topic.empty()) {
    // TODO add synchronized subscriber
    // rgbd_image_sub_ =
    //     nh.subscribe(std::string(rgbd_image_topic), 1, &BasePlugin::imageRgbdCallback, this);
  }

  std::string depth_image_topic = utils::getParameterDefault(nh, "depth_image_topic", std::string(""));
  if (!depth_image_topic.empty()) {
    // TODO add synchronized subscriber
    // depth_image_sub_ =
    //     nh.subscribe(std::string(depth_image_topic), 1, &BasePlugin::imageDepthCallback, this);
  }

  // Goal
  std::string goal_topic = utils::getParameterDefault(nh, "goal_topic", std::string("/goal"));
  goal_sub_ = nh.subscribe(std::string(goal_topic), 1, &BasePlugin::goalCallback, this);

  // Joystick command
  std::string joy_twist_topic = utils::getParameterDefault(nh, "joy_twist_topic", std::string("/cmd_twist"));
  joy_twist_sub_ = nh.subscribe(std::string(joy_twist_topic), 1, &BasePlugin::joyTwistCallback, this);

  // Setup publishers
  // Status
  status_pub_ = nh.advertise<field_local_planner_msgs::Status>("/field_local_planner/status", 10);
  // Path
  path_pub_ = nh.advertise<nav_msgs::Path>("/field_local_planner/path", 10);
  // Current goal
  current_goal_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(ros::this_node::getName() + "/current_goal", 10);
  // Current goal
  current_base_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(ros::this_node::getName() + "/current_base", 10);

  // Output twist type type
  output_twist_type_ = utils::getParameterDefault(nh, "output_twist_type", std::string("twist"));
  std::string output_twist_topic = utils::getParameterDefault(nh, "output_twist_topic", std::string("/field_local_planner/twist"));
  if (output_twist_type_ == "twist") {
    output_twist_pub_ = nh.advertise<geometry_msgs::Twist>(output_twist_topic, 10);

  } else if (output_twist_type_ == "twist_stamped") {
    output_twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic, 10);

  } else {
    ROS_ERROR_STREAM("Invalid twist type [" << output_twist_type_ << "], valid options are 'twist' or 'twist_stamped'");
    exit(1);
  }

  // Setup services

  // Setup action server
  action_server_ = std::make_shared<ActionServer>(nh, "action_server", false);
  action_server_->registerGoalCallback(boost::bind(&BasePlugin::moveToRequestActionHandler, this));
  action_server_->registerPreemptCallback(boost::bind(&BasePlugin::preemptActionHandler, this));
  action_server_->start();

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&BasePlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

bool BasePlugin::execute(const ros::Time& stamp, geometry_msgs::Twist& twist_msg, nav_msgs::Path& path_msg,
                         field_local_planner_msgs::Status& status_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(stamp);

  // Compute output
  BaseLocalPlanner::Output output;
  bool valid_output = local_planner_->execute(ts, output);

  // Print useful info to terminal
  printStateInfo(output.status.state);

  // Publish custom visualizations of the local planner
  publishVisualizations();

  // Publish common stuff
  publishStatus(output.status);
  if (valid_output) {
    publishTwist(output.twist);
    publishPath(output.path);
    publishCurrentGoal(T_f_g_);
  }

  return valid_output;
}

void BasePlugin::setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header) {
  // Convert stamp
  Time ts = utils::toTimeStamp(header.stamp);

  // Get transformation if frames do not match: frame_id (auxiliary) in fixed_frame f
  Pose3 T_f_a = queryTransform(fixed_frame_, header.frame_id, header.stamp);

  // Transform message
  Pose3 T_a_b = utils::toPose3(pose_msg);                       // Transformation of base b in auxiliary frame
  Pose3 T_f_b = fixBaseInversion(T_f_a * T_a_b, fixed_frame_);  // Base in fixed frame

  // Set data
  local_planner_->setPoseInFixed(T_f_b, ts);

  // Publish
  publishCurrentBase(T_f_b);
}

void BasePlugin::setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header) {
  // Convert stamp
  Time ts = utils::toTimeStamp(header.stamp);

  Twist twist = utils::toTwist(twist_msg);
  local_planner_->setVelocityInBase(twist, ts);
}

void BasePlugin::setGoal(const geometry_msgs::Pose& goal_msg, const std_msgs::Header& header) {
  // Convert stamp
  Time ts = utils::toTimeStamp(header.stamp);

  // Check if goal is defined in a valid frame
  if (!isValidFrame(header.frame_id)) {
    ROS_ERROR_STREAM("setGoal: Frame [" << header.frame_id << "] is not valid.");
    return;
  }

  // Get transformation if frames do not match
  // frame_id (auxiliary) in fixed_frame f
  Pose3 T_f_a = queryTransform(fixed_frame_, header.frame_id, header.stamp);

  // Transform message
  Pose3 T_a_g = utils::toPose3(goal_msg);  // Transformation of base b in auxiliary frame
  T_f_g_ = T_f_a * T_a_g;                  // Base in fixed frame

  // Publish goal pose as message and TF
  publishCurrentGoal(T_f_g_);

  // Query base in fixed frame
  Pose3 T_f_b = queryTransform(fixed_frame_, base_frame_, header.stamp);

  // Set data
  ROS_INFO_STREAM("Setting new goal in frame [" << fixed_frame_ << "] : \n  position: " << T_f_g_.translation().transpose()
                                                << "\n  orientation (Euler): " << T_f_g_.rotation().rpy().transpose());
  local_planner_->setGoalInFixed(T_f_g_, T_f_b, ts);
}

void BasePlugin::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
  setPose(pose_msg->pose.pose, pose_msg->header);

  // Execute local planner
  geometry_msgs::Twist twist;
  nav_msgs::Path path;
  field_local_planner_msgs::Status status;
  execute(pose_msg->header.stamp, twist, path, status);
}

void BasePlugin::twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg) {
  setVelocity(twist_msg->twist.twist, twist_msg->header);
}

void BasePlugin::odometryCallback(const nav_msgs::OdometryConstPtr& odo_msg) {
  setPose(odo_msg->pose.pose, odo_msg->header);
  setVelocity(odo_msg->twist.twist, odo_msg->header);
}

// Goal callback
void BasePlugin::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg) {
  setGoal(goal_msg->pose, goal_msg->header);
}

// Joystick callback
void BasePlugin::joyTwistCallback(const geometry_msgs::TwistConstPtr& twist_msg) {
  ROS_FATAL("BasePlugin::joyTwistCallback: Not implemented");
}

// Action server
void BasePlugin::moveToRequestActionHandler() {
  ROS_INFO_STREAM("[BasePlugin] Action Server - New goal");

  // Get goal from server
  geometry_msgs::PoseWithCovarianceStamped goal_msg = action_server_->acceptNewGoal()->goal;

  // Pass goal to the local planner
  setGoal(goal_msg.pose.pose, goal_msg.header);
}

void BasePlugin::preemptActionHandler() {
  ROS_INFO_STREAM("[BasePlugin] Action Server - Stop");
  action_server_->setPreempted();
  local_planner_->stop();
  publishZeroTwist();
}

void BasePlugin::dynamicReconfigureCallback(BaseConfig& config, uint32_t level) {
  BaseLocalPlanner::Parameters p = local_planner_->getBaseParameters();

  // Trackline parameters
  UPDATE_COMMON_PARAMS(differential_mode)
  UPDATE_COMMON_PARAMS(robot_length)
  UPDATE_COMMON_PARAMS(robot_width)
  UPDATE_COMMON_PARAMS(robot_height)
  UPDATE_COMMON_PARAMS(distance_to_goal_thr)
  UPDATE_COMMON_PARAMS(orientation_to_goal_thr)
  UPDATE_COMMON_PARAMS(max_linear_velocity_x)
  UPDATE_COMMON_PARAMS(max_linear_velocity_y)
  UPDATE_COMMON_PARAMS(max_angular_velocity_z)
  UPDATE_COMMON_PARAMS(progress_threshold)
  UPDATE_COMMON_PARAMS(failure_timeout_sec)

  local_planner_->setBaseParameters(p);
}

void BasePlugin::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(cloud_msg->header.stamp);
  // local_planner_->setPointCloud(cloud, T_b_s);
  ROS_FATAL("BasePlugin::pointCloudCallback: Not implemented");
}

void BasePlugin::gridMapCallback(const grid_map_msgs::GridMap& grid_map_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(grid_map_msg.info.header.stamp);

  // Convert grid map
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map);
  grid_map.convertToDefaultStartIndex();

  // Query transformation from grid map frame
  Pose3 T_f_m = queryTransform(fixed_frame_, grid_map.getFrameId(), grid_map_msg.info.header.stamp);

  // Set grid map in local planner
  local_planner_->setGridMap(grid_map, T_f_m, ts);

  if (grid_map_to_cloud_) {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    Pose3 T_f_s;
    getPointCloudFromGridMap(grid_map, grid_map_msg.info.header.stamp, cloud, T_f_s);

    local_planner_->setPointCloud(cloud, T_f_s, ts);
  }
}

// Sensor callbacks
void BasePlugin::imageRgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Time ts = utils::toTimeStamp(info_msg->header.stamp);
  // local_planner_->setImageRgb(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageRgb: Not implemented");
}

void BasePlugin::imageRgbdCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Time ts = utils::toTimeStamp(info_msg->header.stamp);
  // local_planner_->setImageRgbd(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageRgbd: Not implemented");
}

void BasePlugin::imageDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Time ts = utils::toTimeStamp(info_msg->header.stamp);
  // local_planner_->setImageDepth(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageDepth: Not implemented");
}

void BasePlugin::publishTransform(const Pose3& T_parent_child, const std::string& parent, const std::string& child,
                                  const ros::Time& stamp) {
  geometry_msgs::TransformStamped transform_parent_child = utils::toTransformStamped(T_parent_child, parent, child, stamp);
  tf_broadcaster_.sendTransform(transform_parent_child);
}

void BasePlugin::publishStaticTransform(const Pose3& T_parent_child, const std::string& parent, const std::string& child,
                                        const ros::Time& stamp) {
  geometry_msgs::TransformStamped transform_parent_child = utils::toTransformStamped(T_parent_child, parent, child, stamp);
  tf_static_broadcaster_.sendTransform(transform_parent_child);
}

void BasePlugin::publishCurrentGoal(const Pose3& T_fixed_goal, const ros::Time& stamp) {
  // Publish msg
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.pose.pose = utils::toPoseMsg(T_fixed_goal);
  pose_msg.header.frame_id = fixed_frame_;
  pose_msg.header.stamp = stamp;
  current_goal_pub_.publish(pose_msg);

  // Publish TF
  publishTransform(T_fixed_goal, fixed_frame_, "goal_field_local_planner");
}

void BasePlugin::publishCurrentBase(const Pose3& T_fixed_base, const ros::Time& stamp) {
  // Publish msg
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.pose.pose = utils::toPoseMsg(T_fixed_base);
  pose_msg.header.frame_id = fixed_frame_;
  pose_msg.header.stamp = stamp;
  current_base_pub_.publish(pose_msg);

  // Publish TF
  // publishTransform(T_fixed_base, fixed_frame_, "base_local_planner");
}

void BasePlugin::publishPath(const Path& path) {
  nav_msgs::Path path_msg = utils::toPathMsg(path, base_frame_);
  path_pub_.publish(path_msg);
}

void BasePlugin::publishStatus(const BaseLocalPlanner::Status& status) {
  field_local_planner_msgs::Status status_msg = utils::toStatusMsg(status);
  status_pub_.publish(status_msg);

  if (action_server_->isActive()) {
    feedback_.status = status_msg;
    action_server_->publishFeedback(feedback_);
  }
}

void BasePlugin::publishTwist(const Twist& twist, const ros::Time& stamp) {
  Twist corrected_twist = fixBaseInversion(twist);
  geometry_msgs::Twist twist_msg = utils::toTwistMsg(corrected_twist);

  if (output_twist_type_ == "twist") {
    output_twist_pub_.publish(twist_msg);

  } else if (output_twist_type_ == "twist_stamped") {
    geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.twist = twist_msg;
    twist_stamped.header.seq = 0;
    twist_stamped.header.stamp = stamp;  // Check if this is correct
    twist_stamped.header.frame_id = base_frame_;
    output_twist_pub_.publish(twist_stamped);
  }
}

void BasePlugin::publishZeroTwist() {
  publishTwist(Twist::Zero());
}

Pose3 BasePlugin::queryTransform(const std::string& parent, const std::string& child, const ros::Time& stamp) {
  Eigen::Isometry3d eigen_T_parent_child = Eigen::Isometry3d::Identity();
  try {
    geometry_msgs::TransformStamped T_parent_child = tf_buffer_.lookupTransform(parent, child, stamp, ros::Duration(1.0));
    eigen_T_parent_child = tf2::transformToEigen(T_parent_child);

  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  // Return in Pose3 format
  return Pose3(eigen_T_parent_child.matrix());
}

bool BasePlugin::isValidFrame(const std::string& frame) const {
  return std::find(valid_goal_frames_.begin(), valid_goal_frames_.end(), frame) != valid_goal_frames_.end();
}

void BasePlugin::getPointCloudFromGridMap(const grid_map::GridMap& grid_map, const ros::Time& stamp, pcl::PointCloud<PointType>::Ptr& cloud,
                                          Pose3& T_f_s) {
  grid_map::GridMap cropped_grid_map;
  grid_map::Position position(grid_map.getPosition().x(), grid_map.getPosition().y());
  grid_map::Length length(2.0 * grid_map_to_cloud_range_, 2.0 * grid_map_to_cloud_range_);

  bool success;
  cropped_grid_map = grid_map.getSubmap(position, length, success);

  // Convert to point_cloud
  pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>());

  sensor_msgs::PointCloud2 point_cloud;
  grid_map::GridMapRosConverter::toPointCloud(cropped_grid_map, {"elevation", "intensity"}, "elevation", point_cloud);
  pcl::fromROSMsg(point_cloud, *tmp_cloud);

  // Filter using voxel_filter
  voxel_filter_.setInputCloud(tmp_cloud);
  voxel_filter_.filter(*tmp_cloud);

  // Transform to base frame
  Pose3 T_b_m = queryTransform(base_frame_, grid_map.getFrameId(), stamp);
  pcl::transformPointCloud(*tmp_cloud, *cloud, T_b_m.matrix());
  T_f_s = queryTransform(fixed_frame_, base_frame_, stamp);
}

void BasePlugin::printStateInfo(const BaseLocalPlanner::State& new_state) {
  if (new_state != last_state_) {
    if (new_state == BaseLocalPlanner::State::NOT_READY) {
      ROS_INFO("Change to state: NOT_READY (%d)", new_state);

    } else if (new_state == BaseLocalPlanner::State::FINISHED) {
      ROS_INFO("Change to state: FINISHED (%d)", new_state);

    } else if (new_state == BaseLocalPlanner::State::EXECUTING) {
      ROS_INFO("Change to state: EXECUTING (%d)", new_state);

    } else if (new_state == BaseLocalPlanner::State::FAILURE) {
      ROS_WARN("Change to state: FAILURE (%d)", new_state);
    }
  }

  last_state_ = new_state;
}

Pose3 BasePlugin::getBaseInversionTransform() const {
  return Pose3(Rot3::Yaw(M_PI), Vector3::Zero());
}

Pose3 BasePlugin::fixBaseInversion(const Pose3& T, const std::string& frame) const {
  if (base_inverted_) {
    if (frame == base_frame_) {
      return getBaseInversionTransform() * T;

    } else if (frame == fixed_frame_) {
      return T * getBaseInversionTransform();

    } else {
      return T;
    }
  } else {
    return T;
  }
}

Twist BasePlugin::fixBaseInversion(const Twist& b_v) const {
  if (base_inverted_) {
    Twist corrected_twist = b_v;
    corrected_twist(3) *= base_inverted_ ? -1.0 : 1.0;
    corrected_twist(4) *= base_inverted_ ? -1.0 : 1.0;
    return corrected_twist;

  } else {
    return b_v;
  }
}

}  // namespace field_local_planner
