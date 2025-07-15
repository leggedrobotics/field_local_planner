//----------------------------------------
// This file is part of field_local_planner
//
// Copyright (C) 2020-2025 Matías Mattamala, University of Oxford.
//
// field_local_planner is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// field_local_planner is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with field_local_planner.
// If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------
#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <field_local_planner_falco/falco_local_planner.hpp>

namespace field_local_planner {

Falco::Falco() : BaseLocalPlanner() {}

void Falco::setParameters(const Falco::Parameters& p) {
  if (p.config_folder != parameters_.config_folder) {
    loadPathsFromFile(p.config_folder);
  }
  parameters_ = p;

  // Precompute robot diameters
  robot_diameter_ = std::sqrt(std::pow(base_parameters_.robot_length, 2) + std::pow(base_parameters_.robot_width, 2));
}

Falco::Parameters Falco::getParameters() const {
  return parameters_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Falco::getFreePaths() const {
  return free_paths_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Falco::getCollisionMap() const {
  return point_cloud_;
}

Twist Falco::computeTwist() {
  // check collisions and compute free path
  computeFreePath();

  // Select carrot from path
  computeIntermediateCarrot();

  // Compute command from carrot
  double heading_to_carrot = atan2(intermediate_carrot_(1), intermediate_carrot_(0));
  double distance_to_carrot = hypot(intermediate_carrot_(1), intermediate_carrot_(0));

  Twist twist = Twist::Zero();
  if (parameters_.look_ahead_distance < distance_to_goal_) {
    // Track the carrot
    // Angular
    twist(2) = utils::clipValue(heading_to_carrot * parameters_.angular_gain_p, base_parameters_.max_linear_velocity_x);
    // Linear X
    twist(3) = cos(heading_to_carrot) * base_parameters_.max_linear_velocity_x;
    // Linear Y
    twist(4) = sin(heading_to_carrot) * base_parameters_.max_linear_velocity_x;
  } else {
    // Get close to goal
    // Angular
    twist(2) = utils::clipValue(orientation_to_goal_ * parameters_.angular_gain_p, base_parameters_.max_linear_velocity_x);
    // Linear X
    twist(3) = cos(heading_towards_goal_) * base_parameters_.max_linear_velocity_x * distance_to_goal_;
    // Linear Y
    twist(4) = sin(heading_towards_goal_) * base_parameters_.max_linear_velocity_x * distance_to_goal_;
  }

  if (base_parameters_.differential_mode) {
    // Remove twist in Y direction
    twist(4) = 0.0;
  }

  return twist;
}

Path Falco::computePath() {
  Path path;
  double distance = 0.0;
  Vector3 last_added_point(0.0, 0.0, 0.0);

  for (auto point : best_path_) {
    double distance = (point - last_added_point).norm();
    if (distance > 0.1) {
      path.push_back(Pose3(Rot3(), last_added_point));
      last_added_point = point;
    }
  }

  return path;
}

//-------------------------------------------------------------------------------------------------
// Check precomputed paths collisions
//-------------------------------------------------------------------------------------------------
void Falco::computeFreePath() {
  // Path range
  path_range_ = parameters_.sensor_range;
  path_range_ = std::max(path_range_, parameters_.min_path_range);

  // Path scale
  // Store the path scale temporarily
  path_scale_ = parameters_.path_scale;
  path_scale_ = std::max(path_scale_, parameters_.min_path_scale);

  // Main iteration to check the paths
  while (path_scale_ >= parameters_.min_path_scale && path_range_ >= parameters_.min_path_range) {
    // std::cout << "path_scale: " << path_scale_ << ", path_range_: " << path_range_);
    clearPathsLists();
    computeMapCollisions();
    computePathScores();

    feasible_path_found_ = findBestPath();
    // std::cout << "[Falco] found best path: " << feasible_path_found_);
    if (feasible_path_found_) {
      break;
    }
  }
}

void Falco::clearPathsLists() {
  // ROS_INFO("[Falco] clear path lists");
  // Clear path list
  for (int i = 0; i < 36 * NUM_PATHS; i++) {
    clear_path_list_[i] = 0;
    // path_penalty_list_[i] = 0.0;
    path_deformation_list_[i] = 0.0;
    path_traversability_score_[i] = 0.0;
    path_traversability_crossed_voxels_[i] = 0;
  }

  // clear score of group
  for (int i = 0; i < 36 * NUM_GROUPS; i++) {
    path_group_likelihood_[i] = 0.0;
  }
}

void Falco::computeMapCollisions() {
  // Iterate over all the cloud points to find collisions
  int cloud_size = point_cloud_->points.size();
  for (int i = 0; i < point_cloud_->size(); i++) {
    // Precompute all the properties about the voxel that are reused later
    const float x = (point_cloud_->points[i].x - parameters_.robot_clearance) / path_scale_;  // normalized by path_scale
    const float y = (point_cloud_->points[i].y - parameters_.robot_clearance) / path_scale_;  // normalized by path_scale

    // Get traversability
    // Defined in the intensity channel
    const float traversability = std::min(std::max(point_cloud_->points[i].intensity, 0.f), 1.f);

    // Check the score
    bool is_traversable = traversability > parameters_.traversability_thr;

    // Compute distance and angle to obstacle
    float distance_to_obstacle = std::sqrt(x * x + y * y);
    float angle_to_obstacle = std::atan2(y, x);

    // If the obstacle is within the path ranges, check collisions
    if (distance_to_obstacle < path_range_ / path_scale_ &&
        (distance_to_obstacle <= ((distance_to_goal_ + parameters_.goal_clearance) / path_scale_) || !parameters_.use_path_crop_by_goal)) {
      // Iterate over all the group paths to check if the obstacle is in their way
      for (int rot_dir = 0; rot_dir < 36; rot_dir++) {
        // get angle between [-pi, pi]
        float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

        // Rotate the point to express it in the path group frame
        float x2 = cos(rot_ang) * x + sin(rot_ang) * y;
        float y2 = -sin(rot_ang) * x + cos(rot_ang) * y;

        // Get indices in path collision grid (correspondences)
        float scale_y =
            (x2 / GRID_VOXEL_OFFSET_X) + (SEARCH_RADIUS / GRID_VOXEL_OFFSET_Y) * (GRID_VOXEL_OFFSET_X - x2) / GRID_VOXEL_OFFSET_X;

        int idx_x = int((GRID_VOXEL_OFFSET_X + GRID_VOXEL_SIZE / 2 - x2) / GRID_VOXEL_SIZE);
        int idx_y = int((GRID_VOXEL_OFFSET_Y + GRID_VOXEL_SIZE / 2 - y2 / scale_y) / GRID_VOXEL_SIZE);

        // If the indices fall into the correspondences grid
        if (idx_x >= 0 && idx_x < NUM_GRID_VOXELS_X && idx_y >= 0 && idx_y < NUM_GRID_VOXELS_Y) {
          // Compute index to find correspondences
          int idx = NUM_GRID_VOXELS_Y * idx_x + idx_y;

          // Iterate all the paths that pass through the obstacle voxel
          for (int j = 0; j < correspondences_[idx].size(); j++) {
            // Check traversability
            if (!is_traversable) {
              // Update the number of actual collisions with the path
              clear_path_list_[NUM_PATHS * rot_dir + correspondences_[idx][j]]++;
            }
            // Update path score
            path_traversability_score_[NUM_PATHS * rot_dir + correspondences_[idx][j]] += traversability;
            path_traversability_crossed_voxels_[NUM_PATHS * rot_dir + correspondences_[idx][j]]++;
            // Original previous criterion used the largest height to compute a "penalty"
            // TODO: check if the previous approach works
          }
        }
      }
    }

    // Check if the point collides with the robot diameter
    if (parameters_.check_rotational_collisions && !is_traversable) {
      if ((distance_to_obstacle < robot_diameter_ / path_scale_) &&
          ((fabs(x) < base_parameters_.robot_length / path_scale_ / 2.0) || (fabs(y) < base_parameters_.robot_width / path_scale_ / 2.0))) {
        // TODO
      }
    }  // Check rotational collisions
  }    // Map points iteration
}

void Falco::computePathScores() {
  // Compute the scores for each path
  for (int i = 0; i < 36 * NUM_PATHS; i++) {
    int rot_dir = int(i / NUM_PATHS);
    float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);
    float ang_diff = utils::mod2pi(heading_towards_goal_ - rot_ang);

    // select the paths that have less than a fixed number of collisions detected
    if (clear_path_list_[i] <= parameters_.point_per_path_thr) {
      // Compute the main scores/probabilities to select the path

      // Goal probability
      // A Von mises distribution centered in the direction of the goal
      float goal_probability = utils::vonMises(end_dir_path_list_[i % NUM_PATHS] + rot_ang, heading_towards_goal_, parameters_.goal_cost);

      // Traversability probability
      // Average probability over the traversability of all the voxels crossed by the path
      float traversability_probability =
          exp(parameters_.traversability_cost * path_traversability_score_[i] / path_traversability_crossed_voxels_[i]);

      // Differential drive probability
      // If the vehicle is a differential drive, the most probable paths should be in front or the back of the robot
      // Modeled as a mixture of Von Mises
      float differential_mode_probability = 1.0;
      // if(base_parameters_.differential_mode){
      //   differential_mode_probability = 0.5*(utils::vonMises(rot_ang, 0, parameters_.differential_mode_cost) + utils::vonMises(rot_ang,
      //   M_PI, parameters_.differential_mode_cost));
      // }

      // Compute score as the likelihood of all the previous factors
      float score_likelihood = goal_probability * traversability_probability * differential_mode_probability;

      // Compute group index
      int group = NUM_GROUPS * rot_dir + path_list_[i % NUM_PATHS];

      // Store score per path
      path_likelihood_[i] = score_likelihood;

      // Add the score of the path to the score of the group
      if (score_likelihood > 0) {
        path_group_likelihood_[NUM_GROUPS * rot_dir + path_list_[i % NUM_PATHS]] += score_likelihood;
      }
    }
  }
}

bool Falco::findBestPath() {
  // ROS_INFO("[Falco] find best path");
  // Select the group with the maximum score
  // This is a simple linear search
  best_path_group_score_ = 0;
  float min_score = std::numeric_limits<float>::infinity();
  best_path_group_id_ = -1;
  for (int i = 0; i < 36 * NUM_GROUPS; i++) {
    // int rot_dir = int(i / NUM_GROUPS);
    // float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

    // select path with maximum score that doesn't collide with obstacles
    if (best_path_group_score_ < path_group_likelihood_[i]) {
      best_path_group_score_ = path_group_likelihood_[i];
      best_path_group_id_ = i;
    }
    if (min_score > path_group_likelihood_[i]) {
      min_score = path_group_likelihood_[i];
    }
  }

  // Prepare path structure
  best_path_.clear();
  if (best_path_group_id_ >= 0) {
    int rot_dir = int(best_path_group_id_ / NUM_GROUPS);
    float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

    best_path_group_id_ = best_path_group_id_ % NUM_GROUPS;
    int selected_path_length = precomputed_paths_[best_path_group_id_]->points.size();
    best_path_.resize(selected_path_length);

    for (int i = 0; i < selected_path_length; i++) {
      float x = precomputed_paths_[best_path_group_id_]->points[i].x;
      float y = precomputed_paths_[best_path_group_id_]->points[i].y;
      float z = precomputed_paths_[best_path_group_id_]->points[i].z;
      float dis = sqrt(x * x + y * y);

      if (dis <= path_range_ / path_scale_ && dis <= distance_to_goal_ / path_scale_) {
        best_path_[i] = Vector3(path_scale_ * (cos(rot_ang) * x - sin(rot_ang) * y), path_scale_ * (sin(rot_ang) * x + cos(rot_ang) * y),
                                path_scale_ * z);
      } else {
        best_path_.resize(i);
        break;
      }
    }

    // Fill free paths (for visualization)
    free_paths_->clear();
    for (int i = 0; i < 36 * NUM_PATHS; i++) {
      int rot_dir = int(i / NUM_PATHS);
      // float rot_ang = (10.0 * rot_dir - 180.0) * M_PI / 180;
      float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

      pcl::PointXYZI point;
      if (clear_path_list_[i] < parameters_.point_per_path_thr) {
        int free_path_length = paths_[i % NUM_PATHS]->points.size();

        for (int j = 0; j < free_path_length; j++) {
          point = paths_[i % NUM_PATHS]->points[j];

          float path_deformation = path_deformation_list_[i];

          float x = point.x;
          float y = point.y;
          float z = point.z;

          point.x = path_scale_ * (cos(rot_ang) * x - sin(rot_ang) * y);
          point.y = path_scale_ * (sin(rot_ang) * x + cos(rot_ang) * y);
          point.z = path_scale_ * z + path_deformation;
          // point.z = path_group_likelihood_[NUM_GROUPS * rot_dir + path_list_[i % NUM_PATHS]];
          point.intensity =
              path_group_likelihood_[NUM_GROUPS * rot_dir +
                                     path_list_[i % NUM_PATHS]];  // path_likelihood_[i]; //clear_path_list_[i]; //path_likelihood_[i];
          // point.intensity = path_likelihood_[i];

          float dis = sqrt(x * x + y * y);
          if (dis <= path_range_ / path_scale_ &&
              (dis <= (distance_to_goal_ + parameters_.goal_clearance) / path_scale_ || !parameters_.use_path_crop_by_goal)) {
            free_paths_->push_back(point);
          }
        }
      }
    }
  }

  if (best_path_group_id_ < 0) {
    if (path_scale_ >= parameters_.min_path_scale + parameters_.path_scale_step) {
      path_scale_ -= parameters_.path_scale_step;
      path_range_ = parameters_.sensor_range * path_scale_ / parameters_.path_scale;
    } else {
      path_range_ -= parameters_.path_range_step;
    }
  } else {
    return true;
  }

  return false;
}

//-------------------------------------------------------------------------------------------------
// Compute intermediate carrot
//-------------------------------------------------------------------------------------------------
void Falco::computeIntermediateCarrot() {
  if (best_path_.empty()) {
    std::cout << "Best path is empty" << std::endl;
    return;
  }

  // Search carrot in path that is farther than look_ahead_distance
  intermediate_carrot_ = Vector3::Zero();
  for (auto point : best_path_) {
    double distance = point.norm();
    if (distance > parameters_.look_ahead_distance) {
      intermediate_carrot_ = point;
      break;
    }
  }

  // Get end of path
  path_end_ = best_path_.back();
}

//-------------------------------------------------------------------------------------------------
// Load files
//-------------------------------------------------------------------------------------------------
void Falco::loadPathsFromFile(std::string folder) {
  // Initialize required variables
  free_paths_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for (int i = 0; i < NUM_MAP_CLOUD_STACK; i++) {
    map_cloud_stack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < NUM_GROUPS; i++) {
    precomputed_paths_[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  for (int i = 0; i < NUM_PATHS; i++) {
    paths_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < NUM_GRID_VOXELS; i++) {
    correspondences_[i].resize(0);
  }

  std::cout << "Loading files from folder " << folder << "" << std::endl;

  // Load the files
  loadPrecomputedPaths(folder);
  loadPaths(folder);
  loadPathsList(folder);
  loadCorrespondences(folder);
}

FILE* Falco::openFile(std::string& file) {
  std::cout << "Opening file " << file << std::endl;

  FILE* file_ptr = fopen(file.c_str(), "r");
  if (file_ptr == NULL) {
    std::cout << "Error: Cannot read input files, exit." << std::endl;
    exit(1);
  }

  return file_ptr;
}

int Falco::readPlyHeader(FILE* file_ptr) {
  char str[50];
  int val, point_num;

  std::string str_cur, str_last;
  while (str_cur != "end_header") {
    val = fscanf(file_ptr, "%s", str);
    if (val != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    str_last = str_cur;
    str_cur = std::string(str);

    if (str_cur == "vertex" && str_last == "element") {
      val = fscanf(file_ptr, "%d", &point_num);
      if (val != 1) {
        std::cout << "Error reading input files, exit." << std::endl;
        exit(1);
      }
    }
  }

  return point_num;
}

void Falco::loadPrecomputedPaths(std::string folder) {
  std::string file_name = folder + "precomputed_paths.ply";

  FILE* file_ptr = openFile(file_name);
  int point_num = readPlyHeader(file_ptr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, group_id;
  for (int i = 0; i < point_num; i++) {
    val1 = fscanf(file_ptr, "%f", &point.x);
    val2 = fscanf(file_ptr, "%f", &point.y);
    val3 = fscanf(file_ptr, "%f", &point.z);
    val4 = fscanf(file_ptr, "%d", &group_id);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    if (group_id >= 0 && group_id < NUM_GROUPS) {
      precomputed_paths_[group_id]->push_back(point);
    }
  }

  fclose(file_ptr);
}

void Falco::loadPathsList(std::string folder) {
  std::string file_name = folder + "path_list.ply";

  FILE* file_ptr = openFile(file_name);
  if (NUM_PATHS != readPlyHeader(file_ptr)) {
    std::cout << "Error: Incorrect path number, exit." << std::endl;
    exit(1);
  }

  int val1, val2, val3, val4, val5, path_id, group_id;
  float end_x, end_y, end_z;
  for (int i = 0; i < NUM_PATHS; i++) {
    val1 = fscanf(file_ptr, "%f", &end_x);
    val2 = fscanf(file_ptr, "%f", &end_y);
    val3 = fscanf(file_ptr, "%f", &end_z);
    val4 = fscanf(file_ptr, "%d", &path_id);
    val5 = fscanf(file_ptr, "%d", &group_id);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    if (path_id >= 0 && path_id < NUM_PATHS && group_id >= 0 && group_id < NUM_GROUPS) {
      path_list_[path_id] = group_id;
      end_dir_path_list_[path_id] = 2.0 * atan2(end_y, end_x);
    }
  }

  fclose(file_ptr);
}

void Falco::loadPaths(std::string folder) {
  std::string file_name = folder + "paths.ply";

  FILE* file_ptr = openFile(file_name);
  int point_num = readPlyHeader(file_ptr);

  pcl::PointXYZI point;
  int point_skip_num = 30;
  int point_skip_count = 0;
  int val1, val2, val3, val4, val5, path_id;
  for (int i = 0; i < point_num; i++) {
    val1 = fscanf(file_ptr, "%f", &point.x);
    val2 = fscanf(file_ptr, "%f", &point.y);
    val3 = fscanf(file_ptr, "%f", &point.z);
    val4 = fscanf(file_ptr, "%d", &path_id);
    val5 = fscanf(file_ptr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    if (path_id >= 0 && path_id < NUM_PATHS) {
      point_skip_count++;
      if (point_skip_count > point_skip_num) {
        paths_[path_id]->push_back(point);
        point_skip_count = 0;
      }
    }
  }

  fclose(file_ptr);
}

void Falco::loadCorrespondences(std::string folder) {
  std::string file_name = folder + "correspondences.txt";

  FILE* file_ptr = fopen(file_name.c_str(), "r");
  if (file_ptr == NULL) {
    std::cout << "Error: Cannot read input files, exit." << std::endl;
    exit(1);
  }

  int val1, grid_voxel_id, path_id;
  for (int i = 0; i < NUM_GRID_VOXELS; i++) {
    val1 = fscanf(file_ptr, "%d", &grid_voxel_id);
    if (val1 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    while (1) {
      val1 = fscanf(file_ptr, "%d", &path_id);
      if (val1 != 1) {
        std::cout << "Error reading input files, exit." << std::endl;
        exit(1);
      }

      if (path_id != -1) {
        if (grid_voxel_id >= 0 && grid_voxel_id < NUM_GRID_VOXELS && path_id >= 0 && path_id < NUM_PATHS) {
          correspondences_[grid_voxel_id].push_back(path_id);
        }
      } else {
        break;
      }
    }
  }

  fclose(file_ptr);
}

}  // namespace field_local_planner