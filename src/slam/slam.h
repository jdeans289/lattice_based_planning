//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose {
  float delta_x;
  float delta_y;
  float delta_theta;
  double probability;
};

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  Eigen::Matrix2f GetRotationMatrix (const float angle);

  std::vector<Eigen::Vector2f> GetScanPointCloud(const std::vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max);
  
  slam::Pose MostLikelyPose();
  void ReinitializeCube();
  void PrintCube(int num_elems);
  void GetBoundingBox(float bounds[4]);

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  float k_1 = 0.1;
  float k_2 = 0.1;
  float laser_off = 0.2;

  float distance_travelled_og = 0.8; 
  float distance_travelled = distance_travelled_og;
  float angle_travelled_og = 0.5;
  float angle_travelled = angle_travelled_og;

  Eigen::Vector2f p_odom_vector;
  float p_odom_angle;

  // int image_disp = 10;

  float std_dev_sensor = 0.15;

  // negative displacements?

  static constexpr float x_incr = 0.05;
  static constexpr float y_incr = 0.05;
  static constexpr float theta_incr = 0.01; 
 
  static constexpr float x_max = 1;
  static constexpr float y_max = 1;
  static constexpr float theta_max = 1.6; 
  static constexpr float x_min = -1;
  static constexpr float y_min = -1;
  static constexpr float theta_min = -.6; 

  static constexpr int x_width = (int) (x_max - x_min) / x_incr;
  static constexpr int y_width = (int) (y_max - y_min) / y_incr;
  static constexpr int theta_width = (int) (theta_max - theta_min) / theta_incr;

  double cube[x_width][y_width][theta_width];
  double obsv[x_width][y_width][theta_width];


  slam::Pose previous_pose = {
    -1000, -1000, -1000, 0.0
  };
  slam::Pose cumulative_transform = {
    0, 0, 0, 0.0
  };
  slam::Pose best_pose;
  std::vector<Eigen::Vector2f> previous_scan;
  
  static constexpr float x_image_incr = 0.05;
  static constexpr float y_image_incr = 0.05;
  static constexpr float x_image_max = 3;
  static constexpr float y_image_max = 3;
  static constexpr float x_image_min = -3;
  static constexpr float y_image_min = -3;
  static constexpr int x_image_width = (int) (x_image_max - x_image_min) / x_image_incr;
  static constexpr int y_image_width = (int) (y_image_max - y_image_min) / y_image_incr;

  std::vector<Eigen::Vector2f> estimated_map;
  void PrintImage(float image[x_image_width][y_image_width]);
  void InitializeImage(float image[x_image_width][y_image_width]);

  bool quit = false;

  Eigen::Vector2f TransformFromBase(Eigen::Vector2f point, float dx, float dy, float dtheta);
  Eigen::Vector2f TransformToBase(Eigen::Vector2f point, float dx, float dy, float dtheta);

  void AddToMap(std::vector<Eigen::Vector2f>  current_scan);
  void EvaluateMotionModel();
  void EvaluateObservationLikelihood(std::vector<Eigen::Vector2f> current_scan);





};
}  // namespace slam

#endif   // SRC_SLAM_H_
