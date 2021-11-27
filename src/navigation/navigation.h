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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include <math.h>
#include "vector_map/vector_map.h"
#include "simple_queue.h"
#include "visualization/CImg.h"


using std::string;
using geometry::line2f;
using cimg_library::CImg;
using cimg_library::CImgDisplay;


#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float theta;
  float clearance;
  float radius;
  Eigen::Vector2f CoT;
  float angle_travelled;
  float free_path_length;
  float distance_to_goal;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  float score;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         float time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void TrimDistanceToGoal (struct PathOption& option);

  int PixelHash(Eigen::Vector2f& pixel);

  Eigen::Vector2f PixelUnHash(int hash);



  static constexpr int map_x_max = 45;
  static constexpr int map_x_min = -45;
  static constexpr int map_y_max = 35;
  static constexpr int map_y_min = -35;
  static constexpr float map_resolution = 0.25;
  static constexpr int map_x_width = (int) (map_x_max - map_x_min) / map_resolution;
  static constexpr int map_y_width = (int) (map_y_max - map_y_min) / map_resolution;

  bool occupancy_grid [map_x_width][map_y_width];

  CImg<float> image_real;


 private:

  float* Simple1DTOC();
  float* getBestCurvature();
  float getTravellableDistance(struct PathOption& option);

  float GetMaxDistance(struct PathOption& option, Eigen::Vector2f point);
  float GetMaxDistance2(struct PathOption& option, Eigen::Vector2f point);

  float GetMaxDistanceStraight(Eigen::Vector2f point);
  float Simple1DTOC(Eigen::Vector2f point);
  float getDistanceToGoal(struct PathOption& option);
  float GetAngleBetweenVectors (Eigen::Vector2f a, Eigen::Vector2f b);
  void scorePath(struct PathOption& option);

  void GetClearance (struct PathOption& option);

  void TransformPointCloud(float dx, float dy, float theta);


  void DrawCar();
  void DrawPlan();
  void DrawArcs(float theta, float dist);

  void BuildGraph(const string& map_file);



  Eigen::Vector2f GlobalToRobot(Eigen::Vector2f point);
  vector_map::VectorMap map_;

  std::vector<Eigen::Vector2f> neighbors;



  // REAL CAR CONSTANTS
  const float LENGTH = 0.5;
  const float WIDTH = 0.25;
  const float WHEELBASE = 0.35;
  const float TRACK = 0.25;
  const float SAFETY_MARGIN = 0.1;

  // SIMULATOR CONSTANTS
  // const float LENGTH = 0.535;
  // const float WIDTH = 0.281;
  // const float WHEELBASE = 0.535;
  // const float TRACK = 0.281;
  // const float SAFETY_MARGIN = 0.1;

  float CAR_FRONT = WHEELBASE + (LENGTH+SAFETY_MARGIN*2 - WHEELBASE)/2;
  float CAR_INSIDE = (WIDTH + SAFETY_MARGIN*2) / 2.0;
  float CAR_OUTSIDE = -CAR_INSIDE;

  Eigen::Vector2f INNER_FRONT_CORNER = Eigen::Vector2f(CAR_FRONT, CAR_INSIDE);
  Eigen::Vector2f OUTER_FRONT_CORNER = Eigen::Vector2f(CAR_FRONT, CAR_INSIDE);

  const float MAX_VELOCITY = 1.0;
  const float MAX_ACCEL = 0.4;
  const float MAX_DECEL = 0.4;

  const float INF = std::numeric_limits<float>::max();

  Eigen::Vector2f GOAL = Eigen::Vector2f(25, 0);

  Eigen::Vector2f GetTranslation(float velocity, float curvature, float time);
  float GetRotation(float velocity, float curvature, float time);

  std::vector<line2f> path;

  void MakePlan();
  float heuristic(Eigen::Vector2f current, Eigen::Vector2f goal);
  void SetGoal();

  float radius = 3.0;

  bool goal_initialized_ = false;
  float LATENCY = 0.1;

  float CLEARANCE_WEIGHT = 0.1;
  float GOAL_WEIGHT = 10;

  bool VISUALIZE = 1;
  
  int iteration = 0;
  int scratch = 0;

  float previous_velocity = 0;
  float previous_curvature = 0;

  Eigen::Vector2f obstacle; 



  
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
