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



static bool sEquals (float a, float b) {
  return (a >= b - 0.01 && a <= b + 0.01);
}


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

  struct CurveOption {
    // change in position and heading that this curve makes
    Eigen::Vector2f translation;
    float heading;

    // Is this curve driving forwards or backwards?
    bool backwards;

    // Is this curve availible to states where heading is diagonal
    bool diagonal;

    float curvature;
    
    
    // Edge cost to use in planning
    float cost;
    // Cells to obstacle check
    std::vector<Eigen::Vector2f> stamp;

  };

  struct State {
    float x;
    float y;
    float theta;

    bool operator== (const State& other) const {
      return sEquals(x, other.x) && sEquals(y, other.y) && sEquals(theta, other.theta);
    }

    bool operator< (const State& other) const {
      if (!sEquals(x, other.x))
        return x < other.x;

      if (!sEquals(y, other.y))
        return y < other.y;
      
      if (!sEquals(theta, other.theta))
        return theta < other.theta;

      return false;
    }
  };

  struct PathNode {
    struct State state;
    struct CurveOption curve;

    bool operator< (const PathNode& other) const {
      return state < other.state;
    }

    bool operator== (const PathNode& other) const {
      return state == other.state;
    }
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

  float GetCurveRotation(float distance, float curvature);
  Eigen::Vector2f GetCurveTranslation(float distance, float curvature);

  struct State AddCurveToState(const struct State& cur_state, const struct CurveOption& option);

  bool CheckCurveForCollision (const struct State& cur_state, const struct State& end_state, const struct CurveOption& option);

  bool CheckGoalCondition(const struct State& cur_state, const struct State& goal_state);

  struct State AddTransform (const struct State& cur_state, const struct CurveOption& option);

  bool CheckPointForCollision(const Eigen::Vector2f& point);

  float* GetLatticeAction (); 



  static constexpr int map_x_max = 45;
  static constexpr int map_x_min = -45;
  static constexpr int map_y_max = 35;
  static constexpr int map_y_min = -35;
  static constexpr float map_resolution = 0.25;
  static constexpr int map_x_width = (int) (map_x_max - map_x_min) / map_resolution;
  static constexpr int map_y_width = (int) (map_y_max - map_y_min) / map_resolution;

  bool occupancy_grid [map_x_width][map_y_width];

  static constexpr float state_theta_resolution = M_PI / 4;
  static constexpr float state_xy_resolution = 1.0;


  std::map<struct State, float> StateMap;
  std::vector<struct CurveOption> CurveOptions;

  // Rounds x, y, theta to a multiple of resolution
  struct State StateFactory(float x, float y, float theta) {
    struct State newState;

    newState.x = round(x / state_xy_resolution) * state_xy_resolution;
    newState.y = round(y / state_xy_resolution) * state_xy_resolution;
    newState.theta = round(theta / state_theta_resolution) * state_theta_resolution;


    newState.theta = fmod(newState.theta,M_PI * 2);
    if (newState.theta < 0)
        newState.theta += M_PI * 2;

    return newState;
  }

  void MakeSimpleLattice() {
    CurveOptions.clear();
    CurveOption forward = CurveFactory(Eigen::Vector2f(1, 0), 0, false, 0);
    forward.stamp.emplace_back(-.25,0);
    forward.stamp.emplace_back(-.25,-.25);

    forward.stamp.emplace_back(0,0);
    forward.stamp.emplace_back(0,-.25);

    forward.stamp.emplace_back(.25,0);
    forward.stamp.emplace_back(.25,-.25);

    forward.stamp.emplace_back(.5,0);
    forward.stamp.emplace_back(.5,-.25);

    forward.stamp.emplace_back(.75,-.25);
    forward.stamp.emplace_back(.75,0);

    forward.stamp.emplace_back(1,0);
    forward.stamp.emplace_back(1,-.25);

    CurveOption left = CurveFactory(Eigen::Vector2f(1, 1), M_PI/2, false, -1.0);
    left.stamp.emplace_back(-.25,-.25);
    left.stamp.emplace_back(-.25,0);

    left.stamp.emplace_back(0,-.25);
    left.stamp.emplace_back(0,0);

    left.stamp.emplace_back(.25,-.25);
    left.stamp.emplace_back(.25,0);
    left.stamp.emplace_back(.25,-.25);

    left.stamp.emplace_back(.5,0);
    left.stamp.emplace_back(.5,.25);
    left.stamp.emplace_back(.5,.5);

    left.stamp.emplace_back(.75,.25);
    left.stamp.emplace_back(.75,.5);
    left.stamp.emplace_back(.75,.75);
    left.stamp.emplace_back(.75,1);

    left.stamp.emplace_back(1,.5);
    left.stamp.emplace_back(1,.75);
    left.stamp.emplace_back(1,1);



    CurveOption right = CurveFactory(Eigen::Vector2f(1, -1), -M_PI/2, false, 1.0);
    right.stamp.emplace_back(0,0.25);
    right.stamp.emplace_back(0,0);

    right.stamp.emplace_back(.25,0.25);
    right.stamp.emplace_back(.25,0);
    right.stamp.emplace_back(.25,-.25);

    right.stamp.emplace_back(.5,0);
    right.stamp.emplace_back(.5,-.25);
    right.stamp.emplace_back(.5,-.5);

    right.stamp.emplace_back(.75,-.25);
    right.stamp.emplace_back(.75,-.5);
    right.stamp.emplace_back(.75,-.75);
    right.stamp.emplace_back(.75,-1);

    right.stamp.emplace_back(1,-.5);
    right.stamp.emplace_back(1,-.75);
    right.stamp.emplace_back(1,-1);



    CurveOption back = CurveFactory(Eigen::Vector2f(-1, 0), 0, true, 0);
    back.stamp.emplace_back(0,0);
    back.stamp.emplace_back(0,-.25);

    back.stamp.emplace_back(-.25,0);
    back.stamp.emplace_back(-.25,-.25);

    back.stamp.emplace_back(-.5,0);
    back.stamp.emplace_back(-.5,-.25);

    back.stamp.emplace_back(-.75,0);
    back.stamp.emplace_back(-.75,-.25);

    back.stamp.emplace_back(-1,0);
    back.stamp.emplace_back(-1,-.25);


    CurveOption back_left = CurveFactory(Eigen::Vector2f(-1, 1), -M_PI/2, true, -1.0);
    back_left.stamp.emplace_back(0,0);
    back_left.stamp.emplace_back(0,-.25);

    back_left.stamp.emplace_back(-.25,0);
    back_left.stamp.emplace_back(-.25,-.25);

    back_left.stamp.emplace_back(-.5,.25);
    back_left.stamp.emplace_back(-.5,0);
    back_left.stamp.emplace_back(-.5,-.25);

    back_left.stamp.emplace_back(-.75,0);
    back_left.stamp.emplace_back(-.75,.25);
    back_left.stamp.emplace_back(-.75,.5);

    back_left.stamp.emplace_back(-1,.25);
    back_left.stamp.emplace_back(-1,.5);
    back_left.stamp.emplace_back(-1,.75);
    back_left.stamp.emplace_back(-1,1);

    back_left.stamp.emplace_back(-1.25,.5);
    back_left.stamp.emplace_back(-1.25,.75);
    back_left.stamp.emplace_back(-1.25,1);

    CurveOption back_right = CurveFactory(Eigen::Vector2f(-1, -1), M_PI/2, true, 1.0);
    back_right.stamp.emplace_back(0,0);
    back_right.stamp.emplace_back(0,-.25);

    back_right.stamp.emplace_back(-.25,0);
    back_right.stamp.emplace_back(-.25,-.25);

    back_right.stamp.emplace_back(-.5,0);
    back_right.stamp.emplace_back(-.5,-.25);
    back_right.stamp.emplace_back(-.5,-.5);

    back_right.stamp.emplace_back(-.75,-.25);
    back_right.stamp.emplace_back(-.75,-.5);
    back_right.stamp.emplace_back(-.75,-.75);

    back_right.stamp.emplace_back(-1,-.5);
    back_right.stamp.emplace_back(-1,-.75);
    back_right.stamp.emplace_back(-1,-1);
    back_right.stamp.emplace_back(-1,-1.25);

    back_right.stamp.emplace_back(-1.25,-.75);
    back_right.stamp.emplace_back(-1.25,-1);
    back_right.stamp.emplace_back(-1.25,-1.25);

    CurveOptions.emplace_back(forward);
    CurveOptions.emplace_back(left);
    CurveOptions.emplace_back(right);

    CurveOptions.emplace_back(back);
    CurveOptions.emplace_back(back_left);
    CurveOptions.emplace_back(back_right);
  }

  void MakeComplexLattice() {
    CurveOptions.clear();
    // MakeSimpleLattice();
    CurveOption forward = CurveFactory(Eigen::Vector2f(1, 0), 0, false, 0);
    forward.stamp.emplace_back(0,0);
    forward.stamp.emplace_back(0,-.25);

    forward.stamp.emplace_back(.25,0);
    forward.stamp.emplace_back(.25,-.25);

    forward.stamp.emplace_back(.5,0);
    forward.stamp.emplace_back(.5,-.25);

    forward.stamp.emplace_back(.75,-.25);
    forward.stamp.emplace_back(.75,0);

    forward.stamp.emplace_back(1,0);
    forward.stamp.emplace_back(1,-.25);

    CurveOption left = CurveFactory(Eigen::Vector2f(1, 1), M_PI/2, false, -1.0);
    left.cost = M_PI / 2;
    left.stamp.emplace_back(-.25,-.25);
    left.stamp.emplace_back(-.25,0);

    left.stamp.emplace_back(0,-.25);
    left.stamp.emplace_back(0,0);

    left.stamp.emplace_back(.25,-.25);
    left.stamp.emplace_back(.25,0);
    left.stamp.emplace_back(.25,-.25);

    left.stamp.emplace_back(.5,0);
    left.stamp.emplace_back(.5,.25);
    left.stamp.emplace_back(.5,.5);

    left.stamp.emplace_back(.75,.25);
    left.stamp.emplace_back(.75,.5);
    left.stamp.emplace_back(.75,.75);
    left.stamp.emplace_back(.75,1);

    left.stamp.emplace_back(1,.5);
    left.stamp.emplace_back(1,.75);
    left.stamp.emplace_back(1,1);

    CurveOption right = CurveFactory(Eigen::Vector2f(1, -1), -M_PI/2, false, 1.0);
    right.cost = M_PI / 2;
    right.stamp.emplace_back(0,0.25);
    right.stamp.emplace_back(0,0);

    right.stamp.emplace_back(.25,0.25);
    right.stamp.emplace_back(.25,0);
    right.stamp.emplace_back(.25,-.25);

    right.stamp.emplace_back(.5,0);
    right.stamp.emplace_back(.5,-.25);
    right.stamp.emplace_back(.5,-.5);

    right.stamp.emplace_back(.75,-.25);
    right.stamp.emplace_back(.75,-.5);
    right.stamp.emplace_back(.75,-.75);
    right.stamp.emplace_back(.75,-1);

    right.stamp.emplace_back(1,-.5);
    right.stamp.emplace_back(1,-.75);
    right.stamp.emplace_back(1,-1);



    CurveOption back = CurveFactory(Eigen::Vector2f(-1, 0), 0, true, 0);
    back.stamp.emplace_back(0,0);
    back.stamp.emplace_back(0,-.25);

    back.stamp.emplace_back(-.25,0);
    back.stamp.emplace_back(-.25,-.25);

    back.stamp.emplace_back(-.5,0);
    back.stamp.emplace_back(-.5,-.25);

    back.stamp.emplace_back(-.75,0);
    back.stamp.emplace_back(-.75,-.25);

    back.stamp.emplace_back(-1,0);
    back.stamp.emplace_back(-1,-.25);


    CurveOption back_left = CurveFactory(Eigen::Vector2f(-1, 1), -M_PI/2, true, -1.0);
    back_left.cost = M_PI / 2;
    back_left.stamp.emplace_back(0,0);
    back_left.stamp.emplace_back(0,-.25);

    back_left.stamp.emplace_back(-.25,0);
    back_left.stamp.emplace_back(-.25,-.25);

    back_left.stamp.emplace_back(-.5,.25);
    back_left.stamp.emplace_back(-.5,0);
    back_left.stamp.emplace_back(-.5,-.25);

    back_left.stamp.emplace_back(-.75,0);
    back_left.stamp.emplace_back(-.75,.25);
    back_left.stamp.emplace_back(-.75,.5);

    back_left.stamp.emplace_back(-1,.25);
    back_left.stamp.emplace_back(-1,.5);
    back_left.stamp.emplace_back(-1,.75);
    back_left.stamp.emplace_back(-1,1);

    back_left.stamp.emplace_back(-1.25,.5);
    back_left.stamp.emplace_back(-1.25,.75);
    back_left.stamp.emplace_back(-1.25,1);

    CurveOption back_right = CurveFactory(Eigen::Vector2f(-1, -1), M_PI/2, true, 1.0);
    back_right.cost = M_PI / 2;
    back_right.stamp.emplace_back(0,0);
    back_right.stamp.emplace_back(0,-.25);

    back_right.stamp.emplace_back(-.25,0);
    back_right.stamp.emplace_back(-.25,-.25);

    back_right.stamp.emplace_back(-.5,0);
    back_right.stamp.emplace_back(-.5,-.25);
    back_right.stamp.emplace_back(-.5,-.5);

    back_right.stamp.emplace_back(-.75,-.25);
    back_right.stamp.emplace_back(-.75,-.5);
    back_right.stamp.emplace_back(-.75,-.75);

    back_right.stamp.emplace_back(-1,-.5);
    back_right.stamp.emplace_back(-1,-.75);
    back_right.stamp.emplace_back(-1,-1);
    back_right.stamp.emplace_back(-1,-1.25);

    back_right.stamp.emplace_back(-1.25,-.75);
    back_right.stamp.emplace_back(-1.25,-1);
    back_right.stamp.emplace_back(-1.25,-1.25);

    CurveOption right_45 = CurveFactory(Eigen::Vector2f(1, -1), -M_PI/4, false, 1.0);
    right_45.cost = M_PI / 2;
    right_45.stamp.emplace_back(0,-.25);
    right_45.stamp.emplace_back(0,0);

    right_45.stamp.emplace_back(.25,-0.5);
    right_45.stamp.emplace_back(.25,0);
    right_45.stamp.emplace_back(.25,-.25);

    right_45.stamp.emplace_back(.5,-.75);
    right_45.stamp.emplace_back(.5,-.25);
    right_45.stamp.emplace_back(.5,-.5);
    right_45.stamp.emplace_back(.5,-1);

    right_45.stamp.emplace_back(.75,-1.25);
    right_45.stamp.emplace_back(.75,-.5);
    right_45.stamp.emplace_back(.75,-.75);
    right_45.stamp.emplace_back(.75,-1);

    // right_45.stamp.emplace_back(1,-.5);
    right_45.stamp.emplace_back(1,-.75);
    right_45.stamp.emplace_back(1,-1);
    right_45.stamp.emplace_back(1,-1.25);



    CurveOption left_45 = CurveFactory(Eigen::Vector2f(1, 1), M_PI/4, false, -1.0);
    left_45.cost = M_PI / 2;
    left_45.stamp.emplace_back(-.25,-.25);
    left_45.stamp.emplace_back(-.25,0);

    left_45.stamp.emplace_back(0,-.25);
    left_45.stamp.emplace_back(0,0);
    left_45.stamp.emplace_back(0,.25);

    left_45.stamp.emplace_back(.25,-.25);
    left_45.stamp.emplace_back(.25,0);
    left_45.stamp.emplace_back(.25,.25);

    left_45.stamp.emplace_back(.5,0);
    left_45.stamp.emplace_back(.5,.25);
    left_45.stamp.emplace_back(.5,.5);

    left_45.stamp.emplace_back(.75,.25);
    left_45.stamp.emplace_back(.75,.5);
    left_45.stamp.emplace_back(.75,.75);
    // left_45.stamp.emplace_back(.75,1);

    left_45.stamp.emplace_back(1,.5);
    left_45.stamp.emplace_back(1,.75);
    left_45.stamp.emplace_back(1,1);


    CurveOption back_left_45 = CurveFactory(Eigen::Vector2f(-1, 1), -M_PI/4, true, -1.0);
    back_left_45.cost = M_PI / 2;
    back_left_45.stamp.emplace_back(0,0);
    back_left_45.stamp.emplace_back(0,-.25);

    back_left_45.stamp.emplace_back(-.25,0);
    back_left_45.stamp.emplace_back(-.25,-.25);
    back_left_45.stamp.emplace_back(-.25,.25);


    back_left_45.stamp.emplace_back(-.5,.25);
    back_left_45.stamp.emplace_back(-.5,0);
    back_left_45.stamp.emplace_back(-.5,-.25);

    back_left_45.stamp.emplace_back(-.75,0);
    back_left_45.stamp.emplace_back(-.75,.25);
    back_left_45.stamp.emplace_back(-.75,.5);

    back_left_45.stamp.emplace_back(-1,.25);
    back_left_45.stamp.emplace_back(-1,.5);
    back_left_45.stamp.emplace_back(-1,.75);
    back_left_45.stamp.emplace_back(-1,1);

    back_left_45.stamp.emplace_back(-1.25,.5);
    back_left_45.stamp.emplace_back(-1.25,.75);
    back_left_45.stamp.emplace_back(-1.25,1);


    CurveOption back_right_45 = CurveFactory(Eigen::Vector2f(-1, -1), M_PI/4, true, 1.0);
    back_right_45.cost = M_PI / 2;
    back_right_45.stamp.emplace_back(0,0);
    back_right_45.stamp.emplace_back(0,-.25);

    back_right_45.stamp.emplace_back(-.25,0);
    back_right_45.stamp.emplace_back(-.25,-.25);

    back_right_45.stamp.emplace_back(-.5,0);
    back_right_45.stamp.emplace_back(-.5,-.25);
    back_right_45.stamp.emplace_back(-.5,-.5);

    back_right_45.stamp.emplace_back(-.75,-.25);
    back_right_45.stamp.emplace_back(-.75,-.5);
    back_right_45.stamp.emplace_back(-.75,-.75);

    back_right_45.stamp.emplace_back(-1,-.5);
    back_right_45.stamp.emplace_back(-1,-.75);
    back_right_45.stamp.emplace_back(-1,-1);
    back_right_45.stamp.emplace_back(-1,-1.25);

    back_right_45.stamp.emplace_back(-1.25,-.75);
    back_right_45.stamp.emplace_back(-1.25,-1);
    back_right_45.stamp.emplace_back(-1.25,-1.25);


    CurveOption forward_d = CurveFactory(Eigen::Vector2f(sqrt(2), 0), 0, false, 0);
    forward_d.diagonal = true;
    forward_d.stamp.emplace_back(0,0);
    forward_d.stamp.emplace_back(0,-.25);

    forward_d.stamp.emplace_back(.25,0);
    forward_d.stamp.emplace_back(.25,-.25);

    forward_d.stamp.emplace_back(.5,0);
    forward_d.stamp.emplace_back(.5,-.25);

    forward_d.stamp.emplace_back(.75,-.25);
    forward_d.stamp.emplace_back(.75,0);

    forward_d.stamp.emplace_back(1,0);
    forward_d.stamp.emplace_back(1,-.25);

    forward_d.stamp.emplace_back(1.25,0);
    forward_d.stamp.emplace_back(1.25,-.25);

    CurveOption right_d = CurveFactory(Eigen::Vector2f(sqrt(2), 0), M_PI/4, false, 0);
    right_d.diagonal = true;
    right_d.stamp = forward_d.stamp;
    right_d.stamp.emplace_back(1, -.5);
    right_d.stamp.emplace_back(.75, -.5);
    right_d.stamp.emplace_back(.5, -.5);
    right_d.stamp.emplace_back(.25, -.5);


    right_d.cost += 0.1;

    CurveOption left_d = CurveFactory(Eigen::Vector2f(sqrt(2), 0), -M_PI/4, false, 0);
    left_d.diagonal = true;
    left_d.stamp = forward_d.stamp;
    left_d.stamp.emplace_back(1, 0.25);
    left_d.stamp.emplace_back(.75, 0.25);
    left_d.stamp.emplace_back(.5, 0.25);

    left_d.cost += 0.1;

    CurveOptions.emplace_back(forward);
    CurveOptions.emplace_back(left);
    CurveOptions.emplace_back(right);

    CurveOptions.emplace_back(back);
    CurveOptions.emplace_back(back_left);
    CurveOptions.emplace_back(back_right);

    CurveOptions.emplace_back(right_45);
    CurveOptions.emplace_back(left_45);
    CurveOptions.emplace_back(back_right_45);
    CurveOptions.emplace_back(back_left_45);

    CurveOptions.emplace_back(forward_d);
    CurveOptions.emplace_back(right_d);
    CurveOptions.emplace_back(left_d);
  }

  struct CurveOption CurveFactory(Eigen::Vector2f translation, float heading, bool backwards, float curvature) {
    struct CurveOption newCurve;
    newCurve.translation = translation;
    newCurve.heading = heading;
    newCurve.backwards = backwards;
    newCurve.curvature = curvature;
    newCurve.diagonal = false;
    newCurve.cost = translation.norm();
    newCurve.stamp = std::vector<Eigen::Vector2f>();
    return newCurve;
  }

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
  Eigen::Matrix2f GetRotationMatrix (const float angle);

  void VisualizeLatticePath();
  void VisualizeLatticePath2();


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

  const float MAX_VELOCITY = 0.6;
  const float MAX_ACCEL = 0.4;
  const float MAX_DECEL = 0.4;

  const float INF = std::numeric_limits<float>::max();

  Eigen::Vector2f GOAL = Eigen::Vector2f(25, 0);
  float GOAL_ANGLE = 0;


  Eigen::Vector2f GetTranslation(float velocity, float curvature, float time);
  float GetRotation(float velocity, float curvature, float time);

  std::vector<line2f> path;
  std::vector<struct PathNode> path_states;


  void MakePlan();
  void MakeLatticePlan();
  float heuristic(Eigen::Vector2f current, Eigen::Vector2f goal);
  float LatticeHeuristic(const struct State& current, const struct State& goal);

  float HeadingHeuristic(const struct State& current, const struct State& goal);
  void SetGoal();

  float radius = 3.0;

  bool goal_initialized_ = false;
  float LATENCY = 0.1;

  float CLEARANCE_WEIGHT = 0.1;
  float GOAL_WEIGHT = 10;

  bool VISUALIZE = 1;
  float heading_scale = 10.0;
  int iteration = 0;
  int scratch = 0;

  float previous_velocity = 0;
  float previous_curvature = 0;

  bool start_plan = false;
  bool plan_active = false;

  float total_distance = 0;
  Eigen::Vector2f previous_loc; 

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
