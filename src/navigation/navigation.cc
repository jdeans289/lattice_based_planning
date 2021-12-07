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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "visualization/CImg.h"

using Eigen::Vector2f;
using Eigen::Matrix2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;
using cimg_library::CImg;
using cimg_library::CImgDisplay;


using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
// const float kEpsilon = 1e-5;
} //namespace


const float kEpsilon = 1e-5;
bool fEquals (float a, float b) {
  return (a >= b - kEpsilon && a <= b + kEpsilon);
}

bool vEquals (Eigen::Vector2f a, Eigen::Vector2f b) {
  return fEquals(a[0], b[0]) && fEquals(a[1], b[1]);
}

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  neighbors.emplace_back(1,1);
  neighbors.emplace_back(1,0);
  neighbors.emplace_back(1,-1);
  neighbors.emplace_back(0,1);
  neighbors.emplace_back(0,-1);  
  neighbors.emplace_back(-1,1);
  neighbors.emplace_back(-1,0);
  neighbors.emplace_back(-1,-1);

  // for (float i = -.4; i <= .4; i += 0.2) {
  CurveOption forward;
  forward.translation = Eigen::Vector2f(1, 0);
  forward.heading = 0;
  forward.backwards = false;
  forward.curvature = 0;

  CurveOption left;
  left.translation = Eigen::Vector2f(1, 1);
  left.heading = M_PI / 2;
  left.backwards = false;
  left.curvature = -1.0;

  CurveOption right;
  right.translation = Eigen::Vector2f(1, -1);
  right.heading = -(M_PI / 2);
  right.backwards = false;
  left.curvature = 1.0;

  CurveOption back;
  back.translation = Eigen::Vector2f(-1, 0);
  back.heading = 0;
  back.backwards = true;
  back.curvature = 0;

  CurveOption back_left;
  back_left.translation = Eigen::Vector2f(-1, 1);
  back_left.heading = -(M_PI / 2);
  back_left.backwards = true;
  back_left.curvature = -1.0;

  CurveOption back_right;
  back_right.translation = Eigen::Vector2f(-1, -1);
  back_right.heading = (M_PI / 2);
  back_right.backwards = true;
  back_right.curvature = 1.0;

  CurveOptions.emplace_back(forward);
  CurveOptions.emplace_back(left);
  CurveOptions.emplace_back(right);

  CurveOptions.emplace_back(back);
  CurveOptions.emplace_back(back_left);
  CurveOptions.emplace_back(back_right);


  // }

  BuildGraph(map_file);

}

void Navigation::BuildGraph(const string& map_file) {
  map_.Load(map_file);
  printf("Initializing...\n");

  image_real = CImg<unsigned char> (map_x_width,map_y_width,1,3,0);

  for (int i = 0; i < map_x_width; i++) {
    for (int j = 0; j < map_y_width; j++) {
      occupancy_grid[i][j] = 0;
    }
  }

  // printf("map lines: %d\n", (int) map_.lines.size());

  for (size_t j = 0; j < map_.lines.size(); ++j) {
    const line2f map_line = map_.lines[j];
    int p0_pixel_x = (map_line.p0.x() - map_x_min) / map_resolution;
    int p0_pixel_y = (map_line.p0.y() - map_y_min) / map_resolution;

    int p1_pixel_x = (map_line.p1.x() - map_x_min) / map_resolution;
    int p1_pixel_y = (map_line.p1.y() - map_y_min) / map_resolution;

    // printf("Line: %d %d %d %d\n", p0_pixel_x, p0_pixel_y, p1_pixel_x, p1_pixel_y);

    // All the lines are horizontal or vertical, so cheat a little bit
    if (abs(p0_pixel_x - p1_pixel_x) < abs(p0_pixel_y - p1_pixel_y)) {
      for (int y = std::min(p0_pixel_y, p1_pixel_y); y < std::max(p0_pixel_y, p1_pixel_y); y++) {
        occupancy_grid[p0_pixel_x][y] = 1.0;

        const unsigned char color[] = { 255,255,255 };
        image_real.draw_point(p0_pixel_x,map_y_width-y,color);
      }
    } else {
      for (int x = std::min(p0_pixel_x, p1_pixel_x); x < std::max(p0_pixel_x, p1_pixel_x); x++) {
        occupancy_grid[x][p0_pixel_y] = 1.0;

        const unsigned char color[] = { 255,255,255 };
        image_real.draw_point(x,map_y_width-p0_pixel_y,color);
      }
    }
  }

  printf("Grid initialized!\n");

  // GOAL = Vector2f(-21, 14);
  // robot_loc_ = Vector2f(-14, 9);
  // MakePlan();
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  goal_initialized_ = true;
  GOAL = loc;
  GOAL_ANGLE = angle;

  if (localization_initialized_) {
    MakeLatticePlan();
    start_plan = true;
    plan_active = false;
  }

}

int Navigation::PixelHash(Vector2f& pixel) {
  return pixel[0] * map_y_width + pixel[1];
}

Vector2f Navigation::PixelUnHash(int hash) {
  int x = hash / map_y_width;
  int y = hash % map_y_width;
  return Vector2f(x, y);
}

void Navigation::MakePlan() {

    int loc_x = (robot_loc_[0] - map_x_min) / map_resolution;
    int loc_y = (robot_loc_[1] - map_y_min) / map_resolution;
    Vector2f loc_pix(loc_x, loc_y);
    const unsigned char color2[] = { 0,255,0 };


    int goal_x = (GOAL[0] - map_x_min) / map_resolution;
    int goal_y = (GOAL[1] - map_y_min) / map_resolution;
    Vector2f goal_pix(goal_x, goal_y);
    int goal_hash = PixelHash(goal_pix);

    const unsigned char color[] = { 255,0,0 };
    image_real.draw_point(goal_pix[0],map_y_width-goal_pix[1],color);
    
    // printf("Goal: %lf %lf\n", goal_pix[0], goal_pix[1]);
    // printf("loc: %d %d\n", loc_x, loc_y);


    SimpleQueue<int, float> frontier;
    frontier.Push(PixelHash(loc_pix), 0);

    std::map<int, float> cost;
    cost[PixelHash(loc_pix)] = 0;

    std::map<int, int> parent;
    parent[PixelHash(loc_pix)] = -1;

    // printf("Begin hash: %d\n", PixelHash(loc_pix));
    // printf("Goal Hash: %d\n", goal_hash);

    int it = 0;
    while (!frontier.Empty()) {
      // printf("\n\n**********************Iteration %d*******************************\n", it);
      int current_hash = frontier.Pop();
      // printf("Current hash: %d\n", current_hash);
      Eigen::Vector2f current = PixelUnHash(current_hash);
      // printf("Current location: %lf %lf\n", current[0], current[1]);
      if (current_hash == goal_hash) {
        printf("Found goal!\n");
        break;
      }

      for (int i = 0; i < 8; i++) {
        Eigen::Vector2f next = current + neighbors[i];
        if (occupancy_grid[(int)next[0]][(int)next[1]] == 1)
          continue;

        int next_hash = PixelHash(next);
        // printf("Next hash: %d\n", next_hash);
        // printf("Next location: %lf %lf\n", next[0], next[1]);
        float new_cost = cost[current_hash] + neighbors[i].norm();
        // printf("New cost: %f\n", new_cost);
        if (cost.find(next_hash) == cost.end() || new_cost < cost[next_hash]) {
          cost[next_hash] = new_cost;
          // printf("Cost in map: %lf\n", new_cost + heuristic(next, goal_pix));
          frontier.Push(next_hash, -(new_cost + heuristic(next, goal_pix)));
          parent[next_hash] = current_hash;
        }

        // printf("\n");
      }
      it++;
    }

    const unsigned char color3[] = { 0,0,255 };

    int curr_hash = goal_hash;
    vector<Vector2f> path_points;
    path_points.emplace_back(goal_pix);
    while (curr_hash != PixelHash(loc_pix)) {
      curr_hash = parent[curr_hash];
      Vector2f path_loc = PixelUnHash(curr_hash);
      path_points.emplace_back(path_loc);
      image_real.draw_point(path_loc[0],map_y_width-path_loc[1],color3);
    }
    image_real.draw_point(loc_x,map_y_width-loc_y,color2);

    image_real.save("goalmap.bmp");
    printf("Image saved\n");

    printf("Points in path: %lu\n", path_points.size());

    std::reverse(path_points.begin(), path_points.end());
    path.clear();

    Vector2f curr_p0 = path_points[0];
    Vector2f curr_p1 = path_points[1];
    unsigned index = 2;
    Vector2f curr_direction = curr_p1 - curr_p0;
    while (index < path_points.size()) {
      Vector2f next = path_points[index];
      Vector2f new_direction = next - curr_p1;
      if (!vEquals(new_direction, curr_direction)) {
        // Terminate the line and start a new one
        float p0_x = (curr_p0[0] * map_resolution) +  map_x_min;
        float p0_y = (curr_p0[1] * map_resolution) +  map_y_min;
        Vector2f p0 (p0_x, p0_y);

        float p1_x = (curr_p1[0] * map_resolution) +  map_x_min;
        float p1_y = (curr_p1[1] * map_resolution) +  map_y_min;
        Vector2f p1 (p1_x, p1_y);

        line2f path_line(p0, p1);
        path.emplace_back(path_line);
        curr_p0 = curr_p1;
        curr_p1 = next;
        curr_direction = new_direction;
      } else {
        // Extend the line
        curr_p1 = next;
      }
      index++;
    }
    // Add final line
    float p0_x = (curr_p0[0] * map_resolution) +  map_x_min;
    float p0_y = (curr_p0[1] * map_resolution) +  map_y_min;
    Vector2f p0 (p0_x, p0_y);

    float p1_x = (curr_p1[0] * map_resolution) +  map_x_min;
    float p1_y = (curr_p1[1] * map_resolution) +  map_y_min;
    Vector2f p1 (p1_x, p1_y);
    line2f path_line(p0, p1);
    path.emplace_back(path_line);

    printf("Finished with plan!\n");
    printf("Lines in plan: %lu\n", path.size());

    DrawPlan();
} 

// struct State Navigation::AddCurveToState(const struct State& cur_state, const struct CurveOption& option) {
//   // Vector2f translation = GetCurveTranslation(option.distance, option.curvature);
//   // float rotation = GetCurveRotation(option.distance, option.curvature);

//   // printf("Trying curve: curvature %f\tdistance: %f\n", option.curvature, option.distance);
//   // printf("\ttranslation: %f %f\trotation: %f\n", translation[0], translation[1], rotation);
//   // PrintState(cur_state);



//   return ;
// }

bool Navigation::CheckPointForCollision(const Eigen::Vector2f& point) {

  int pixel_x = (point[0] - map_x_min) / map_resolution;
  int pixel_y = (point[1] - map_y_min) / map_resolution;

  return occupancy_grid[pixel_x][pixel_y];
}

bool Navigation::CheckCurveForCollision (const struct State& cur_state, const struct State& end_state, const struct CurveOption& option) {

  Vector2f p0 (cur_state.x, cur_state.y);
  Vector2f p1 (end_state.x, end_state.y);

  Vector2f connect = p1 - p0;
  float length = connect.norm();
  float increment = 0;
  while (increment < length) {
    Vector2f point_to_check = p0 + (increment/length) * connect;

    if (CheckPointForCollision(point_to_check))
      return true;

    increment += 0.25;
  }

  return false;
}

void PrintState(const struct State& state) {
  printf("State x: %f\ty: %f\ttheta: %f\n", (double)state.x, (double)state.y, (double)state.theta);
}

// "close enough"
bool Navigation::CheckGoalCondition(const struct State& cur_state, const struct State& goal_state) {
  // let's call goal tolerance within 0.5 m and pi/2 degrees
  
  Vector2f curr_pos(cur_state.x, cur_state.y);
  Vector2f goal_pos(goal_state.x, goal_state.y);

  if ((curr_pos - goal_pos).norm() < 0.5 && abs(cur_state.theta - goal_state.theta) < M_PI / 4) {
    return true;
  }
  return false;

} 

struct State Navigation::AddTransform (const struct State& cur_state, const struct CurveOption& option) {
  struct State transformed_state;

  Eigen::Vector2f new_pos(cur_state.x, cur_state.y);

  Eigen::Matrix2f rot = GetRotationMatrix(cur_state.theta);

  new_pos += rot * option.translation; 

  transformed_state.x = new_pos[0];
  transformed_state.y = new_pos[1];
  transformed_state.theta = cur_state.theta + option.heading;

  return transformed_state;
}

void Navigation::PrecomputeObstacleChecking(struct State& location_state, 
                                            bool visited[(int) (map_x_max - map_x_min)][(int) (map_y_max - map_y_min)])
{
  int pixel_x = (location_state.x - map_x_min);
  int pixel_y = (location_state.y - map_y_min);

  if (visited[pixel_x][pixel_y])
    return;

  visited[pixel_x][pixel_y] = true;
  for (struct CurveOption neighbor : CurveOptions) 
  {
    bool reachable = true;
    struct State next_state = AddTransform(location_state, neighbor);
    float incr = map_resolution;

    // x direction
    if (fEquals(location_state.theta, 0) || fEquals(location_state.theta, M_PI) || fEquals(location_state.theta, -M_PI))
    {
      // Backwards
      if (neighbor.translation[0] < 0)
        incr = map_resolution * -1;

      for (float i = 0; abs(i) <= abs(neighbor.translation[0]) && reachable; i += map_resolution)
      {
        int next_x = ((location_state.x + i) - map_x_min);
        int map_x = next_x / map_resolution;
        int map_y = pixel_y / map_resolution;

        if (occupancy_grid[map_x][map_y])
          reachable = False;
      }

      incr = map_resolution;
      if (neighbor.translation[1] < 0)
        incr = -incr;
      
      for (float i = 0; abs(i) <= abs(neighbor.translation[1]) && reachable; i += map_resolution)
      {
        int next_x = ((location_state.x + neighbor.translation[0]) - map_x_min);
        int map_x = next_x / map_resolution;
        int map_y = ((location_state.y + i) - map_y_min) / map_resolution;

        if (occupancy_grid[map_x][map_y])
          reachable = False;
      }

      if (reachable)
      {
        checking_grid[(int) (next_state.x - map_x_min)][(int) (next_state.y - map_y_min)] = true;
        PrecomputeObstacleChecking(next_state, visited);
      }
    }
    else
    {
      if (neighbor.translation[0] < 0)
        incr = map_resolution * -1;

      for (float i = 0; abs(i) <= abs(neighbor.translation[0]) && reachable; i += map_resolution)
      {
        int next_y = ((location_state.y + i) - map_x_min);
        int map_x = pixel_x / map_resolution;
        int map_y = next_y / map_resolution;

        if (occupancy_grid[map_x][map_y])
          reachable = False;
      }

      incr = map_resolution;
      if (neighbor.translation[1] < 0)
        incr = -incr;
      
      for (float i = 0; abs(i) <= abs(neighbor.translation[1]) && reachable; i += map_resolution)
      {
        int next_y = ((location_state.y + neighbor.translation[0]) - map_x_min);
        int map_y = next_y / map_resolution;
        int map_x = ((location_state.x + i) - map_x_min) / map_resolution;

        if (occupancy_grid[map_x][map_y])
          reachable = False;
      }
    }
    if (reachable)
    {
      checking_grid[(int) (next_state.x - map_x_min)][(int) (next_state.y - map_y_min)] = true;
      PrecomputeObstacleChecking(next_state, visited);
    }
  }
}

void Navigation::MakeLatticePlan() {

    struct State location_state = StateFactory(robot_loc_[0], robot_loc_[1], robot_angle_);
    printf("Initial location: ");
    PrintState(location_state);

    struct State goal_state = StateFactory(GOAL[0], GOAL[1], GOAL_ANGLE);
    printf("Goal location: ");
    PrintState(goal_state);

    SimpleQueue<struct State, float> frontier;
    frontier.Push(location_state, 0);

    std::map<struct State, float> cost;
    cost[location_state] = 0;

    std::map<struct State, struct PathNode> parent;
    CurveOption no_action;
    no_action.translation = Eigen::Vector2f(0,0);
    no_action.heading = 0;
    no_action.backwards = false;
    no_action.curvature = 0;

    int failed = 1; 
    printf("Starting lattice based exploration\n");
    int it = 0;

    // Begin A* search
    while (!frontier.Empty()) {
      if (it > 500) {
        printf("Planning failed\n");
        break;
      }

      struct State current_loc = frontier.Pop();
      if (CheckGoalCondition(current_loc, goal_state)) {
        printf("Found goal!\n");
        failed = 0;
        break;
      }

      // Check each of the possible curves in the lattice
      for (struct CurveOption neighbor : CurveOptions) {

        struct State next_state = AddTransform(current_loc, neighbor);
        
        // check for collisions
        if (CheckCurveForCollision(current_loc, next_state, neighbor))
          continue;

        // Edge cost is the length of the curve
        float new_cost = cost[current_loc] + neighbor.translation.norm();

        // If the node has never been explored before, or if the cost is less than a previously found cost:
        if (cost.find(next_state) == cost.end() || new_cost < cost[next_state]) {
          // Update the cost
          cost[next_state] = new_cost;
          frontier.Push(next_state, -(new_cost + LatticeHeuristic(next_state, goal_state)));
          struct PathNode curr_path;
          curr_path.state = current_loc;
          curr_path.curve = neighbor;
          // Add to parent list - used to reconstruct path
          parent[next_state] = curr_path;
        }
      }
      it++;
    }

    if (failed) {
      printf("Plan failed\n");
      return;
    }


    // const unsigned char color3[] = { 0,0,255 };
    struct PathNode goal_node;
    goal_node.state = goal_state;
    goal_node.curve = no_action;

    struct PathNode curr_node;
    curr_node.state = goal_state;

    // Reconstruct path
    path_states.clear();
    path_states.emplace_back(goal_node);
    while (!(curr_node.state == location_state)) {
      curr_node = parent[curr_node.state];
      path_states.emplace_back(curr_node);
    }

    printf("States in path: %lu\n", path_states.size());

    std::reverse(path_states.begin(), path_states.end());
    for (unsigned i = 0; i < path_states.size(); i++) {
      printf("State %d: ", i);
      PrintState(path_states[i].state);
    }

    VisualizeLatticePath();


    // path.clear();

    // Vector2f curr_p0 = path_points[0];
    // Vector2f curr_p1 = path_points[1];
    // unsigned index = 2;
    // Vector2f curr_direction = curr_p1 - curr_p0;
    // while (index < path_points.size()) {
    //   Vector2f next = path_points[index];
    //   Vector2f new_direction = next - curr_p1;
    //   if (!vEquals(new_direction, curr_direction)) {
    //     // Terminate the line and start a new one
    //     float p0_x = (curr_p0[0] * map_resolution) +  map_x_min;
    //     float p0_y = (curr_p0[1] * map_resolution) +  map_y_min;
    //     Vector2f p0 (p0_x, p0_y);

    //     float p1_x = (curr_p1[0] * map_resolution) +  map_x_min;
    //     float p1_y = (curr_p1[1] * map_resolution) +  map_y_min;
    //     Vector2f p1 (p1_x, p1_y);

    //     line2f path_line(p0, p1);
    //     path.emplace_back(path_line);
    //     curr_p0 = curr_p1;
    //     curr_p1 = next;
    //     curr_direction = new_direction;
    //   } else {
    //     // Extend the line
    //     curr_p1 = next;
    //   }
    //   index++;
    // }
    // // Add final line
    // float p0_x = (curr_p0[0] * map_resolution) +  map_x_min;
    // float p0_y = (curr_p0[1] * map_resolution) +  map_y_min;
    // Vector2f p0 (p0_x, p0_y);

    // float p1_x = (curr_p1[0] * map_resolution) +  map_x_min;
    // float p1_y = (curr_p1[1] * map_resolution) +  map_y_min;
    // Vector2f p1 (p1_x, p1_y);
    // line2f path_line(p0, p1);
    // path.emplace_back(path_line);

    // printf("Finished with plan!\n");
    // printf("Lines in plan: %lu\n", path.size());

    // DrawPlan();
} 

void Navigation::VisualizeLatticePath() {
  
  // visualization::DrawLine(line.p0, line.p1,0xFF0000,global_viz_msg_);
  if (path_states.size() == 0)
    return;

  // Vector2f cumulative_transform(0,0);
  // float cumulative_heading = 0;
  for (unsigned i = 0; i < path_states.size()-1; i++) {
    // draw lines for now
    State s1 = path_states[i].state;
    State s2 = path_states[i+1].state;

    Vector2f p0(s1.x, s1.y);
    Vector2f p1(s2.x, s2.y);

    if (s1.theta == s2.theta) {
      visualization::DrawLine(p0, p1, 0xFF0000, global_viz_msg_);
    } else {
      Eigen::Vector2f center;
      float start_angle = 0;
      float end_angle = 0;
      if (fEquals(s1.theta, 0) || fEquals(s1.theta, M_PI) || fEquals(s1.theta, -M_PI)) {
        // Going x direction
        center[0] = s1.x;
        center[1] = s2.y;
         
         // Turning left
         if( fEquals(s2.theta - s1.theta, M_PI/2)) {

          // If moving in positive x direction
          if (s2.x > s1.x)
          {
            start_angle = -M_PI / 2;
            end_angle = 0;
          }

          else
          {
            start_angle = M_PI/2;
            end_angle = M_PI;
          }

        }
        // Turning right 
        else {
          if (s2.x > s1.x)
          {
            start_angle = 0;
            end_angle = M_PI / 2;
          }
          
          // When moving in negative x direction
          else{
            start_angle = M_PI;
            end_angle = -M_PI/2;
          }
        }

      } else {
        // Going y direction
        center[0] = s2.x;
        center[1] = s1.y;


        if( fEquals(s2.theta - s1.theta, M_PI/2)) {
        // Turning left

          if (s2.y > s1.y)
          {
            start_angle = 0;
            end_angle = M_PI / 2;
          }

          else{
            start_angle = M_PI;
            end_angle = -M_PI/2;
          }
          

        } else {
          // Turning right

          if (s2.y > s1.y)
          {
            start_angle = M_PI/2;
            end_angle = M_PI;
          }

          else{
            start_angle = -M_PI/2;
            end_angle = 0;
          }
        }

      }

      float radius = 1.0;
      visualization::DrawArc(center, radius, start_angle, end_angle, 0xFF0000, global_viz_msg_);
    }
  }
}


float Navigation::heuristic(Vector2f current, Vector2f goal) {
  return (current - goal).norm();
}

float Navigation::LatticeHeuristic(const struct State& current, const struct State& goal) {
  Vector2f cur_loc (current.x, current.y);
  Vector2f goal_loc (goal.x, goal.y);

  return (cur_loc - goal_loc).norm();
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  total_distance += (loc - robot_loc_).norm();

  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;


  // printf("Location: %lf %lf\n", loc[0], loc[1]);
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   float time) {
  point_cloud_ = cloud;                                     
}

void Navigation::TransformPointCloud(float dx, float dy, float theta) {
  // Transform point cloud using translation and rotation angle
  for (auto point : point_cloud_) {
    point[0] = point[0] + dx;
    point[1] = point[1] + dy;

    float x = point[0];
    float y = point[1];

    point[0] = x * cos(theta) - y * sin(theta);
    point[1] = y * cos(theta) + x * sin(theta);
  }

}

// Finds max distance robot can travel for the curvature specificed in option.curvature
float Navigation::getTravellableDistance(struct PathOption& option)
{
  float curvature = option.curvature;
  float res = INF;

  // Calculate the steering angle from curvature
  option.theta = atan(WHEELBASE * curvature);
  for (auto point : point_cloud_)
  {
    float distance = GetMaxDistance(option, point);
    res = std::min(distance, res);
  }

  option.free_path_length = res;
  return res;
}

// Calculates score for given curvature based on free path length, clerance, and distance to goal
void Navigation::scorePath(struct PathOption& option) {
  // Calls to functions population option object
  getTravellableDistance(option);
  TrimDistanceToGoal(option);
  GetClearance(option);
  
  option.score = option.free_path_length + CLEARANCE_WEIGHT * option.clearance + -GOAL_WEIGHT * option.distance_to_goal;
}

// Function returns optimal curvature for robot to take in the range [-1.0, 1.0]
float* Navigation::getBestCurvature() {
  float curvature = -1.0;
  float delta_c = 0.1;
  float best_curvature = 0.0;
  float dist = 0.0;
  float best_score = 0;

  while (curvature <= 1.0)
  {
    struct PathOption option;
    option.curvature = curvature;

    // Calculate score for given curvature and keep track of best scoring curvature
    scorePath(option);
    if (option.score > best_score) {
      best_curvature = option.curvature;
      best_score = option.score;
      dist = option.free_path_length;
    }

    curvature += delta_c;
  }

  return new float[2] {best_curvature, dist};
}

float* Navigation::GetLatticeAction () {
  float curvature;
  if (!plan_active) {
    if (!start_plan) {
      return new float[2] {0, 0};
    }

    // Start a new plan
    total_distance = 0;
    curvature = path_states[0].curve.curvature;

    // bool adding_distance = true;
    unsigned i = 1;

    float ret_distance = path_states[0].curve.translation.norm();
    bool direction = path_states[0].curve.backwards;

    while (i < path_states.size()) {
      if (direction == path_states[i].curve.backwards) {
        ret_distance += path_states[i].curve.translation.norm();
      } else {
        break;
      }
      i++;
    }
    printf("Starting new plan: %f %f\n", curvature, ret_distance);
    plan_active = true;
    start_plan = false;

    // if going forwards
    if (direction == false)
      return new float[2] {curvature, ret_distance};
    else 
      return new float[2] {curvature, -ret_distance};




  } else {
    // Continue with active plan
    
    // Figure out where in plan we are
    unsigned i = 0; 
    float accumulated_distance = 0;
    while (i < path_states.size() && accumulated_distance + path_states[i].curve.translation.norm() < total_distance) {
      i++;
      accumulated_distance += path_states[i].curve.translation.norm();
    }

    if (accumulated_distance >= total_distance) {
      printf("End of path\n");
      // End of path
      plan_active = false;
      float zero = 0.0;
      return new float[2] {zero, zero};
    }

    // Now we know that we are leftover_distance through state i
    float leftover_distance = total_distance - accumulated_distance;
    float ret_distance = path_states[i].curve.translation.norm() - leftover_distance;

    bool direction = path_states[i].curve.backwards;

    float curvature = path_states[i].curve.curvature;

    i++;
    while (i < path_states.size()) {
      if (direction == path_states[i].curve.backwards) {
        ret_distance += path_states[i].curve.translation.norm();
      } else {
        break;
      }
      i++;
    }



    printf("Continuing plan: %f %f\n", curvature, ret_distance);

    // if going forwards
    if (direction == false)
      return new float[2] {curvature, ret_distance};
    else 
      return new float[2] {curvature, -ret_distance};
  }
}

float* Navigation::Simple1DTOC()
{
  // Use curvature and distance values to implement 1D Time optimal control loop at this time step
  float* action = GetLatticeAction();
  float curvature = action[0];
  float dist = action[1];
  bool backwards = false;
  if (dist < 0) {
    dist = -dist;
    backwards = true;
  }

  // float curvature = 0;
  // float dist = 0; 

  if (VISUALIZE) {
    DrawCar();
    DrawArcs(curvature, dist);
    // DrawPlan();
    VisualizeLatticePath();
  }
  float zero = 0.0;
  // Distance needed to deccelerate
  float x3 = pow(robot_vel_[0],2) / MAX_DECEL;
  // accelerate towards vmax
  if (robot_vel_[0] < MAX_VELOCITY && dist >= x3)
  {
    float new_v = robot_vel_[0] + MAX_ACCEL * 1/20;
    if (backwards) {
      return new float[2] {curvature, std::max((float)0.2, -new_v)};  
    } else {
      return new float[2] {curvature, std::max((float)0.2, new_v)}; 
    }
  }
  
  // Cruise at max velocity
  if (robot_vel_[0] == MAX_VELOCITY && dist >= x3) 
  {
    if (backwards) {
      return new float[2] {curvature, MAX_VELOCITY};
    } else {
      return new float[2] {curvature, -MAX_VELOCITY};
    }
  }
  
  // Stop
  float new_v = robot_vel_[0] - MAX_DECEL * 1/5;
  if (backwards) {
    return new float[2] {curvature, std::min(-new_v, zero)};
  } else {
    return new float[2] {curvature, std::max(new_v, zero)};
  }
}

void Navigation::SetGoal()
{
  // Sets Goal based on circle itersection with path
  // Circle centered at (robot_loc_[0], robot_loc[1]) with a radius
  GOAL = Vector2f(25, 0);
  Vector2f temp_goal;
  Vector2f temp_goal_local;
  for (auto line : path)
  {

      float dist1 = pow((pow(line.p1.x() - robot_loc_[0], 2) + pow(line.p1.y() - robot_loc_[1], 2)), 0.5);
      float dist0 = pow((pow(line.p0.x() - robot_loc_[0], 2) + pow(line.p0.y() - robot_loc_[1], 2)), 0.5);

      // Set to end of segment inside circle. May be overwritten, or maybe not if it is the last segment
      if (dist1 < radius) {
        temp_goal_local = line.p1;
        continue;
      }

      // Segment completely inside circle, we don't care
      if ((dist0 < radius && dist1 < radius))
        continue;

      // Segment begins and ends outside the circle - we might care
      // if (dist0 > radius && dist1 > radius) {
      float dis;
      Vector2f point;
      bool inter = geometry::FurthestFreePointCircle (line.p0, line.p1, robot_loc_, radius, &dis, &point);
      if (!inter)
        continue;

      Vector2f d = line.p1 - line.p0;
      Vector2f f = line.p0 - robot_loc_;

      float a = d.dot( d );
      float b = 2*f.dot( d );
      float c = f.dot( f ) - radius*radius;

      float discriminant = b*b-4*a*c;

      discriminant = sqrt( discriminant );


      // float t1 = (-b - discriminant)/(2*a);
      float t2 = (-b + discriminant)/(2*a);
      temp_goal_local = line.p0 + d*t2;

      continue; 
  } 
  
  // visualization::DrawCross(temp_goal_local, 1.0, 0x000000, global_viz_msg_);

  temp_goal = GlobalToRobot(temp_goal_local);

  // visualization::DrawCross(temp_goal, 1.0, 0x000FFF, local_viz_msg_);
  GOAL = temp_goal;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  // Clear previous visualizations.
  if (iteration == 0) {
    obstacle = Eigen::Vector2f(-19.3, 8.0);
  }

    visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  
  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // if (iteration % 100 == 0) {
    visualization::ClearVisualizationMsg(local_viz_msg_);
    // TODO
    // Predict where robot will be
    // Shift point cloud
    // float dx;
    // float dy;
    // float theta;

    // float time = 0;
    // Latency Compensation loop where point cloud is transformed and speed and curvature is predicted
    // while (time < LATENCY) {
    //   Eigen::Vector2f delta = GetTranslation(previous_velocity, previous_curvature, time);
    //   dx = delta[0];
    //   dy = delta[1];
    //   SetGoal();
    //   theta = GetRotation(previous_velocity, previous_curvature, time);
    //   TransformPointCloud(dx, dy, theta);
    //   float* res = Simple1DTOC();
    //   previous_curvature = res[0];
    //   previous_velocity = res[1];
    //   time += ((float) 1/20);
    // }
    float* res = Simple1DTOC();

    drive_msg_.curvature = res[0];
    drive_msg_.velocity = res[1];
    // drive_msg_.curvature = 0;
    // drive_msg_.velocity = -1;

  // for (auto point : point_cloud_) {
  //   visualization::DrawPoint(point,0x4287f5,local_viz_msg_);
  // }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  iteration++;
}

void Navigation::TrimDistanceToGoal (struct PathOption& option) {
  // Take a maximum free path length and trim it to closest point of approach

  // special case for straight
  if (fEquals(option.theta,0)) {
    float new_max_dist = GOAL(0);
    option.free_path_length = std::min(new_max_dist, option.free_path_length);
    option.distance_to_goal = GOAL(0) - option.free_path_length;
  } else {
    // Gets the angle between current pos and goal
    float phi = GetAngleBetweenVectors(-option.CoT, GOAL - option.CoT);
    float new_max_dist = phi * option.radius;

    option.free_path_length = std::min(new_max_dist, option.free_path_length);
    option.distance_to_goal = abs((GOAL - option.CoT).norm() - option.radius);
  }
}

// Returns Rotation angle for transforming point cloud
float Navigation::GetRotation(float velocity, float curvature, float time) {
  float theta = atan(WHEELBASE * curvature);
  float radius = WHEELBASE / tan(theta/2);
  Vector2f CoT(0,radius);

  float distance_travelled = velocity * time;
  float distance_angle = distance_travelled / radius;

  return (M_PI / 2) - distance_angle;
}

// Returns Translation displacement for transforming point cloud
Eigen::Vector2f Navigation::GetTranslation(float velocity, float curvature, float time) {
  float theta = atan(WHEELBASE * curvature);
  float radius = WHEELBASE / tan(theta/2);
  Vector2f CoT(0,radius);

  float distance_travelled = velocity * time;
  float distance_angle = distance_travelled / radius;
  Eigen::Vector2f B(0,0);

  float ac = radius;
  float ab = radius;
  float bc = 2 * radius * sin(distance_angle/2.0);

  float c_y = (pow(ab,2) + pow(ac,2) - pow(bc,2)) / (2 * ab);
  float c_x = sqrt(pow(ac,2) - pow(c_y,2));

  return Eigen::Vector2f(c_x, c_y);
}


// Returns rotation displacement of the curve
float Navigation::GetCurveRotation(float distance, float curvature) {
  float theta = atan(WHEELBASE * curvature);
  float radius = WHEELBASE / tan(theta/2);
  Vector2f CoT(0,radius);

  float distance_angle = distance / radius;

  return distance_angle;
}

Eigen::Matrix2f Navigation::GetRotationMatrix (const float angle) {
  Eigen::Matrix2f rot;
  rot(0,0) = cos(angle);
  rot(0,1) = -sin(angle);
  rot(1,0) = sin(angle);
  rot(1,1) = cos(angle);
  return rot;
}

// Returns Translation displacement of the curve
Eigen::Vector2f Navigation::GetCurveTranslation(float distance, float curvature) {
  float theta = atan(WHEELBASE * curvature);
  float radius = WHEELBASE / tan(theta/2);
  Vector2f CoT(0,radius);

  // float distance_travelled = velocity * time;
  float distance_angle = distance / radius;
  Eigen::Vector2f B(0,0);


  // float ac = radius;
  // float ab = radius;
  float bc = 2 * radius * sin(distance_angle/2.0);

  Vector2f translation(bc, 0);

  Eigen::Matrix2f rot = GetRotationMatrix(distance_angle);
  printf("Total distance: %f\n", bc);


  return rot * translation;


  // float c_y = (pow(ab,2) + pow(ac,2) - pow(bc,2)) / (2 * ab);
  // float c_x = sqrt(pow(ac,2) - pow(c_y,2));


  // return Eigen::Vector2f(c_x, c_y);
}

void Navigation::GetClearance (struct PathOption& option) {
  float width = WIDTH + SAFETY_MARGIN * 2;
  float length = LENGTH + SAFETY_MARGIN * 2;
  float wheelbase = WHEELBASE;

  float radius = option.radius;
  Vector2f CoT(0,radius);

  // min and max radius of swept volume
  float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  float min_radius = radius - width/2.0;

  float min_clearance = INF;

  // The point closest to car defines the min clearance
  for (auto point : point_cloud_) {
    Eigen::Vector2f point_vector = point - CoT;
    float point_radius = point_vector.norm();
    
    // See if point is at a reasonable angle
    Eigen::Vector2f car_vector (0, -radius);
    float point_angle = GetAngleBetweenVectors(car_vector, point_vector);
    float distance_angle = option.free_path_length / radius;

    // If the point is outside of the distance traversed by car, we don't care
    if (point_angle < distance_angle) {

      if (point_radius >= min_radius && point_radius <= max_radius) {
        // collides - we don't care about this one
        // also shouldn't happen, but might if car is overshooting
        min_clearance = 0;
        continue;
      }


        // Figure out if point is on inside or outside of swept volume 
        float inside_distance = abs(point_radius - min_radius);
        float outside_distance = abs(point_radius - max_radius);

        // point is close to arc that is travelled
        min_clearance = std::min(min_clearance, std::min(inside_distance, outside_distance));
    } else {
      float angle_difference = point_angle - distance_angle;
      float inside_distance = sqrt(pow(radius,2) + pow(min_radius,2) - 2 * radius * min_radius * cos(angle_difference));
      float outside_distance = sqrt(pow(radius,2) + pow(max_radius,2) - 2 * radius * max_radius * cos(angle_difference));
      min_clearance = std::min(min_clearance, std::min(inside_distance, outside_distance));
    }
  }

  option.clearance = min_clearance;
}

float Navigation::GetMaxDistanceStraight(Eigen::Vector2f point) {
  float l = LENGTH + SAFETY_MARGIN * 2;
  float w = WIDTH + SAFETY_MARGIN * 2;
  float wb = WHEELBASE;

  float car_front = wb + (l - wb)/2;

  // Get the bounding box of swept volume of car
  Eigen::Vector2f bbox_min (0, -w / 2.0);
  Eigen::Vector2f bbox_max (INF, w/2);

  // Check if point is within bbox
  if (point(0) >= bbox_min(0) && point(0) <= bbox_max(0) && point(1) >= bbox_min(1) && point(1) <= bbox_max(1)) { 
    // distance between point and car front is the max free path
    float distance = point(0) - car_front;

    // Sometimes we end up with negative distance if car overshot and point is within safety margin
    return distance; 
  }

  // Point does not collide - return infinity (maybe this is bad)
  return INF;
}
float Navigation::GetMaxDistance(struct PathOption& option, Eigen::Vector2f point) { 
  float theta = option.theta;

  // If angle is 0, use straight special case
  if (fEquals(theta, 0.0)) {
    return GetMaxDistanceStraight(point);
  }

  // just flip calculation if theta is to the right
  if (theta < 0) {
    theta = -theta;
    point(1) = -point(1);
  }

  float width = WIDTH + SAFETY_MARGIN * 2;
  float length = LENGTH + SAFETY_MARGIN * 2;
  float wheelbase = WHEELBASE;

  float radius = wheelbase / tan(theta/2);
  Vector2f CoT(0,radius);

  // hacky, saving values for later use
  if(option.theta < 0) {
    option.CoT = Eigen::Vector2f(0, -radius);
  } else {
    option.CoT = CoT;
  }
  option.radius = radius;

  // Get the min and max radius of the swept volume of car
  float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  float min_radius = radius - width/2.0;
  
  // get the radius between obstacle and CoT
  auto point_vec = point - CoT;
  float point_radius = abs(point_vec.norm());

  // If point_radius is within swept volume, means it collides
  if (point_radius >= min_radius && point_radius <= max_radius) {
    // Get alpha + beta
    // alpha = angle between current base_link and base_link after driving max length
    // beta = angle between base_line and point on car that collides
    // alpha + beta can be easily found by getting angle between base_link and point
    auto down_vec = Eigen::Vector2f(0, -radius);
    float ab = GetAngleBetweenVectors(down_vec, point_vec);

    // Use inner corner radius to determine whether collision is with front or side
    float inner_corner_radius = sqrt(pow(min_radius,2) + pow(length - (length - wheelbase)/2.0,2));
    float beta = 0;

    // Use trig to get angles - don't bother calculating actual points (from class)
    if (point_radius >= min_radius && point_radius <= inner_corner_radius) {
      // collision with side
      float y = radius - width/2;
      beta = acos(y / point_radius);

    } else {
      // collision with front
      float x = length - ((length - wheelbase)/2.0);
      beta = asin(x / point_radius);
    }

    // Alpha is angle that we travel
    
    float alpha = ab - beta;
    
    float path_length = radius * alpha;
    if (path_length < 0) {
      option.angle_travelled = 0;
      return 0.0;
    } else {
      option.angle_travelled = alpha;
      return radius * alpha;
    }
  }
  option.angle_travelled = M_PI * 2 / 4;
  // If no collision, return a quarter of the circle
  return M_PI*2 * radius / 4;
}

float Navigation::GetAngleBetweenVectors (Eigen::Vector2f a, Eigen::Vector2f b) {
  float angle = acos(a.dot(b) / (a.norm() * b.norm()));
  if (angle < 0) {
    angle = (M_PI * 2) + angle;
  }
  return angle;
}

void Navigation::DrawPlan() {
  for (line2f line : path) {
  visualization::DrawLine(line.p0, line.p1,0xFF0000,global_viz_msg_);
  }

  visualization::DrawArc(robot_loc_,
             radius,
             0.0,
             2 * M_PI,
             0x00000,
             global_viz_msg_);
}

void Navigation::DrawCar() {
  float l = LENGTH + SAFETY_MARGIN * 2;
  float w = WIDTH + SAFETY_MARGIN * 2;
  float wb = WHEELBASE;

  Eigen::Vector2f bbox_min (0 - (l - wb)/2, -w / 2.0);
  Eigen::Vector2f bbox_max ((wb + (l - wb)/2), w / 2);
  visualization::DrawLine(bbox_min, Eigen::Vector2f(bbox_min(0),bbox_max(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_min, Eigen::Vector2f(bbox_max(0),bbox_min(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_max, Eigen::Vector2f(bbox_min(0),bbox_max(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_max, Eigen::Vector2f(bbox_max(0),bbox_min(1)),0x000000,local_viz_msg_);
}

void Navigation::DrawArcs(float curvature, float dist) {
  if (fEquals(curvature, 0.0)) {
    visualization::DrawLine(Eigen::Vector2f(CAR_FRONT,0), Eigen::Vector2f(dist+CAR_FRONT,0), 0xff0000, local_viz_msg_);
    return;
  }
  if (curvature > 0) {
    // these might be slightly off but you get the gist
    float theta = atan(WHEELBASE * curvature);
    float radius = WHEELBASE / tan(theta/2);
    Vector2f CoT(0,radius);
    visualization::DrawArc(CoT, radius, -M_PI / 2, -M_PI / 2 + (dist/radius), 0xff0000, local_viz_msg_);


  } else {
      float theta = atan(WHEELBASE * -curvature);
      float radius = WHEELBASE / tan(theta/2);
      Vector2f CoT(0,-radius);
      visualization::DrawArc(CoT, radius, (M_PI / 2) - (dist/radius), M_PI / 2, 0xff0000, local_viz_msg_);
  }
}

Eigen::Vector2f Navigation::GlobalToRobot(Eigen::Vector2f point) {

  // Translate global frame coords to robot coords

  float angle = -robot_angle_;
  Eigen::Matrix2f rot;
  rot(0,0) = cos(angle);
  rot(0,1) = -sin(angle);
  rot(1,0) = sin(angle);
  rot(1,1) = cos(angle);

  auto translated_point = rot * (point - robot_loc_);

  return translated_point;

}


}  // namespace navigation
