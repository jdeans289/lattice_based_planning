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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"
#include <stdexcept>

#include <stdio.h>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "visualization/CImg.h"


using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

using cimg_library::CImg;
using cimg_library::CImgDisplay;


const float kEpsilon = 1e-5;

bool fEquals (float a, float b) {
  return (a >= b - kEpsilon && a <= b + kEpsilon);
}

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    p_odom_vector(0, 0),
    p_odom_angle(0) {}

void SLAM::PrintImage(float image[x_image_width][y_image_width])
{
  printf("Printing Image\n");
  for (int pixel_x = 0; pixel_x < x_image_width; pixel_x++)
  {
    for (int pixel_y = 0; pixel_y < y_image_width; pixel_y++)
      printf("[ %f ]", image[pixel_x][pixel_y]);
    
    printf("\n");
  }
}

void SLAM::PrintCube(int num_elems)
{
  printf("Printing Cube\n");
  int printed = 0;
  for (int pixel_x = 0; pixel_x < x_width; pixel_x++)
  {
    for (int pixel_y = 0; pixel_y < y_width; pixel_y++)
    {
      for (int pixel_theta = 0; pixel_theta < theta_width; pixel_theta++)
      {
        float dx = pixel_x * x_incr;
        float dy = pixel_y * y_incr;
        float dtheta = pixel_theta * theta_incr;
        printf("dx: %f dy: %f dtheta: %f --> %f\n", dx, dy, dtheta, cube[pixel_x][pixel_y][pixel_theta]);
        printed++;

        if (printed > num_elems)
          return;
      }
    }
  }
}



void SLAM::InitializeImage(float image[x_image_width][y_image_width])
{
  for (int pixel_x = 0; pixel_x < x_image_width; pixel_x++)
    for (int pixel_y = 0; pixel_y < y_image_width; pixel_y++)
      image[pixel_x][pixel_y] = 0.0;
}

void SLAM::ReinitializeCube()
{
  for (int pixel_x = 0; pixel_x < x_width; pixel_x++)
    for (int pixel_y = 0; pixel_y < y_width; pixel_y++)
      for (int pixel_theta = 0; pixel_theta < theta_width; pixel_theta++) {
          cube[pixel_x][pixel_y][pixel_theta] = 0.0;
          obsv[pixel_x][pixel_y][pixel_theta] = 0.0;
      }
}

struct Pose SLAM::MostLikelyPose()
{
  // If the first pose, the cumulative transformations are all 0 therefore best pose is dx = 0, dy = 0, dtheta = 0
  //  if (previous_pose.delta_x == -1000 && previous_pose.delta_y == -1000 && previous_pose.delta_theta == -1000)
  // {
  //   struct Pose first_pose = {
  //     0, 0, 0, 0
  //   };
  //   return first_pose;
  // }

  float best_x = 0;
  float best_y = 0;
  float best_theta = 0;
  float most_likely = -1000000000000000000000000000000.0;


  // Consult cube to find displacement that results in most likely pose
  for (int pixel_x = 0; pixel_x < x_width; pixel_x++)
  {
    for (int pixel_y = 0; pixel_y < y_width; pixel_y++)
    {
      for (int pixel_theta = 0; pixel_theta < theta_width; pixel_theta++)
      {   

          if (most_likely < cube[pixel_x][pixel_y][pixel_theta])
          {
            
            best_x = (pixel_x * x_incr) + x_min;
            best_y = (pixel_y * y_incr) + y_min;
            best_theta = (pixel_theta * theta_incr) + theta_min;
            most_likely = cube[pixel_x][pixel_y][pixel_theta];
          }
      }
    }
  }

  struct Pose new_pose = { best_x, best_y, best_theta, most_likely};

  // printf("BEST POSE: %f %f %f %f\n", best_x, best_y, best_theta, most_likely);
  return new_pose;
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
  
  // Find best pose with respect to pose 1 (using the cumulative_transform variable)
  *loc = Vector2f(cumulative_transform.delta_x, cumulative_transform.delta_y);
  *angle = cumulative_transform.delta_theta;
}

Eigen::Matrix2f SLAM::GetRotationMatrix (const float angle) {
  Eigen::Matrix2f rot;
  rot(0,0) = cos(angle);
  rot(0,1) = -sin(angle);
  rot(1,0) = sin(angle);
  rot(1,1) = cos(angle);
  return rot;
}


vector<Vector2f> SLAM::GetScanPointCloud(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max)
  {
    vector<Vector2f> points;

    float angle = angle_min;
    int range_index = 0;
    float angle_increment = (angle_max - angle_min) / ranges.size();

    // Convert laser scans to points
    while (angle <= angle_max) 
    {
      Vector2f point;
      float range = ranges[range_index];
      point[0] = laser_off + range * cos(angle);
      point[1] = range * sin(angle);

      points.push_back(point);
      angle += angle_increment;
      range_index += 1;
    }
    return points;
  }

void SLAM::GetBoundingBox(float bounds[4])
{
  float xmin = 10000000000000000;
  float ymin = 10000000000000000;
  float xmax = -1000000000000000;
  float ymax = -1000000000000000;

  for (auto& point : previous_scan)
  {
    xmin = std::min(point[0], xmin);
    ymin = std::min(point[1], ymin);
    xmax = std::max(point[0], xmax);
    ymax = std::max(point[1], ymax);
  }

  bounds[0] = xmin;
  bounds[1] = xmax;
  bounds[2] = ymin;
  bounds[3] = ymax;
  // printf("BOUNDS: %f %f %f %f", xmin, xmax, ymin, ymax);
} 

// Go from a point in the basis given by dx dy dtheta to 0,0,0
Eigen::Vector2f SLAM::TransformFromBase(Eigen::Vector2f point, float dx, float dy, float dtheta) {
  auto rot = GetRotationMatrix(dtheta);
  Vector2f translation (dx, dy);

  return rot*point + translation;
}

// Go from a point in 0,0,0 to basis given by dx dy dtheta
Eigen::Vector2f SLAM::TransformToBase(Eigen::Vector2f point, float dx, float dy, float dtheta) {
  auto rot = GetRotationMatrix(-dtheta);
  Vector2f translation (-dx, -dy);

  return rot * (point + translation);
}

void SLAM::EvaluateMotionModel() {
  Vector2f translation_hat = p_odom_vector;
  float angle_hat = p_odom_angle;
  printf("Odom vector: %lf %lf\tAngle: %f\n", translation_hat[0], translation_hat[1], angle_hat);

  // printf("TRANSLATION: %f %f, Angle Disp: %f\n", translation_hat[0], translation_hat[1], angle_hat);
  // printf("DIST: %f\n", distance_travelled);

  // Try out all possible delta x, y, theta
  for (int pixel_x = 0; pixel_x < x_width; pixel_x++)
  {
    for (int pixel_y = 0; pixel_y < y_width; pixel_y++)
    {
      for (int pixel_theta = 0; pixel_theta < theta_width; pixel_theta++)
      {
        float dx = (pixel_x * x_incr) + x_min;
        float dy = (pixel_y * y_incr) + y_min;
        float dtheta = (pixel_theta * theta_incr) + theta_min;

        double prob = 0.0;
        float std_dev = k_1 * pow(pow(translation_hat[0], 2) + pow(translation_hat[1], 2), 0.5) + k_2 * (pow(angle_hat, 2));

        // Decoupled evaluation of multivariate gaussian where product is taken along x, y, and theta
        prob += (-0.5 * pow((dx - translation_hat[0])/std_dev, 2));
        prob += (-0.5 * pow((dy - translation_hat[1])/std_dev, 2));
        prob += (-0.5 * pow((dtheta - angle_hat)/std_dev, 2));

        // Motion model probability
        cube[pixel_x][pixel_y][pixel_theta] += prob;
      }
    }
  }
}

void SLAM::EvaluateObservationLikelihood(std::vector<Eigen::Vector2f> current_scan) {

    double image[x_image_width][y_image_width];
    // quit = true;
    // printf("Image resolution: %d x %d\n", im_rows, im_cols);

    for (unsigned int pixel_x = 0; pixel_x < x_image_width; pixel_x++) {
      for (unsigned int pixel_y = 0; pixel_y < y_image_width; pixel_y++) {
        image[pixel_x][pixel_y] = 0;
      }
    }


  CImg<float> image_real(x_image_width,y_image_width,1,1,0);

  int radius = 3;
  for (auto point : previous_scan) {
    int point_pixel_x = (point[0] - x_image_min) / x_image_incr;
    int point_pixel_y = (point[1] - y_image_min) / y_image_incr;

    if (point_pixel_x < 0 || point_pixel_x >= x_image_width || point_pixel_y < 0 || point_pixel_y >= y_image_width)
      continue;

    const unsigned char color[] = { 255,255,255 };
    image_real.draw_point(point_pixel_x,point_pixel_y,color);

    for (int i = -radius; i < radius; i++) {
      for (int j = -radius; j < radius; j++) {
        int x = point_pixel_x + i;
        int y = point_pixel_y + j;
        float x_val = point[0] + (x_image_incr * i);
        float y_val = point[1] + (y_image_incr * j);

        if (x < 0 || y < 0 || x >= x_image_width || y >= y_image_width) 
          continue;

        if (pow(x - point_pixel_x, 2) + pow(y - point_pixel_y, 2) <= pow(radius,2)) {
          image[x][y] += -0.5 * pow((x_val - point[0])/std_dev_sensor, 2);
          image[x][y] += -0.5 * pow((y_val - point[1])/std_dev_sensor, 2);
        }
      }
    }
  }

  float min = 1000000000000000000;
    for (unsigned int pixel_x = 0; pixel_x < x_image_width; pixel_x++) {
      for (unsigned int pixel_y = 0; pixel_y < y_image_width; pixel_y++) {
        if (image[pixel_x][pixel_y] < min)
          min = image[pixel_x][pixel_y];
      }
    }

  min = min - 10000000;
  for (unsigned int pixel_x = 0; pixel_x < x_image_width; pixel_x++) {
    for (unsigned int pixel_y = 0; pixel_y < y_image_width; pixel_y++) {
      if (fEquals(image[pixel_x][pixel_y], 0))
        image[pixel_x][pixel_y] = min;
    }
  }

  for (auto point : current_scan)
  {
    // To test with an offset, do it here
    // point[0] += 0.10;
    // point[1] -= 0.38;
    // auto rot1 = GetRotationMatrix(0.25);
    // point = rot1 * point;


    for (int pixel_theta = 0; pixel_theta < theta_width; pixel_theta++)
    {
      float dtheta = (pixel_theta * theta_incr) + theta_min;
      auto rot = GetRotationMatrix(dtheta);
      Vector2f rot_point = rot * point;

      for (int pixel_y = 0; pixel_y < y_width; pixel_y++)
      {
        for (int pixel_x = 0; pixel_x < x_width; pixel_x++)
        {
          float dx = (pixel_x * x_incr) + x_min;
          float dy = (pixel_y * y_incr) + y_min;

          Vector2f shifted_point = rot_point + Vector2f(dx,dy);

          int point_pixel_x = (shifted_point[0] - x_image_min) / x_image_incr;
          int point_pixel_y = (shifted_point[1] - y_image_min) / y_image_incr;

          if (point_pixel_x < 0 || point_pixel_x >= x_image_width || point_pixel_y < 0 || point_pixel_y >= y_image_width)
            continue;

          obsv[pixel_x][pixel_y][pixel_theta] += image[point_pixel_x][point_pixel_y];
        }
      }
    }
  }

  for (int pixel_x = 0; pixel_x < x_width; pixel_x++) {
    for (int pixel_y = 0; pixel_y < y_width; pixel_y++) {
      for (int pixel_theta = 0; pixel_theta < theta_width; pixel_theta++) {
        cube[pixel_x][pixel_y][pixel_theta] += obsv[pixel_x][pixel_y][pixel_theta];
      }
    }
  }
}

// Preconditions have been met - determine most likely pose and add scan to map
void SLAM::AddToMap(std::vector<Eigen::Vector2f> current_scan) {

  printf("Adding points to map\n");
  ReinitializeCube();

  Vector2f translation_hat = p_odom_vector;
  float angle_hat = p_odom_angle;
  printf("Odom vector: %lf %lf\tAngle: %f\n", translation_hat[0], translation_hat[1], angle_hat);

  // Do odometry first
  
  EvaluateMotionModel();
  EvaluateObservationLikelihood(current_scan);

  previous_scan = current_scan;
  best_pose = MostLikelyPose();
  printf("Best pose: %lf %lf %lf\n", best_pose.delta_x, best_pose.delta_y, best_pose.delta_theta);
  auto rot = GetRotationMatrix(cumulative_transform.delta_theta);
  Vector2f new_vec = rot * Vector2f(best_pose.delta_x, best_pose.delta_y);
  cumulative_transform.delta_x += new_vec[0];
  cumulative_transform.delta_y += new_vec[1];
  cumulative_transform.delta_theta += best_pose.delta_theta;
  previous_pose = best_pose;
  p_odom_vector = Vector2f(0,0);
  p_odom_angle = 0;

  printf("New pose: %lf %lf %lf\n", cumulative_transform.delta_x, cumulative_transform.delta_y, cumulative_transform.delta_theta);

  for (auto& point : previous_scan)
  {
    // Transform point to reference frame of pose 1
    Vector2f shifted_point = TransformFromBase(point, cumulative_transform.delta_x, cumulative_transform.delta_y, cumulative_transform.delta_theta);

    estimated_map.push_back(shifted_point);
  }
}


// x_2 = current
// x_1 previous

// estimate x_2 by trying out all x_1 + u
// Scan of x_2 ground truth and we compare that to the transformed x_1 laser scan

// Sum of gaussians approach
void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  // printf("Check1\n");

  // For initial pose
  if (previous_pose.delta_x == -1000 && previous_pose.delta_y == -1000 && previous_pose.delta_theta == -1000)
  {
    previous_pose.delta_x = 0;
    previous_pose.delta_y = 0;
    previous_pose.delta_theta = 0;
    vector<Vector2f> points = GetScanPointCloud(ranges, range_min, range_max, angle_min, angle_max);
    previous_scan = points;
    ReinitializeCube();
    return;
  }

  // If conditions met (0.5 m dist or 45 degrees rotated)
  if (distance_travelled <= 0.0 || angle_travelled <= 0)
  { 
    vector<Vector2f> current_scan = GetScanPointCloud(ranges, range_min, range_max, angle_min, angle_max);

    AddToMap(current_scan);
    distance_travelled = distance_travelled_og;
    angle_travelled = angle_travelled_og;
    return;
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }

  p_odom_vector += odom_loc - prev_odom_loc_;
  p_odom_angle += odom_angle - prev_odom_angle_;
  distance_travelled -= (odom_loc - prev_odom_loc_).norm();
  angle_travelled -= abs(odom_angle - prev_odom_angle_);
  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
  return;
}

vector<Vector2f> SLAM::GetMap() {
  return estimated_map;
}

}  // namespace slam
