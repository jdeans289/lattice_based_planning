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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;

  public:
    Particle() {
      loc = Eigen::Vector2f(0, 0);
      angle = 0.0;
      weight = 0.0;
    }


    Particle(float x, float y, float theta, double init_weight) {
      loc = Eigen::Vector2f(x,y);
      angle = theta;
      weight = init_weight;
    }
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  Eigen::Vector2f LaserScanToPoint(float angle, float distance);
  void NormalizeWeights();
  int SearchBins(std::vector<float>& bins, float sample);
  Eigen::Vector2f RobotToGlobal (Eigen::Vector2f point, const Eigen::Vector2f& loc, const float angle);
  Eigen::Vector2f GlobalToRobot (Eigen::Vector2f point, const Eigen::Vector2f& loc, const float angle);
  Eigen::Matrix2f GetRotationMatrix (const float angle);

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);
  
  Particle KMeansClustering(int k, float x_init, float y_init) const;

  vector_map::VectorMap map_;
  int laser_point_trim = 1;


 private:

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  const int num_initial_particles = 50;

  const double initial_std_x = 0.2;
  const double initial_std_y = 0.2;
  const double initial_std_theta = M_PI / 12;

  const double laser_x_offset = 0.18;

  int num_scans_predicted;

  // Standard deviation of the sensor
  // Seems pretty small?
  double update_variance = 0.15;

  // Account for correlation between rays on update step
  // 1    -> no correlation
  // 1/n  -> perfect correlation (n = number of rays)
  double gamma = 1 / 50.0;

  int visualize_particle_filter = 1;

  const float k = 1.0;
  const float odom_var_x = 0.1;
  const float odom_var_y = 0.1;
  const float odom_var_t = 0.1;



  // Added by us
  Eigen::Vector2f prev_odom_loc = Eigen::Vector2f(-1000, -1000);
  float prev_odom_angle = 0;

  float distance_travelled_og = .15; //.15 according to professor
  float distance_travelled = distance_travelled_og;
  float angle_travelled_og = .175; //.175 according to professor
  float angle_travelled = angle_travelled_og;


  int num_updates_og = 5;
  int num_updates = num_updates_og;

  Eigen::Vector2f d_short = Eigen::Vector2f(2, 2);
  Eigen::Vector2f d_long = Eigen::Vector2f(2, 2);
  int total_time = 0;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
