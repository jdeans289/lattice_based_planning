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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include <map>
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"
#include <stdexcept>

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;


DEFINE_double(num_particles, 50, "Number of particles");
const float kEpsilon = 1e-5;

bool fEquals (float a, float b) {
  return (a >= b - kEpsilon && a <= b + kEpsilon);
}

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

Eigen::Vector2f ParticleFilter::LaserScanToPoint(float angle, float distance) {
    Eigen::Vector2f point;
    point[0] = laser_x_offset + distance * cos(angle);
    point[1] = distance * sin(angle);

    return point;
}

// translate from robot frame to global frame
Eigen::Vector2f ParticleFilter::RobotToGlobal (Eigen::Vector2f point, const Vector2f& loc, const float angle) {
  Eigen::Matrix2f rot = GetRotationMatrix(angle);

  auto translated_point = (rot * point) + loc;
  return translated_point;
}

// translate from global frame to robot frame
Eigen::Vector2f ParticleFilter::GlobalToRobot (Eigen::Vector2f point, const Vector2f& loc, const float angle) {
  Eigen::Matrix2f rot = GetRotationMatrix(-angle);

  auto translated_point = rot * (point - loc);
  return translated_point;
}

Eigen::Matrix2f ParticleFilter::GetRotationMatrix (const float angle) {
  Eigen::Matrix2f rot;
  rot(0,0) = cos(angle);
  rot(0,1) = -sin(angle);
  rot(1,0) = sin(angle);
  rot(1,1) = cos(angle);
  return rot;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);

  float angle_delta = (angle_max - angle_min) / num_ranges;
  float theta = angle_min;

  // Create the laser scan points
  for (int i = 0; i < num_ranges; i += laser_point_trim) {
    // Initialize with max range
    Vector2f robot_point = LaserScanToPoint(theta, range_max);
    Vector2f global_point = RobotToGlobal(robot_point, loc, angle);

    // compare ray with each line on map
    // Optimize this with AABB if time permits
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];
      
      // Create the scan line - from robot location to the end of the scanner ray aka point of point cloud
      line2f scan_line(loc.x(), loc.y(), global_point.x(), global_point.y());

      // Intersect them
      Vector2f intersection_point;
      bool intersects = map_line.Intersection(scan_line, &intersection_point);

      // If there is an intersection - then this scanner ray hits a wall
      if (intersects) {
        // Make a new line that goes from robot to new intersection (wall)
        line2f intersection_line(loc.x(), loc.y(), intersection_point.x(), intersection_point.y());

        // Make sure the intersection is actually shorter than the current line but longer than min range
        if (intersection_line.Length() < scan_line.Length() && intersection_line.Length() > range_min) {
          // Update the point to be this wall
          global_point = intersection_point;
        }
      }
    }

    // After we have compared against all of the walls, change it back to robot frame and set scan
    scan[i] = GlobalToRobot(global_point, loc, angle);

    // go to next point
    theta += laser_point_trim*angle_delta;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  Particle p = *p_ptr;

  // Get the scan that would be expected if the robot truly is at this location
  vector<Vector2f> predicted_scan;
  GetPredictedPointCloud(p.loc, p.angle, num_scans_predicted, range_min, range_max, angle_min, angle_max, &predicted_scan);
  
  // Calculating the log likelihood
  double log_likelihood = 0;
  std::vector<double> differences;

  // Angle delta
  float angle_delta = (angle_max - angle_min) / ranges.size();
  float angle = angle_min;

  // Calculate difference from expect point cloud for each point in real point cloud
  for (unsigned index = 0; index < ranges.size(); index+=laser_point_trim) {
    float true_range = ranges[index];

    // If there's some error in the reading, skip it
    if (true_range > range_max || true_range < range_min) 
      continue;

    // Get the actual vs predicted point for this scan value
    Vector2f true_point = LaserScanToPoint(angle, true_range);
    Vector2f predicted_point = predicted_scan[index];

    // s_i - s_hat_i
    Eigen::Vector2f difference = true_point - predicted_point;

    // Robust observation likelihood - trim difference if it's too big or small
    // difference[0] = std::max(difference[0], -d_short[0]);
    // difference[1] = std::max(difference[1], -d_short[1]); 

    // difference[0] = std::min(difference[0], d_long[0]); 
    // difference[1] = std::min(difference[1], d_long[1]); 

    differences.push_back(difference.norm());

    // 
    // log_likelihood += pow(difference.norm(), 2) / pow(update_variance, 2);
    
    // Move to the next laser point
    angle += laser_point_trim*angle_delta;
  }

  std::sort(differences.begin(), differences.end());

  int begin = differences.size() * 0.1;
  int end   = differences.size() * 0.9;

  for (int i = begin; i < end; i++) {
    log_likelihood += pow(differences[i], 2) / pow(update_variance, 2);
  }

  // overall weight is the -gamma * sum of log likelihoods
  p_ptr->weight = -gamma * log_likelihood;
}

void ParticleFilter::NormalizeWeights() {
  // Make sure we don't do anything over empty list
  if (particles_.size() < 1) 
    return;

  double weight_sum = 0;
  // Find Max Weight
  double max = particles_[0].weight;
  for (auto p : particles_) {
    if (p.weight > max && !fEquals(p.weight, 0.0))
      max = p.weight;
  }

  // scale every weight
  for (auto& p : particles_) {
    p.weight -= max;
  }

  // Now normalize
  // need the un-logged sum 
  for (auto p : particles_) {
    weight_sum += exp(p.weight);
  }

  double normalized_sum = 0;
  if (!fEquals(weight_sum, 0.0)) {
    for (auto& p : particles_) {
      // normalize weights
      p.weight = exp(p.weight) / weight_sum;
      normalized_sum += p.weight;
    }
    // printf("Normalized sum: %lf\n", normalized_sum);
  } else {
    // If we're here something messed up
    printf("Weights total to 0\n");
    for (auto& p : particles_) {
      p.weight = 1.0 / (float)particles_.size();
    }
  }
}

int ParticleFilter::SearchBins(vector<float>& bins, float sample) {
  for (unsigned i = 0; i < bins.size(); i++) {
    if (sample <= bins[i] && (i == 0 || sample > bins[i-1])) {
      return i;
    }
  }
  return -1;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  if (particles_.size() < 1)
    return;

  printf("Resampling\n");
  vector<Particle> new_particles;

  vector<float> bins;
  double running_sum = 0;

  // Create the bins
  for (unsigned i = 0; i < particles_.size(); i++) {
    running_sum += particles_[i].weight;
    bins.push_back(running_sum);
  }


  float sample = rng_.UniformRandom(0, 1);
  for (unsigned i = 0; i < particles_.size(); i++) {
    
    // Uncomment for low variance resampling
    // if (total_time < 1)
    sample = rng_.UniformRandom(0, 1);
    int bin_index = SearchBins(bins, sample);
    if (bin_index == -1) {
      // Shouldn't be here
      printf("Bins are NaN\n");
      return;
    }

    new_particles.emplace_back(particles_[bin_index]);

    // For low variance resampling
    float oon = 1/((float) (1.0 * num_initial_particles));
    sample += oon;
    if (sample >= 1)
      sample -= 1;

    // printf("%d ", bin_index);
  }

  particles_ = new_particles;

    // Set all the weights back to equal
  for (Particle& p : particles_) {
    p.weight = 1.0 / (double)new_particles.size();
  }
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  num_scans_predicted = ranges.size();
  total_time += 1;

  // If we have travlled at least a certain distance and angle, update
  if (distance_travelled < 0 || angle_travelled < 0) {
    printf("UPDATE\n");
    // Update the weights of the particles
    for (auto& p_ptr : particles_) {
      Update(ranges, range_min, range_max, angle_min, angle_max, &p_ptr);
    }
    // Normalize weights to be 0-1
    NormalizeWeights();

    num_updates -= 1;

    distance_travelled = distance_travelled_og;
    angle_travelled = angle_travelled_og;
  }

  if (num_updates <= 0)
  {
    printf("\nRESAMPLE\n");
    Resample();
    num_updates = num_updates_og;
  }

}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // For first time step when odometry is not known:
  if (prev_odom_loc[0] == (float)-1000) {
    // Todo do you need deep copy?
    prev_odom_loc[0] = odom_loc[0];
    prev_odom_loc[1] = odom_loc[1];
    prev_odom_angle = odom_angle;
  } 

  else 
  {
    // Update distance and angle travelled
    float displacement = (odom_loc - prev_odom_loc).norm();
    distance_travelled -= displacement;
    angle_travelled -= abs(odom_angle - prev_odom_angle);

    for (unsigned int i = 0; i < particles_.size(); i++)
    {
      Particle particle = particles_[i];

      // T_2^Odom - T_1^Odom
      Vector2f delta_odom = odom_loc - prev_odom_loc;
      // R(-Theta_1^Odom)
      Eigen::Matrix2f rot = GetRotationMatrix(-prev_odom_angle);
      // R(theta_1^Map) * delta T_base_link
      Vector2f delta_T_bl = rot * delta_odom;
      // T_2^Map = T_1^Map + ...
      Vector2f translation = GetRotationMatrix(particle.angle) * delta_T_bl;
      Vector2f T_map = particle.loc + GetRotationMatrix(particle.angle) * delta_T_bl;

      float delta_theta_bl = odom_angle - prev_odom_angle;
      float theta_map = particle.angle + delta_theta_bl;
      
      // For x, y positions and angle sample Gaussian centered around next positive with odometry and std deviation k * odometry
      float next_x = rng_.Gaussian(T_map[0], k * abs(translation[0]));
      float next_y = rng_.Gaussian(T_map[1], k * abs(translation[1]));
      float next_theta = rng_.Gaussian(theta_map, k *  abs(delta_theta_bl));
      // float next_theta = rng_.Gaussian(theta_map, 0);

      particles_[i].loc[0] = next_x;
      particles_[i].loc[1] = next_y;
      particles_[i].angle = next_theta;
    }
    // Keep track of previous odometry
    prev_odom_loc[0] = odom_loc[0];
    prev_odom_loc[1] = odom_loc[1];
    prev_odom_angle = odom_angle;
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  printf("Initializing...\n");
  particles_.clear();
  double weight = 1.0 / num_initial_particles;
  for (int i = 0; i < num_initial_particles; i++) {
    float x = rng_.Gaussian(loc(0), initial_std_x);
    float y = rng_.Gaussian(loc(1), initial_std_y);
    float theta = rng_.Gaussian(angle, initial_std_theta);

    particles_.emplace_back(x, y, theta, weight);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // // Compute the best estimate of the robot's location based on the current set
  // // of particles. The computed values must be set to the `loc` and `angle`
  // // variables to return them. Modify the following assignments:
  if (particles_.size() > 0) {
    double x_sum = 0;
    double y_sum = 0;
    // double theta_sum = 0;
    // double weight_sum = 0;
    double cos_sum = 0.0;
    double sin_sum = 0.0;
    for (auto particle : particles_) {
      x_sum += particle.loc.x();
      y_sum += particle.loc.y();
      // theta_sum += particle.angle;
      // weight_sum += particle.weight;
      cos_sum += cos(particle.angle);
      sin_sum += sin(particle.angle);
    }

    double size = (double)particles_.size();
    if (total_time % 200 == 0)
    {
      Particle location = KMeansClustering(3, x_sum/size, y_sum/size);
      loc = location.loc;
    }
    else
      loc = Vector2f(x_sum/size, y_sum/size);

    // printf("Estimated Location: %f %f\n", loc[0], loc[1]);
    // throw std::invalid_argument( "received negative value" );
    angle = atan2(sin_sum / size,cos_sum / size);
  } else {
    loc = Vector2f(0, 0);
    angle = 0;
  }
}


Particle ParticleFilter::KMeansClustering(int k, float x_init, float y_init) const
{
  Particle mus[k];
  for (int i = 0; i < k; i++)
  {
    Particle p;
    p.loc[0] = rand() % k / 2 + x_init +  rand() %  -k / 2 ;
    p.loc[1] = rand() % k / 2  + y_init +  rand() %  -k / 2;
    // printf("MUS Location: %f %f\n", p.loc[0], p.loc[1]);
    mus[i] = p;
  }
  
  while (true)
  {
    // printf("K_MEANS\n");
    float new_mu_x[k];
    float new_mu_y[k];
    float counts[k];
    for (int i = 0; i < k; i++)
    {
      new_mu_x[i] = 0.0;
      new_mu_y[i] = 0.0;
      counts[i] = 0.0;
    }

    for (auto p : particles_)
    {
      float closest_mean = 10000000000;
      int label = 0;
      for (int i = 0; i < k; i++)
      {
          float distance = pow((p.loc - mus[i].loc).norm(), 0.5);
          // printf("POS: %f %f\n", p.loc[0], p.loc[1]);
          if (distance < closest_mean)
          {
            closest_mean = distance;
            label = i;
          }
      }

      new_mu_x[label] += p.loc.x();
      new_mu_y[label] += p.loc.y();
      counts[label] += 1.0;
    }

    // printf("NEW MU 0: %f %f %f\n", new_mu_x[0], new_mu_y[0], counts[0]);
    // printf("NEW MU 1: %f %f %f \n", new_mu_x[1], new_mu_y[1], counts[1]);
    // printf("NEW MU 2: %f %f %f\n", new_mu_x[2], new_mu_y[2], counts[2]);

    bool converged = true;
    Particle next_mus[k];
    int common_cluster = -1.0;
    float most_counts = -1.0;
    for (int i = 0; i < k; i++)
    {
      Particle next_p;
      float new_mean_x = new_mu_x[i]/counts[i];
      float new_mean_y = new_mu_y[i]/counts[i];
      next_p.loc.x() = new_mean_x;
      next_p.loc.y() = new_mean_y;
      next_mus[i] = next_p;
      // printf("DIFF: %f\n",pow((next_mus[i].loc - mus[i].loc).norm(), 0.5));
      if (abs(pow((next_mus[i].loc - mus[i].loc).norm(), 0.5)) > 0.001)
          converged = false;
        
      if (most_counts < counts[i])
      {
        common_cluster = i;
        most_counts = counts[i];
      }
      mus[i] = next_mus[i];
    }

    // printf("BEST CLUSTER: %f %f\n*****************\n", next_mus[common_cluster].loc[0], next_mus[common_cluster].loc[1]);
    if (converged)
      return next_mus[common_cluster];
  }
  return mus[0];
}


}  // namespace particle_filter
